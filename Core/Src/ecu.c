/*
 * ecu.c
 *
 *  Created on: Mar 4, 2022
 *      Author: VHEMaster
 */

#include "main.h"
#include "delay.h"
#include "ecu.h"
#include "packets.h"
#include "flash.h"
#include "xCommand.h"
#include "csps.h"
#include "misc.h"
#include "adc.h"
#include "speed.h"
#include "sensors.h"
#include "outputs.h"
#include "injector.h"
#include "sst25vf032b.h"
#include "interpolation.h"
#include "math_misc.h"
#include "config.h"
#include "pid.h"

#include <string.h>
#include "arm_math.h"

#define ENRICHMENT_STATES_COUNT 3

typedef struct {
    union {
        struct {
            HAL_StatusTypeDef Load : 2;
            HAL_StatusTypeDef Save : 2;
            HAL_StatusTypeDef Init : 2;
        }Struct;
        uint8_t Byte;
    }Flash;
    union {
        struct {
            HAL_StatusTypeDef Save : 2;
            HAL_StatusTypeDef Load : 2;
        }Struct;
        uint8_t Byte;
    }Bkpsram;
    //TODO: add more diagnostic fields
}sStatus;

static GPIO_TypeDef * const gIgnPorts[4] = { IGN_1_GPIO_Port, IGN_2_GPIO_Port, IGN_3_GPIO_Port, IGN_4_GPIO_Port };
static const uint16_t gIgnPins[4] = { IGN_1_Pin, IGN_2_Pin, IGN_3_Pin, IGN_4_Pin };

static GPIO_TypeDef * const gInjChPorts[2] = { INJ_CH1_GPIO_Port, INJ_CH2_GPIO_Port };
static const uint16_t gInjChPins[2] = { INJ_CH1_Pin, INJ_CH2_Pin};

static sEcuTable gEcuTable[TABLE_SETUPS_MAX];
static sEcuParams gEcuParams;
static sEcuCorrections gEcuCorrections;
static sEcuCriticalBackup gEcuCriticalBackup;
static sStatus gStatus = {{{0}}};
static sParameters gParameters = {0};
static sForceParameters gForceParameters = {0};

static volatile uint8_t gEcuCriticalBackupWriteAccess = 0;
static volatile uint8_t gEcuIdleValveCalibrate = 0;
static volatile uint8_t gEcuIdleValveCalibrateOk = 0;
static volatile uint8_t gEcuInitialized = 0;
static volatile uint8_t gEcuCutoffProcessing = 0;

static sMathPid gPidIdleIgnition = {0};
static sMathPid gPidIdleAirFlow = {0};

static HAL_StatusTypeDef ecu_get_start_allowed(void)
{
  return gParameters.StartAllowed ? HAL_OK : HAL_ERROR;
}

static void ecu_set_start_allowed(HAL_StatusTypeDef allowed)
{
  gParameters.StartAllowed = allowed == HAL_OK ? 1 : 0;
}

static uint8_t ecu_get_table(void)
{
  return gParameters.CurrentTable;
}

static void ecu_set_table(uint8_t number)
{
  gParameters.CurrentTable = number;
}

static void ecu_config_init(void)
{
  int8_t status;

  memset(gEcuTable, 0, sizeof(gEcuTable));
  memset(&gEcuParams, 0, sizeof(gEcuParams));
  memset(&gEcuCorrections, 0, sizeof(gEcuCorrections));
  memset(&gEcuCriticalBackup, 0, sizeof(gEcuCriticalBackup));
  memset(&gStatus, 0, sizeof(gStatus));

  gStatus.Flash.Struct.Init = config_init();

  while(!(status = config_load_params(&gEcuParams))) {}
  if(status < 0) {
    gStatus.Flash.Struct.Load = HAL_ERROR;
    config_default_params(&gEcuParams);
    while(!(status = config_save_params(&gEcuParams))) {}
    if(status < 0) {
        gStatus.Flash.Struct.Save = HAL_ERROR;
    }
  }

  for(int i = 0; i < TABLE_SETUPS_MAX; i++) {
    while(!(status = config_load_table(&gEcuTable[i], i))) {}
    if(status < 0) {
      gStatus.Flash.Struct.Load = HAL_ERROR;
      config_default_table(&gEcuTable[i], i);
      while(!(status = config_save_table(&gEcuTable[i], i))) {}
      if(status < 0) {
          gStatus.Flash.Struct.Save = HAL_ERROR;
      }
    }
  }

  while(!(status = config_load_corrections(&gEcuCorrections))) {}
  if(status < 0) {
    gStatus.Flash.Struct.Load = HAL_ERROR;
    config_default_corrections(&gEcuCorrections);
    while(!(status = config_save_corrections(&gEcuCorrections))) {}
    if(status < 0) {
        gStatus.Flash.Struct.Save = HAL_ERROR;
    }
  }

  while(!(status = config_load_critical_backup(&gEcuCriticalBackup))) {}
  if(status < 0) {
    gStatus.Bkpsram.Struct.Load = HAL_ERROR;
    config_default_critical_backup(&gEcuCriticalBackup);
    while(!(status = config_save_critical_backup(&gEcuCriticalBackup))) {}
    if(status < 0) {
        gStatus.Bkpsram.Struct.Save = HAL_ERROR;
    }
  }
}

static float ecu_get_air_destiny(float pressure, float temperature)
{
  //Output in g/cc
  const float M = 29.0f;
  const float R = 8314462.61815324f;
  float g_cc3 = (pressure * M) / (R * (temperature + 273.0f));
  return g_cc3;
}

static void ecu_update_current_table(void)
{
  uint32_t table = 0;

  if(gEcuParams.isForceTable) {
      table = gEcuParams.forceTable;
  } else if(gEcuParams.isSwitchByExternal) {
    //TODO:
  } else {
    switch(gParameters.SwitchPosition) {
      case 0 : table = gEcuParams.switchPos0Table; break;
      case 1 : table = gEcuParams.switchPos1Table; break;
      case 2 : table = gEcuParams.switchPos2Table; break;
      default : break;
    }
  }

  gParameters.CurrentTable = table;
}

static inline void ecu_pid_update(uint8_t isidle)
{
  uint32_t table_number = gParameters.CurrentTable;
  sEcuTable *table = &gEcuTable[table_number];

  math_pid_set_clamp(&gPidIdleIgnition, table->idle_ign_deviation_min, table->idle_ign_deviation_max);
  math_pid_set_clamp(&gPidIdleAirFlow, 0, 255);

  if(isidle) {
    math_pid_set_koffs(&gPidIdleIgnition, table->idle_ign_to_rpm_pid_p, table->idle_ign_to_rpm_pid_i, table->idle_ign_to_rpm_pid_d);
    math_pid_set_koffs(&gPidIdleAirFlow, table->idle_valve_to_massair_pid_p, table->idle_valve_to_massair_pid_i, table->idle_valve_to_massair_pid_d);
  } else {
    math_pid_set_koffs(&gPidIdleIgnition, table->idle_ign_to_rpm_pid_p, 0, 0);
    math_pid_set_koffs(&gPidIdleAirFlow, table->idle_valve_to_massair_pid_p, 0, 0);
  }
}

static void ecu_pid_init(void)
{
  math_pid_init(&gPidIdleIgnition);
  math_pid_init(&gPidIdleAirFlow);
  ecu_pid_update(0);
}

static void ecu_update(void)
{
  static uint32_t adaptation_last = 0;
  static float fuel_consumed = 0;
  static float km_driven = 0;
  static uint32_t updated_last = 0;
  uint32_t now = Delay_Tick;
  float adapt_diff = DelayDiff(now, adaptation_last);
  float diff = DelayDiff(now, updated_last);
  uint32_t table_number = gParameters.CurrentTable;
  sEcuTable *table = &gEcuTable[table_number];
  sMathInterpolateInput ipRpm = {0};
  sMathInterpolateInput ipMap = {0};
  sMathInterpolateInput ipTemp = {0};
  sMathInterpolateInput ipSpeed = {0};
  sMathInterpolateInput ipThr = {0};
  sMathInterpolateInput ipVoltages = {0};

  sMathInterpolateInput ipFilling = {0};

  sMathInterpolateInput ipEnrichmentMap = {0};
  sMathInterpolateInput ipEnrichmentThr = {0};

  float rpm;
  float map;
  float fuel_ratio;
  float air_temp;
  float engine_temp;
  float throttle;
  float power_voltage;
  float reference_voltage;
  float speed;
  float knock;
  float idle_valve_position;
  float knock_raw;

  float wish_fuel_ratio;
  float filling_map;
  float filling_thr;
  float effective_volume;
  float ignition_angle;
  float ignition_time;
  float injector_lag;
  float cycle_air_flow;
  float mass_air_flow;
  float injection_time;
  float injection_phase;
  float air_destiny;
  float fuel_flow_per_us;
  float knock_filtered;
  float knock_noise_level;
  float idle_valve_pos_correction;

  float min, max;
  static uint32_t prev_halfturns = 0;
  uint32_t halfturns;
  static uint32_t enrichment_step = 0;
  static float enrichment_map_states[ENRICHMENT_STATES_COUNT] = {0};
  static float enrichment_thr_states[ENRICHMENT_STATES_COUNT] = {0};
  static float enrichment_status_map = 0.0f;
  static float enrichment_status_thr = 0.0f;
  float enrichment_map_value;
  float enrichment_thr_value;
  float enrichment_by_map_hpf;
  float enrichment_by_thr_hpf;
  static float enrichment = 0.0f;
  float enrichment_proportion;
  float fuel_amount_per_cycle;

  float fill_proportion;
  float fuel_pressure;
  float fuel_abs_pressure;
  float fuel_consumption;

  float idle_wish_rpm;
  float idle_wish_massair;
  float idle_wish_ignition;
  float idle_rpm_shift;
  float idle_table_valve_pos;
  float idle_wish_valve_pos;
  float idle_angle_correction;
  float injection_dutycycle;

  float filling_diff;
  float filling_diff_map;
  float filling_diff_thr;
  float lpf_calculation;
  float fill_correction_map;
  float fill_correction_thr;
  float idle_valve_pos_adaptation;
  float idle_valve_pos_dif;

  uint8_t rotates;
  uint8_t running;
  uint8_t idle_flag;
  uint8_t cutoff_processing = gEcuCutoffProcessing;
  sCspsData csps = csps_data();

  halfturns = csps_gethalfturns();
  running = csps_isrunning();
  rotates = csps_isrotates();
  fill_proportion = table->fill_proportion_map_vs_thr;
  enrichment_proportion = table->enrichment_proportion_map_vs_thr;
  fuel_pressure = table->fuel_pressure;

  rpm = csps_getrpm(csps);
  speed = speed_getspeed();
  sens_get_map(&map);
  sens_get_knock(&knock);
  sens_get_knock_raw(&knock_raw);
  sens_get_air_temperature(&air_temp);
  sens_get_engine_temperature(&engine_temp);
  sens_get_throttle_position(&throttle);
  sens_get_reference_voltage(&reference_voltage);
  sens_get_power_voltage(&power_voltage);
  idle_valve_position = out_get_idle_valve();

  if(gEcuParams.useLambdaSensor)
    sens_get_o2_fuelratio(&fuel_ratio, NULL);

  idle_flag = throttle < 0.5f && running;

  fuel_abs_pressure = fuel_pressure + (1.0f - map * 0.00001f);
  fuel_flow_per_us = table->injector_performance * 1.66666667e-8f * table->fuel_mass_per_cc; // perf / 60.000.000
  fuel_flow_per_us *= fuel_abs_pressure / fuel_pressure;

  speed *= gEcuParams.speedCorrection;

  ipRpm = math_interpolate_input(rpm, table->rotates, table->rotates_count);
  ipMap = math_interpolate_input(map, table->pressures, table->pressures_count);
  ipTemp = math_interpolate_input(engine_temp, table->engine_temps, table->engine_temp_count);
  ipSpeed = math_interpolate_input(speed, table->idle_rpm_shift_speeds, table->idle_speeds_shift_count);
  ipThr = math_interpolate_input(throttle, table->throttles, table->throttles_count);
  ipVoltages = math_interpolate_input(power_voltage, table->voltages, table->voltages_count);

  filling_map = math_interpolate_2d(ipRpm, ipMap, TABLE_ROTATES_MAX, table->fill_by_map);
  filling_thr = math_interpolate_2d(ipRpm, ipThr, TABLE_ROTATES_MAX, table->fill_by_thr);

  fill_correction_map = math_interpolate_2d_int16(ipRpm, ipMap, TABLE_ROTATES_MAX, gEcuCorrections.fill_by_map) * 0.00006103515625f;
  fill_correction_thr = math_interpolate_2d_int16(ipRpm, ipThr, TABLE_ROTATES_MAX, gEcuCorrections.fill_by_thr) * 0.00006103515625f;

  filling_map *= fill_correction_map + 1.0f;
  filling_thr *= fill_correction_thr + 1.0f;

  effective_volume = filling_map * (fill_proportion) + filling_thr * (1.0f - fill_proportion);
  effective_volume *= gEcuParams.engineVolume;

  air_destiny = ecu_get_air_destiny(map, air_temp);
  cycle_air_flow = effective_volume * 0.25f * air_destiny * 1000.0f;
  mass_air_flow = rpm * 0.03333333f * cycle_air_flow * 0.001f * 3.6f; // rpm / 60 * 2

  while(halfturns != prev_halfturns) {
    prev_halfturns++;
    enrichment_map_states[enrichment_step] = map;
    enrichment_thr_states[enrichment_step] = throttle;
    if(++enrichment_step >= ENRICHMENT_STATES_COUNT)
      enrichment_step = 0;

    if(running) {
      math_minmax(enrichment_map_states, ENRICHMENT_STATES_COUNT, &min, &max);
      if(min > max) enrichment_map_value = 0.0f; else enrichment_map_value = max - min;
      math_minmax(enrichment_thr_states, ENRICHMENT_STATES_COUNT, &min, &max);
      if(min > max) enrichment_thr_value = 0.0f; else enrichment_thr_value = max - min;

      ipEnrichmentMap = math_interpolate_input(enrichment_map_value, table->pressures, table->pressures_count);
      enrichment_map_value = math_interpolate_1d(ipEnrichmentMap, table->enrichment_by_map_sens);

      ipEnrichmentThr = math_interpolate_input(enrichment_thr_value, table->throttles, table->throttles_count);
      enrichment_thr_value = math_interpolate_1d(ipEnrichmentThr, table->enrichment_by_map_sens);

      enrichment_by_map_hpf = math_interpolate_1d(ipRpm, table->enrichment_by_map_sens);
      enrichment_by_thr_hpf = math_interpolate_1d(ipRpm, table->enrichment_by_thr_sens);

      enrichment_status_map *= 1.0f - enrichment_by_map_hpf;
      enrichment_status_thr *= 1.0f - enrichment_by_thr_hpf;

      if(enrichment_map_value > enrichment_status_map)
        enrichment_status_map = enrichment_map_value;
      if(enrichment_thr_value > enrichment_status_thr)
        enrichment_status_thr = enrichment_thr_value;
    } else {
      enrichment_status_map = 0.0f;
      enrichment_status_thr = 0.0f;
    }
    enrichment = enrichment_status_map * (enrichment_proportion) + enrichment_status_thr * (1.0f - enrichment_proportion);
  }

  ipFilling = math_interpolate_input(cycle_air_flow, table->fillings, table->fillings_count);

  ignition_angle = math_interpolate_2d(ipRpm, ipFilling, TABLE_ROTATES_MAX, table->ignitions);
  ignition_angle += math_interpolate_2d_int16(ipRpm, ipFilling, TABLE_ROTATES_MAX, gEcuCorrections.ignitions) * 0.001220703125f;

  wish_fuel_ratio = math_interpolate_2d(ipRpm, ipFilling, TABLE_ROTATES_MAX, table->fuel_mixtures);
  injection_phase = math_interpolate_2d(ipRpm, ipFilling, TABLE_ROTATES_MAX, table->injection_phase);


  if(idle_flag && running) {
    if(out_get_fan(NULL) != GPIO_PIN_RESET) {
      ignition_angle += table->idle_ign_fan_corr;
    }
  }

  if(cutoff_processing) {
    ignition_angle = gEcuParams.cutoffAngle;
    wish_fuel_ratio = gEcuParams.cutoffMixture;
  }

  if(!gEcuParams.useLambdaSensor)
    fuel_ratio = wish_fuel_ratio;

  injector_lag = math_interpolate_1d(ipVoltages, table->injector_lag);
  ignition_time = math_interpolate_1d(ipVoltages, table->ignition_time);
  ignition_time *= math_interpolate_1d(ipRpm, table->ignition_time_rpm_mult);

  fuel_amount_per_cycle = cycle_air_flow * 0.001f / wish_fuel_ratio;
  injection_time = fuel_amount_per_cycle / fuel_flow_per_us;
  injection_time *= enrichment + 1.0f;
  injection_time += injector_lag;

  idle_wish_rpm = math_interpolate_1d(ipTemp, table->idle_wish_rotates);
  idle_wish_massair = math_interpolate_1d(ipTemp, table->idle_wish_massair);
  idle_wish_ignition = math_interpolate_1d(ipRpm, table->idle_wish_ignition);

  idle_rpm_shift = math_interpolate_1d(ipSpeed, table->idle_rpm_shift);
  knock_noise_level = math_interpolate_1d(ipRpm, table->knock_noise_level);

  idle_wish_rpm += idle_rpm_shift;

  idle_valve_pos_adaptation = math_interpolate_1d_int16(ipRpm, gEcuCorrections.idle_valve_to_rpm) * 0.015625f;

  idle_table_valve_pos = math_interpolate_1d(ipRpm, table->idle_valve_to_rpm);
  idle_wish_valve_pos = idle_table_valve_pos;
  idle_wish_valve_pos *= idle_valve_pos_adaptation + 1.0f;

  ecu_pid_update(idle_flag);

  math_pid_set_target(&gPidIdleAirFlow, idle_wish_massair);
  math_pid_set_target(&gPidIdleIgnition, idle_wish_rpm);

  idle_valve_pos_correction = math_pid_update(&gPidIdleAirFlow, mass_air_flow, now);
  idle_angle_correction = math_pid_update(&gPidIdleIgnition, rpm, now);

  if(idle_flag && !cutoff_processing) {
    ignition_angle += idle_angle_correction;
  }

  idle_wish_valve_pos += idle_valve_pos_correction;

  injection_dutycycle = injection_time / csps_getperiod(csps) * 2.0f;

  out_set_idle_valve(idle_wish_valve_pos);

  knock_filtered = knock - knock_noise_level;
  if(knock_filtered < 0.0f)
    knock_filtered = 0.0f;

  if(!running) {
    ignition_angle = table->ignition_initial;
    if(throttle >= 90.0f)
      injection_time = 0;
  }

  if(!rotates) {
    injection_time = 0;
    cycle_air_flow = 0;
    mass_air_flow = 0;
    injection_dutycycle = 0;
    ignition_time = 0;
    effective_volume = 0;
    fuel_consumption = 0;
  } else {
    km_driven += speed * 2.77777778e-10f * diff; // speed / (60 * 60 * 1.000.000) * diff
    fuel_consumed += fuel_amount_per_cycle / table->fuel_mass_per_cc * (diff / (60000000.0f / rpm)) * 0.001f;
    fuel_consumption = fuel_consumed / km_driven * 100.0f;
  }

  if(gEcuParams.useKnockSensor) {

  }

  if(gEcuParams.performAdaptation) {
    if(running) {
      if(!gEcuCriticalBackupWriteAccess) {
        if(adapt_diff >= 100000) {
          adaptation_last = now;
          lpf_calculation = adapt_diff * 0.000001f * 0.1f;

          if(gEcuParams.useLambdaSensor) {
            filling_diff = (fuel_ratio / wish_fuel_ratio) - 1.0f;

            filling_diff_map = filling_diff * table->fill_proportion_map_vs_thr;
            fill_correction_map *= filling_diff_map * lpf_calculation + 1.0f;

            filling_diff_thr = filling_diff * (1.0f - table->fill_proportion_map_vs_thr);
            fill_correction_thr *= filling_diff_thr * lpf_calculation + 1.0f;

            math_interpolate_2d_set_int16(ipRpm, ipMap, TABLE_ROTATES_MAX, gEcuCorrections.fill_by_map, fill_correction_map);
            math_interpolate_2d_set_int16(ipRpm, ipMap, TABLE_ROTATES_MAX, gEcuCorrections.fill_by_thr, fill_correction_thr);
          }

          idle_valve_pos_dif = idle_wish_valve_pos / idle_table_valve_pos - 1.0f;
          idle_valve_pos_adaptation *= idle_valve_pos_dif * lpf_calculation + 1.0f;

          math_interpolate_1d_set_int16(ipRpm, gEcuCorrections.idle_valve_to_rpm, idle_valve_pos_adaptation);

        }
      }
    }
  }

  if(!gEcuCriticalBackupWriteAccess) {
    gEcuCriticalBackup.km_driven += km_driven;
    gEcuCriticalBackup.fuel_consumed += fuel_consumed;
    km_driven = 0;
    fuel_consumed = 0;
  }

  //TODO: add usage of gForceParameters

  gParameters.AdcKnockVoltage = knock_raw;
  gParameters.AdcAirTemp = ADC_GetVoltage(AdcChAirTemperature);
  gParameters.AdcEngineTemp = ADC_GetVoltage(AdcChEngineTemperature);
  gParameters.AdcManifoldAirPressure = ADC_GetVoltage(AdcChManifoldAbsolutePressure);
  gParameters.AdcThrottlePosition = ADC_GetVoltage(AdcChThrottlePosition);
  gParameters.AdcPowerVoltage = ADC_GetVoltage(AdcChPowerVoltage);
  gParameters.AdcReferenceVoltage = ADC_GetVoltage(AdcMcuChReferenceVoltage);
  gParameters.AdcLambdaUR = ADC_GetVoltage(AdcChO2UR);
  gParameters.AdcLambdaUA = ADC_GetVoltage(AdcChO2UA);

  gParameters.KnockSensor = knock;
  gParameters.KnockSensorFiltered = knock_filtered;
  gParameters.AirTemp = air_temp;
  gParameters.EngineTemp = engine_temp;
  gParameters.ManifoldAirPressure = map;
  gParameters.ThrottlePosition = throttle;
  gParameters.ReferenceVoltage = reference_voltage;
  gParameters.PowerVoltage = power_voltage;
  gParameters.FuelRatio = fuel_ratio;

  gParameters.IdleFlag = idle_flag;
  gParameters.RPM = rpm;
  gParameters.Speed = speed;
  gParameters.MassAirFlow = mass_air_flow;
  gParameters.CyclicAirFlow = cycle_air_flow;
  gParameters.EffectiveVolume = effective_volume;
  gParameters.AirDestiny = air_destiny;
  gParameters.WishFuelRatio = wish_fuel_ratio;
  gParameters.IdleValvePosition = idle_valve_position;
  gParameters.WishIdleRPM = idle_wish_rpm;
  gParameters.WishIdleMassAirFlow = idle_wish_massair;
  gParameters.WishIdleValvePosition = idle_wish_valve_pos;
  gParameters.WishIdleIgnitionAngle = idle_wish_ignition;
  gParameters.IgnitionAngle = ignition_angle;
  gParameters.InjectionPhase = injection_phase;
  gParameters.InjectionPulse = injection_time;
  gParameters.InjectionDutyCycle = injection_dutycycle;
  gParameters.InjectionEnrichment = enrichment;
  gParameters.IgnitionPulse = ignition_time;
  gParameters.IdleSpeedShift = idle_rpm_shift;

  gParameters.DrivenKilometers = gEcuCriticalBackup.km_driven;
  gParameters.FuelConsumed = gEcuCriticalBackup.fuel_consumed;
  gParameters.FuelConsumption = fuel_consumption;

  gParameters.OilSensor = sens_get_oil_pressure(NULL) != GPIO_PIN_RESET;
  gParameters.StarterSensor = sens_get_starter(NULL) != GPIO_PIN_RESET;
  gParameters.HandbrakeSensor = sens_get_handbrake(NULL) != GPIO_PIN_RESET;
  gParameters.ChargeSensor = sens_get_charge(NULL);
  gParameters.Rsvd1Sensor = sens_get_rsvd1(NULL) != GPIO_PIN_RESET;
  gParameters.Rsvd2Sensor = sens_get_rsvd2(NULL) != GPIO_PIN_RESET;

  gParameters.FuelPumpRelay = out_get_fuelpump(NULL) != GPIO_PIN_RESET;
  gParameters.FanRelay = out_get_fan(NULL) != GPIO_PIN_RESET;
  gParameters.CheckEngine = out_get_checkengine(NULL) != GPIO_PIN_RESET;
  gParameters.StarterRelay = out_get_starter(NULL) != GPIO_PIN_RESET;
  gParameters.Rsvd1Output = out_get_rsvd1(NULL) != GPIO_PIN_RESET;
  gParameters.Rsvd2Output = out_get_rsvd2(NULL) != GPIO_PIN_RESET;

  gParameters.IsRunning = running;

  updated_last = now;
}

static void ecu_init_post_init(void)
{
  Misc_EnableIdleValvePosition(gEcuCriticalBackup.idle_valve_position);
  if(gStatus.Bkpsram.Struct.Load != HAL_OK) {
    if(gStatus.Bkpsram.Struct.Save != HAL_OK) {

    } else {
      gStatus.Bkpsram.Struct.Load = HAL_OK;
    }
    gEcuIdleValveCalibrateOk = 0;
    gEcuIdleValveCalibrate = 1;
  }
}

static void ecu_idle_valve_process(void)
{
  int8_t status;
  if(gEcuIdleValveCalibrate) {
    status = Misc_CalibrateIdleValve();
    if(status) {
      gEcuIdleValveCalibrate = 0;
      gEcuIdleValveCalibrateOk = status > 0;
    }
  }
}

static void ecu_backup_save_process(void)
{
  int8_t status;

  gEcuCriticalBackupWriteAccess = 1;
  status = config_save_critical_backup(&gEcuCriticalBackup);

  if(status) {
    gEcuCriticalBackupWriteAccess = 0;
    if(status < 0 && gStatus.Bkpsram.Struct.Save == HAL_OK)
      gStatus.Bkpsram.Struct.Save = HAL_ERROR;
  }

}

static uint8_t ecu_cutoff_ign_act(uint8_t cy_count, uint8_t cylinder)
{
  //TODO
  return 1;
}

static uint8_t ecu_cutoff_inj_act(uint8_t cy_count, uint8_t cylinder)
{
  //TODO
  return 1;
}

static void ecu_coil_saturate(uint8_t cy_count, uint8_t cylinder)
{
  if(cy_count == 4 && cylinder < 4) {
    gIgnPorts[cylinder]->BSRR = gIgnPins[cylinder];
  } else if(cy_count == 2) {
    if(cylinder == 0) {
      gIgnPorts[0]->BSRR = gIgnPins[0];
      gIgnPorts[3]->BSRR = gIgnPins[3];
    } else if(cylinder == 1) {
      gIgnPorts[1]->BSRR = gIgnPins[1];
      gIgnPorts[2]->BSRR = gIgnPins[2];
    }
  }
}

static void ecu_coil_ignite(uint8_t cy_count, uint8_t cylinder)
{
  if(cy_count == 4 && cylinder < 4) {
    gIgnPorts[cylinder]->BSRR = gIgnPins[cylinder] << 16;
  } else if(cy_count == 2) {
    if(cylinder == 0) {
      gIgnPorts[0]->BSRR = gIgnPins[0] << 16;
      gIgnPorts[3]->BSRR = gIgnPins[3] << 16;
    } else if(cylinder == 1) {
      gIgnPorts[1]->BSRR = gIgnPins[1] << 16;
      gIgnPorts[2]->BSRR = gIgnPins[2] << 16;
    }
  }
}

static void ecu_inject(uint8_t cy_count, uint8_t cylinder, uint32_t time)
{
  if(cy_count == 4 && cylinder < 4) {
    injector_enable(cylinder, time);
  } else if(cy_count == 2) {
    if(cylinder == 0) {
      injector_enable(InjectorCy1, time);
      injector_enable(InjectorCy4, time);
    } else if(cylinder == 1) {
      injector_enable(InjectorCy2, time);
      injector_enable(InjectorCy3, time);
    }
  }
}

static void ecu_process(void)
{
  sCspsData csps = csps_data();
  static float oldanglesbeforeignite[4] = {0,0,0,0};
  static float oldanglesbeforeinject[4] = {0,0,0,0};
  static uint8_t saturated[4] = { 1,1,1,1 };
  static uint8_t ignited[4] = { 1,1,1,1 };
  static uint8_t injected[4] = { 1,1,1,1 };
  static uint8_t injection[4] = { 1,1,1,1 };
  float angle[4];
  float anglesbeforeignite[4];
  float anglesbeforeinject[4];

  float found = csps_isfound();
  float uspa = csps_getuspa(csps);
  float period = csps_getperiod(csps);
  float time_sat;
  float time_pulse;
  float angle_ignite;
  float saturate;
  float angles_per_turn;
  float inj_phase;
  float inj_pulse;
  float inj_angle;
  uint8_t phased;
  uint8_t cy_count;
  uint8_t individual_coils;
  uint8_t use_tsps;
  uint8_t injector_channel;

  injector_channel = gEcuTable[gParameters.CurrentTable].inj_channel;
  individual_coils = gEcuParams.isIndividualCoils;
  use_tsps = gEcuParams.useTSPS;
  phased = use_tsps && individual_coils && csps_isphased(csps);
  cy_count = phased ? 4 : 2;
  angle_ignite = gParameters.IgnitionAngle;
  time_sat = gParameters.IgnitionPulse;
  time_pulse = 2000;

  inj_phase = gParameters.InjectionPhase;
  inj_pulse = gParameters.InjectionPulse;

  if(phased) {
    angles_per_turn = 720.0f;
  } else {
    angles_per_turn = 360.0f;
    saturated[2] = ignited[2] = injection[2] = injected[2] = 1;
    saturated[3] = ignited[3] = injection[3] = injected[3] = 1;
    inj_pulse *= 0.5;

  }

  while(inj_phase > angles_per_turn * 0.5f) {
    inj_phase -= angles_per_turn;
  }

  if(phased) {
    for(int i = 0; i < cy_count; i++) {
      angle[i] = csps_getphasedangle_cy(csps, i);
    }
  } else {
    angle[0] = csps_getangle14(csps);
    angle[1] = csps_getangle23from14(angle[0]);
  }

  if(found)
  {
    //TODO: do smooth switching between tables
    IGN_NALLOW_GPIO_Port->BSRR = IGN_NALLOW_Pin << 16;
    gInjChPorts[injector_channel]->BSRR = gInjChPins[injector_channel];
    gInjChPorts[injector_channel ^ 1]->BSRR = gInjChPins[injector_channel ^ 1] << 16;

    if(period < time_sat + time_pulse) {
      time_sat = period * ((float)time_sat / (float)(time_sat + time_pulse));
    }

    saturate = time_sat / uspa;
    inj_angle = inj_pulse / uspa;

    //Ignition part
    for(int i = 0; i < cy_count; i++)
    {
      if(angle[i] < -angle_ignite)
        anglesbeforeignite[i] = -angle[i] - angle_ignite;
      else
        anglesbeforeignite[i] = angles_per_turn - angle[i] - angle_ignite;

      if(anglesbeforeignite[i] - oldanglesbeforeignite[i] > 0.0f && anglesbeforeignite[i] - oldanglesbeforeignite[i] < 180.0f)
        anglesbeforeignite[i] = oldanglesbeforeignite[i];

      if(anglesbeforeignite[i] - saturate < 0.0f)
      {
        if(!saturated[i] && !ignited[i])
        {
          saturated[i] = 1;

          if(ecu_cutoff_ign_act(cy_count, i))
            ecu_coil_saturate(cy_count, i);
        }
      }

      if(oldanglesbeforeignite[i] - anglesbeforeignite[i] < -1.0f)
      {
        if(!ignited[i] && saturated[i])
        {
          ignited[i] = 1;
          saturated[i] = 0;

          ecu_coil_ignite(cy_count, i);
        }
      }
      else ignited[i] = 0;

      oldanglesbeforeignite[i] = anglesbeforeignite[i];
    }

    //Injection part
    for(int i = 0; i < cy_count; i++)
    {
      if(angle[i] < inj_phase)
        anglesbeforeinject[i] = -angle[i] + inj_phase;
      else
        anglesbeforeinject[i] = angles_per_turn - angle[i] + inj_phase;

      if(anglesbeforeinject[i] - oldanglesbeforeinject[i] > 0.0f && anglesbeforeinject[i] - oldanglesbeforeinject[i] < 180.0f)
        anglesbeforeinject[i] = oldanglesbeforeinject[i];

      if(anglesbeforeinject[i] - inj_angle < 0.0f)
      {
        if(!injection[i] && !injected[i])
        {
          injection[i] = 1;

          if(ecu_cutoff_inj_act(cy_count, i) && inj_pulse > 0.0f)
            ecu_inject(cy_count, i, inj_pulse);
        }
      }

      if(oldanglesbeforeinject[i] - anglesbeforeinject[i] < -1.0f)
      {
        if(!injected[i] && injection[i])
        {
          injected[i] = 1;
          injection[i] = 0;

          //We may end injection here. TODO: check it somehow?
        }
      }
      else injected[i] = 0;

      oldanglesbeforeinject[i] = anglesbeforeinject[i];
    }
  } else {
    IGN_NALLOW_GPIO_Port->BSRR = IGN_NALLOW_Pin;
    gInjChPorts[0]->BSRR = gInjChPins[0] << 16;
    gInjChPorts[1]->BSRR = gInjChPins[1] << 16;
  }
}

static void ecu_fuelpump_process(void)
{
  static uint32_t active_last = 0;
  static uint8_t active = 0;
  uint32_t now = Delay_Tick;
  uint8_t rotates = csps_isrotates();

  if(rotates || active_last == 0 || DelayDiff(now, active_last) < 1000000) {
    active_last = now;
    active = 1;
  } else {
    active = 0;
  }

  if(active) {
    out_set_fuelpump(GPIO_PIN_SET);
  } else {
    out_set_fuelpump(GPIO_PIN_RESET);
  }

}

static void ecu_fan_process(void)
{
  float engine_temp;
  HAL_StatusTypeDef status;
  GPIO_PinState fan_state;
  float temp_low, temp_high;

  temp_low = gEcuParams.fanLowTemperature;
  temp_high = gEcuParams.fanHighTemperature;
  fan_state = out_get_fan(NULL);
  status = sens_get_engine_temperature(&engine_temp);

  if(status != HAL_OK) {
    out_set_fan(GPIO_PIN_SET);
  } else if(fan_state != GPIO_PIN_RESET) {
    if(engine_temp < temp_low) {
      out_set_fan(GPIO_PIN_RESET);
    }
  } else if(engine_temp >= temp_high) {
    out_set_fan(GPIO_PIN_SET);
  }
}

void ecu_init(void)
{
  ecu_config_init();

  ecu_set_start_allowed(HAL_OK);
  ecu_set_table(0);

  ecu_pid_init();

  ecu_init_post_init();

  gEcuInitialized = 1;
}

void ecu_irq_fast_loop(void)
{
  if(!gEcuInitialized)
    return;

  ecu_process();

}

void ecu_irq_slow_loop(void)
{
  if(!gEcuInitialized)
    return;

  ecu_update_current_table();
  ecu_update();
  ecu_idle_valve_process();
  ecu_backup_save_process();
  ecu_fuelpump_process();
  ecu_fan_process();

}

void ecu_loop(void)
{

}

void ecu_parse_command(eTransChannels xChaSrc, uint8_t * msgBuf, uint32_t length)
{

}
