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
#include "bluetooth.h"
#include "outputs.h"
#include "injector.h"
#include "sst25vf032b.h"
#include "interpolation.h"
#include "math_misc.h"
#include "config.h"
#include "pid.h"
#include "crc.h"
#include "failures.h"

#include <string.h>
#include "arm_math.h"

#define ENRICHMENT_STATES_COUNT 6

typedef volatile struct {
    uint8_t savereq;
    uint8_t loadreq;
    uint8_t issaving;
    uint8_t isloading;

    eTransChannels savereqsrc;
    eTransChannels loadreqsrc;
}sMem;

typedef volatile struct {
    uint8_t Processing;
    uint8_t Cutting;
}sCutoff;

static GPIO_TypeDef * const gIgnPorts[ECU_CYLINDERS_COUNT] = { IGN_1_GPIO_Port, IGN_2_GPIO_Port, IGN_3_GPIO_Port, IGN_4_GPIO_Port };
static const uint16_t gIgnPins[ECU_CYLINDERS_COUNT] = { IGN_1_Pin, IGN_2_Pin, IGN_3_Pin, IGN_4_Pin };

static GPIO_TypeDef * const gInjChPorts[2] = { INJ_CH1_GPIO_Port, INJ_CH2_GPIO_Port };
static const uint16_t gInjChPins[2] = { INJ_CH1_Pin, INJ_CH2_Pin};

static sEcuTable gEcuTable[TABLE_SETUPS_MAX];
static sEcuParams gEcuParams;
static sEcuCorrections gEcuCorrections;
static sEcuCriticalBackup gEcuCriticalBackup;
static uint8_t volatile gStatusReset = 0;
static sStatus gStatus = {{{0}}};
static sParameters gParameters = {0};
static sForceParameters gForceParameters = {0};
static uint8_t gCheckBitmap[CHECK_BITMAP_SIZE] = {0};

static volatile uint8_t gEcuIdleValveCalibrate = 0;
static volatile uint8_t gEcuIdleValveCalibrateOk = 0;
static volatile uint8_t gEcuInitialized = 0;
static volatile uint8_t gEcuIsError = 0;

static sMathPid gPidIdleIgnition = {0};
static sMathPid gPidIdleAirFlow = {0};

static sDrag Drag = {0};
static sMem Mem = {0};
static sCutoff Cutoff = {0};

static uint8_t ecu_get_table(void)
{
  return gParameters.CurrentTable;
}

static void ecu_set_table(uint8_t number)
{
  gParameters.SwitchPosition = number;
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

  if(gStatus.Flash.Struct.Init == HAL_OK) {
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
  } else {
    config_default_params(&gEcuParams);
    gStatus.Flash.Struct.Load = HAL_ERROR;
    gStatus.Flash.Struct.Save = HAL_ERROR;
    for(int i = 0; i < TABLE_SETUPS_MAX; i++)
      config_default_table(&gEcuTable[i], i);
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
  //Output in mg/cc
  const float M = 29.0f;
  const float R = 8314.46261815324f;
  float mg_cc3 = (pressure * M) / (R * (temperature + 273.0f));
  return mg_cc3;
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

STATIC_INLINE void ecu_pid_update(uint8_t isidle)
{
  uint32_t table_number = gParameters.CurrentTable;
  sEcuTable *table = &gEcuTable[table_number];

  math_pid_set_clamp(&gPidIdleIgnition, table->idle_ign_deviation_min, table->idle_ign_deviation_max);
  math_pid_set_clamp(&gPidIdleAirFlow, -256, 256);

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


float gDebugMap = 20000;
float gDebugAirTemp = 20.0f;
float gDebugEngineTemp = 90.0f;
float gDebugThrottle = 5;
float gDebugReferenceVoltage = 5.1f;
float gDebugPowerVoltage = 14.4f;

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
  float acceleration;
  float knock;
  float idle_valve_position;
  float knock_raw;
  float uspa;

  float wish_fuel_ratio;
  float filling_map;
  float map_thr;
  float effective_volume;
  float ignition_angle;
  float ignition_time;
  float injector_lag;
  float cycle_air_flow;
  float mass_air_flow;
  float injection_time;
  float injection_phase;
  float injection_phase_duration;
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
  float knock_threshold;

  float warmup_mixture;
  float warmup_mix_koff;

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
  float map_diff_thr;
  float lpf_calculation;
  float fill_correction_map;
  float map_correction_thr;
  float idle_valve_pos_adaptation;
  float idle_valve_pos_dif;
  float ignition_correction;
  float wish_fault_rpm;
  float start_mixture;
  float long_term_correction;

  HAL_StatusTypeDef knock_status;
  uint8_t rotates;
  uint8_t running;
  uint8_t phased;
  uint8_t idle_flag;
  uint8_t o2_valid = 0;
  uint8_t cutoff_processing = Cutoff.Processing;
  sCspsData csps = csps_data();

  halfturns = csps_gethalfturns();
  running = csps_isrunning();
  rotates = csps_isrotates();
  phased = csps_isphased(csps);
  uspa = csps_getuspa(csps);
  enrichment_proportion = table->enrichment_proportion_map_vs_thr;
  fuel_pressure = table->fuel_pressure;
  long_term_correction = gEcuCorrections.long_term_correction;

  gStatus.Sensors.Struct.Csps = csps_iserror() == 0 ? HAL_OK : HAL_ERROR;
  rpm = csps_getrpm(csps);
  speed = speed_getspeed();
  acceleration = speed_getacceleration();
  //gStatus.Sensors.Struct.Map = sens_get_map(&map);
  map = gDebugMap;
  knock_status = sens_get_knock(&knock);
  sens_get_knock_raw(&knock_raw);
  //gStatus.Sensors.Struct.AirTemp = sens_get_air_temperature(&air_temp);
  air_temp = gDebugAirTemp;
  //gStatus.Sensors.Struct.EngineTemp = sens_get_engine_temperature(&engine_temp);
  engine_temp = gDebugEngineTemp;
  //gStatus.Sensors.Struct.ThrottlePos = sens_get_throttle_position(&throttle);
  throttle = gDebugThrottle;
  //gStatus.Sensors.Struct.ReferenceVoltage = sens_get_reference_voltage(&reference_voltage);
  reference_voltage = gDebugReferenceVoltage;
  //gStatus.Sensors.Struct.PowerVoltage = sens_get_power_voltage(&power_voltage);
  power_voltage = gDebugPowerVoltage;

  if(gStatus.Sensors.Struct.EngineTemp != HAL_OK)
    engine_temp = 40.0f;
  if(gStatus.Sensors.Struct.AirTemp != HAL_OK)
    air_temp = 10.0f;

  if(gEcuParams.useLambdaSensor) {
    gStatus.Sensors.Struct.Lambda = sens_get_o2_fuelratio(&fuel_ratio, &o2_valid);
  } else {
    gStatus.Sensors.Struct.Lambda = HAL_OK;
  }

  if(running && gEcuParams.useTSPS && !phased) {
    gStatus.Sensors.Struct.Tsps = HAL_ERROR;
  } else {
    gStatus.Sensors.Struct.Tsps = HAL_OK;
  }

  if(gEcuParams.useKnockSensor) {
    gStatus.Sensors.Struct.Knock = knock_status;
  } else {
    gStatus.Sensors.Struct.Knock = HAL_OK;
  }

  idle_flag = throttle < 2.0f && running;

  if(gStatus.Sensors.Struct.Map == HAL_OK && gStatus.Sensors.Struct.ThrottlePos != HAL_OK) {
    wish_fault_rpm = 1400.0f;
  } else if(gStatus.Sensors.Struct.Map != HAL_OK && gStatus.Sensors.Struct.ThrottlePos == HAL_OK) {
    wish_fault_rpm = 1700.0f;
  } else {
    wish_fault_rpm = 0.0f;
  }

  speed *= gEcuParams.speedCorrection;

  ipRpm = math_interpolate_input(rpm, table->rotates, table->rotates_count);
  ipThr = math_interpolate_input(throttle, table->throttles, table->throttles_count);

  map_thr = math_interpolate_2d(ipRpm, ipThr, TABLE_ROTATES_MAX, table->map_by_thr);
  map_correction_thr = math_interpolate_2d(ipRpm, ipThr, TABLE_ROTATES_MAX, gEcuCorrections.map_by_thr);

  map_thr *= map_correction_thr + 1.0f;

  if(gStatus.Sensors.Struct.ThrottlePos == HAL_OK && gStatus.Sensors.Struct.Map != HAL_OK) {
    map = map_thr;
  }

  fuel_abs_pressure = fuel_pressure + (1.0f - map * 0.00001f);
  fuel_flow_per_us = table->injector_performance * 1.66666667e-8f * table->fuel_mass_per_cc; // perf / 60.000.000
  fuel_flow_per_us *= fuel_abs_pressure / fuel_pressure;

  ipTemp = math_interpolate_input(engine_temp, table->engine_temps, table->engine_temp_count);
  ipSpeed = math_interpolate_input(speed, table->idle_rpm_shift_speeds, table->idle_speeds_shift_count);
  ipMap = math_interpolate_input(map, table->pressures, table->pressures_count);
  ipVoltages = math_interpolate_input(power_voltage, table->voltages, table->voltages_count);

  filling_map = math_interpolate_2d(ipRpm, ipMap, TABLE_ROTATES_MAX, table->fill_by_map);

  fill_correction_map = math_interpolate_2d(ipRpm, ipMap, TABLE_ROTATES_MAX, gEcuCorrections.fill_by_map);

  filling_map *= fill_correction_map + 1.0f;

  effective_volume = filling_map * gEcuParams.engineVolume;

  air_destiny = ecu_get_air_destiny(map, air_temp);

  cycle_air_flow = effective_volume * 0.25f * air_destiny;
  mass_air_flow = rpm * 0.03333333f * cycle_air_flow * 0.001f * 3.6f; // rpm / 60 * 2

  while(halfturns != prev_halfturns) {
    prev_halfturns++;
    enrichment_map_states[enrichment_step] = map;
    enrichment_thr_states[enrichment_step] = throttle;
    if(++enrichment_step >= ENRICHMENT_STATES_COUNT)
      enrichment_step = 0;

    if(running) {

      if(gStatus.Sensors.Struct.Map == HAL_OK) {
        math_minmax(enrichment_map_states, ENRICHMENT_STATES_COUNT, &min, &max);
        if(min > max) enrichment_map_value = 0.0f; else enrichment_map_value = max - min;
        ipEnrichmentMap = math_interpolate_input(enrichment_map_value, table->pressures, table->pressures_count);
        enrichment_map_value = math_interpolate_1d(ipEnrichmentMap, table->enrichment_by_map_sens);
        enrichment_by_map_hpf = math_interpolate_1d(ipRpm, table->enrichment_by_map_sens);
        enrichment_status_map *= 1.0f - enrichment_by_map_hpf;
        if(enrichment_map_value > enrichment_status_map)
          enrichment_status_map = enrichment_map_value;
      } else {
        enrichment_status_map = 0.0f;
      }

      if(gStatus.Sensors.Struct.ThrottlePos == HAL_OK) {
        math_minmax(enrichment_thr_states, ENRICHMENT_STATES_COUNT, &min, &max);
        if(min > max) enrichment_thr_value = 0.0f; else enrichment_thr_value = max - min;
        ipEnrichmentThr = math_interpolate_input(enrichment_thr_value, table->throttles, table->throttles_count);
        enrichment_thr_value = math_interpolate_1d(ipEnrichmentThr, table->enrichment_by_map_sens);
        enrichment_by_thr_hpf = math_interpolate_1d(ipRpm, table->enrichment_by_thr_sens);
        enrichment_status_thr *= 1.0f - enrichment_by_thr_hpf;
        if(enrichment_thr_value > enrichment_status_thr)
          enrichment_status_thr = enrichment_thr_value;
      } else {
        enrichment_status_thr = 0.0f;
      }

    } else {
      enrichment_status_map = 0.0f;
      enrichment_status_thr = 0.0f;
    }

    if(gStatus.Sensors.Struct.ThrottlePos == HAL_OK && gStatus.Sensors.Struct.Map == HAL_OK) {
      enrichment = enrichment_status_map * (enrichment_proportion) + enrichment_status_thr * (1.0f - enrichment_proportion);
    } else if(gStatus.Sensors.Struct.ThrottlePos == HAL_OK) {
      enrichment = enrichment_status_thr;
    } else if(gStatus.Sensors.Struct.Map == HAL_OK) {
      enrichment = enrichment_status_map;
    } else enrichment = 0;
  }

  ipFilling = math_interpolate_input(cycle_air_flow, table->fillings, table->fillings_count);

  ignition_angle = math_interpolate_2d(ipRpm, ipFilling, TABLE_ROTATES_MAX, table->ignitions);
  ignition_correction = math_interpolate_2d(ipRpm, ipFilling, TABLE_ROTATES_MAX, gEcuCorrections.ignitions);


  if(idle_flag && running) {
    if(out_get_fan(NULL) != GPIO_PIN_RESET || gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.FanRelay == OutputDiagShortToGnd) {
      ignition_correction += table->idle_ign_fan_corr;

      if(ignition_correction > table->idle_ign_deviation_max)
        ignition_correction = table->idle_ign_deviation_max;
      else if(ignition_correction < table->idle_ign_deviation_min)
        ignition_correction = table->idle_ign_deviation_min;
    }
  }
  ignition_angle += ignition_correction;

  wish_fuel_ratio = math_interpolate_2d(ipRpm, ipFilling, TABLE_ROTATES_MAX, table->fuel_mixtures);
  injection_phase = math_interpolate_2d(ipRpm, ipFilling, TABLE_ROTATES_MAX, table->injection_phase);

  start_mixture = math_interpolate_1d(ipTemp, table->start_mixtures);

  if(gForceParameters.Enable.InjectionPhase)
    injection_phase = gForceParameters.InjectionPhase;


  if(gForceParameters.Enable.IgnitionAngle)
    ignition_angle = gForceParameters.IgnitionAngle;
  if(gForceParameters.Enable.IgnitionOctane)
    ignition_angle += gForceParameters.IgnitionOctane;

  if(running) {
    warmup_mix_koff = math_interpolate_1d(ipTemp, table->warmup_mix_koffs);
    if(warmup_mix_koff != 0.0f) {
      warmup_mixture = math_interpolate_1d(ipTemp, table->warmup_mixtures);
      wish_fuel_ratio = warmup_mixture * warmup_mix_koff + wish_fuel_ratio * (1.0f - warmup_mix_koff);
    }
  } else {
    wish_fuel_ratio = start_mixture;
  }

  if(cutoff_processing) {
    ignition_angle = gEcuParams.cutoffAngle;
    wish_fuel_ratio = gEcuParams.cutoffMixture;
  }

  if(gForceParameters.Enable.WishFuelRatio)
    wish_fuel_ratio = gForceParameters.WishFuelRatio;

  if(!gEcuParams.useLambdaSensor)
    fuel_ratio = wish_fuel_ratio;

  injector_lag = math_interpolate_1d(ipVoltages, table->injector_lag);
  ignition_time = math_interpolate_1d(ipVoltages, table->ignition_time);
  ignition_time *= math_interpolate_1d(ipRpm, table->ignition_time_rpm_mult);

  if(gForceParameters.Enable.IgnitionTime)
    ignition_time = gForceParameters.IgnitionTime;

  fuel_amount_per_cycle = cycle_air_flow * 0.001f / wish_fuel_ratio;
  injection_time = fuel_amount_per_cycle / fuel_flow_per_us;
  injection_time *= enrichment + 1.0f;
  injection_time *= long_term_correction + 1.0f;
  injection_time += injector_lag;

  if(gForceParameters.Enable.InjectionTime)
    injection_time = gForceParameters.InjectionTime;

  idle_wish_rpm = math_interpolate_1d(ipTemp, table->idle_wish_rotates);
  idle_wish_massair = math_interpolate_1d(ipTemp, table->idle_wish_massair);
  idle_wish_ignition = math_interpolate_1d(ipRpm, table->idle_wish_ignition);

  idle_rpm_shift = math_interpolate_1d(ipSpeed, table->idle_rpm_shift);
  knock_noise_level = math_interpolate_1d(ipRpm, table->knock_noise_level);
  knock_threshold = math_interpolate_1d(ipRpm, table->knock_threshold);

  idle_wish_rpm += idle_rpm_shift;

  if(gForceParameters.Enable.WishIdleRPM)
    idle_wish_rpm = gForceParameters.WishIdleRPM;

  if(idle_wish_rpm < wish_fault_rpm) {
    idle_wish_rpm = wish_fault_rpm;
    idle_wish_massair *= wish_fault_rpm / idle_wish_rpm;
  }

  if(gForceParameters.Enable.WishIdleMassAirFlow)
    idle_wish_massair = gForceParameters.WishIdleMassAirFlow;

  idle_table_valve_pos = math_interpolate_2d(ipRpm, ipTemp, TABLE_ROTATES_MAX, table->idle_valve_to_rpm);
  idle_valve_pos_adaptation = math_interpolate_2d(ipRpm, ipTemp, TABLE_ROTATES_MAX, gEcuCorrections.idle_valve_to_rpm);

  idle_wish_valve_pos = idle_table_valve_pos;
  idle_wish_valve_pos *= idle_valve_pos_adaptation + 1.0f;

  ecu_pid_update(idle_flag);

  math_pid_set_target(&gPidIdleAirFlow, idle_wish_massair);
  math_pid_set_target(&gPidIdleIgnition, idle_wish_rpm);

  idle_valve_pos_correction = math_pid_update(&gPidIdleAirFlow, mass_air_flow, now);
  idle_angle_correction = math_pid_update(&gPidIdleIgnition, rpm, now);

  if(!idle_flag) {
    idle_valve_pos_correction = 0;
    idle_angle_correction = 0;
  }

  if(idle_flag && !cutoff_processing) {
    ignition_angle = idle_wish_ignition;
    ignition_angle += idle_angle_correction;

    if(gForceParameters.Enable.WishIdleIgnitionAngle) {
      math_pid_reset(&gPidIdleIgnition);
      ignition_angle = gForceParameters.WishIdleIgnitionAngle;
    }
  }

  idle_wish_valve_pos += idle_valve_pos_correction;

  if(gForceParameters.Enable.WishIdleValvePosition) {
    math_pid_reset(&gPidIdleAirFlow);
    idle_wish_valve_pos = gForceParameters.WishIdleValvePosition;
  }

  injection_dutycycle = injection_time / (csps_getperiod(csps) * 2.0f);

  out_set_idle_valve(idle_wish_valve_pos);

  knock_filtered = knock - knock_noise_level;
  if(knock_filtered < 0.0f)
    knock_filtered = 0.0f;

  if(gStatus.Sensors.Struct.Map != HAL_OK && gStatus.Sensors.Struct.ThrottlePos != HAL_OK) {
    running = 0;
    rotates = 0;
  }

  if(!rotates) {
    running = 0;
    injection_time = 0;
    cycle_air_flow = 0;
    mass_air_flow = 0;
    injection_dutycycle = 0;
    ignition_time = 0;
    effective_volume = 0;
    fuel_consumption = 0;
  } else {
    km_driven += speed * 2.77777778e-10f * diff; // speed / (60 * 60 * 1.000.000) * diff
    if(fuel_amount_per_cycle < 1000.0f)
      fuel_consumed += fuel_amount_per_cycle / table->fuel_mass_per_cc * (diff / (60000000.0f / rpm)) * 0.001f;
    fuel_consumption = fuel_consumed / km_driven * 100.0f;
  }

  if(!running) {
    ignition_angle = table->ignition_initial;
    if(gStatus.Sensors.Struct.ThrottlePos == HAL_OK && throttle >= 80.0f)
      injection_time = 0;
  }

  if(uspa > 0.0f && injection_time > 0.0f) {
    injection_phase_duration = injection_time / uspa;
  } else {
    injection_phase_duration = 0;
  }

  if(injection_dutycycle > 0.85f) {
    gStatus.InjectionUnderflow = HAL_ERROR;
  } else {
    gStatus.InjectionUnderflow = HAL_OK;
  }

  if(gEcuParams.useKnockSensor && gStatus.Sensors.Struct.Knock == HAL_OK && !gForceParameters.Enable.IgnitionAngle) {
    if(csps_isrunning()) {
      if(knock_filtered >= knock_threshold) {
        gStatus.KnockStatus = KnockStatusDedonation;
      } else if(knock < knock_noise_level * 0.5f) {
        gStatus.KnockStatus = KnockStatusLowNoise;
      } else {
        gStatus.KnockStatus = KnockStatusOk;
      }
    } else {
      gStatus.KnockStatus = KnockStatusOk;
    }
  }

  if(adapt_diff >= 100000) {
    adaptation_last = now;
    if(running) {
      if(gEcuParams.performAdaptation) {
        lpf_calculation = adapt_diff * 0.000001f * 0.1f;

        if(gEcuParams.useLambdaSensor && gStatus.Sensors.Struct.Lambda == HAL_OK && o2_valid && !gForceParameters.Enable.InjectionTime) {
          gEcuCorrections.long_term_correction = 0.0f;

          filling_diff = (fuel_ratio / wish_fuel_ratio) - 1.0f;

          if(gStatus.Sensors.Struct.Map == HAL_OK) {
            fill_correction_map *= filling_diff * lpf_calculation + 1.0f;
            math_interpolate_2d_set(ipRpm, ipMap, TABLE_ROTATES_MAX, gEcuCorrections.fill_by_map, fill_correction_map);
          }

          if(gStatus.Sensors.Struct.ThrottlePos == HAL_OK && gStatus.Sensors.Struct.Map == HAL_OK) {
            map_diff_thr = map / map_thr;
            map_correction_thr *= map_diff_thr * lpf_calculation + 1.0f;
            math_interpolate_2d_set(ipRpm, ipThr, TABLE_ROTATES_MAX, gEcuCorrections.map_by_thr, map_correction_thr);
          }

        }

        if(idle_flag && gStatus.Sensors.Struct.Map == HAL_OK && gStatus.Sensors.Struct.ThrottlePos == HAL_OK && !gForceParameters.Enable.WishIdleValvePosition) {
          idle_valve_pos_dif = idle_wish_valve_pos / idle_table_valve_pos - 1.0f;
          idle_valve_pos_adaptation *= idle_valve_pos_dif * lpf_calculation + 1.0f;

          math_interpolate_2d_set(ipRpm, ipTemp, TABLE_ROTATES_MAX, gEcuCorrections.idle_valve_to_rpm, idle_valve_pos_adaptation);
        }


      } else {
        if(gEcuParams.useLambdaSensor && gStatus.Sensors.Struct.Lambda == HAL_OK && !gForceParameters.Enable.InjectionTime) {
          lpf_calculation = adapt_diff * 0.000001f * 0.016666667f;
          filling_diff = (fuel_ratio / wish_fuel_ratio) - 1.0f;
          gEcuCorrections.long_term_correction *= filling_diff * lpf_calculation + 1.0f;
        }
      }
    }
  }

  idle_valve_position = out_get_idle_valve();

  gEcuCriticalBackup.idle_valve_position = idle_valve_position;
  gEcuCriticalBackup.km_driven += km_driven;
  gEcuCriticalBackup.fuel_consumed += fuel_consumed;
  km_driven = 0;
  fuel_consumed = 0;

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
  gParameters.LongTermCorrection = long_term_correction;

  gParameters.IdleFlag = idle_flag;
  gParameters.RPM = rpm;
  gParameters.Acceleration = acceleration;
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
  gParameters.InjectionPhaseDuration = injection_phase_duration;
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
  gParameters.IsCheckEngine = gEcuIsError;
  gParameters.InjectorChannel = table->inj_channel;

  strcpy(gParameters.CurrentTableName, table->name);

  updated_last = now;
}

static void ecu_init_post_init(void)
{
  Misc_EnableIdleValvePosition(gEcuCriticalBackup.idle_valve_position);
  if(gStatus.Bkpsram.Struct.Load != HAL_OK) {
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
  static uint8_t state = 0;
  static uint32_t save_corecction_last = 0;
  uint32_t now = Delay_Tick;
  int8_t critical_status = 0;
  int8_t backup_status = 0;

  if(!CRC16_IsBusy()) {
    critical_status = config_save_critical_backup(&gEcuCriticalBackup);
  }

  switch(state) {
    case 0:
      break;
    case 1:
      if(DelayDiff(now, save_corecction_last) > 1000000) {
        save_corecction_last = now;
        state++;
      } else {
        state = 0;
      }
      break;
    case 2:
      if(!CRC16_IsBusy()) {
        backup_status = config_save_corrections(&gEcuCorrections);
        if(backup_status) {
          state = 0;
        }
      }
      break;

    default:
      state = 0;
      break;
  }

  if(critical_status || backup_status) {
    if((critical_status < 0 || backup_status < 0) && gStatus.Bkpsram.Struct.Save == HAL_OK)
      gStatus.Bkpsram.Struct.Save = HAL_ERROR;
  }
}

STATIC_INLINE uint8_t ecu_cutoff_ign_act(uint8_t cy_count, uint8_t cylinder, float rpm)
{
  static uint8_t cutoff_processing_prev = 0;
  static int8_t cutoffcnt0 = -1;
  static int8_t cutoffcnt1 = -1;
  static int8_t cutoffcnt2 = -1;
  static int8_t cutoffcnt3 = -1;
  static int8_t cutoffcnt4 = -1;
  static int8_t cutoffcnt5 = -1;
  static int8_t cutoffcnt6 = -1;
  static int8_t cy_prev = -1;

  if(cutoff_processing_prev)
  {
    cutoff_processing_prev--;
    Cutoff.Processing = 1;
  }
  else Cutoff.Processing = 0;

  //Cutoff always enabled
  if(1)
  {
    int32_t mode = gEcuParams.cutoffMode;
    float cutoffrpm = gEcuParams.cutoffRPM;
    if(rpm >= cutoffrpm)
    {
      if(mode == 0)
      {
        Cutoff.Processing = 1;
        cutoff_processing_prev = 32;
        Cutoff.Cutting = 1;
        return 0;
      }
      else if(mode == 1)
      {
        if(cutoffcnt0 == -1)
          cutoffcnt0 = 0;
      }
      else if(mode == 2)
      {
        if(cutoffcnt1 == -1)
          cutoffcnt1 = 0;
      }
      else if(mode == 3)
      {
        if(cutoffcnt2 == -1)
          cutoffcnt2 = 0;
      }
      else if(mode == 4)
      {
        if(cutoffcnt3 == -1)
          cutoffcnt3 = 0;
      }
      else if(mode == 5)
      {
        if(cutoffcnt4 == -1)
          cutoffcnt4 = 0;
      }
      else if(mode == 6)
      {
        if(cutoffcnt5 == -1)
          cutoffcnt5 = 0;
      }
      else if(mode == 7)
      {
        if(cutoffcnt6 == -1)
          cutoffcnt6 = 0;
      }
    }
  }

  if(cy_prev != cylinder)
  {
    cy_prev = cylinder;

    if(cutoffcnt0 >= 0)
    {
      Cutoff.Processing = 1;
      cutoff_processing_prev = 36+4+8;
      if(++cutoffcnt0 > 36)
        cutoffcnt0 = -1;
      else if(cutoffcnt0 <= 36-4) {
        Cutoff.Cutting = 1;
        return 0;
      }
    }

    if(cutoffcnt1 >= 0)
    {
      Cutoff.Processing = 1;
      cutoff_processing_prev = 24+4+8;
      if(++cutoffcnt1 > 24)
        cutoffcnt1 = -1;
      else if(cutoffcnt1 <= 24-4) {
        Cutoff.Cutting = 1;
        return 0;
      }
    }

    if(cutoffcnt2 >= 0)
    {
      Cutoff.Processing = 1;
      cutoff_processing_prev = 16+4+8;
      if(++cutoffcnt2 > 16)
        cutoffcnt2 = -1;
      else if(cutoffcnt2 <= 16-4) {
        Cutoff.Cutting = 1;
        return 0;
      }
    }

    if(cutoffcnt3 >= 0)
    {
      Cutoff.Processing = 1;
      cutoff_processing_prev = 8+4+8;
      if(++cutoffcnt3 > 8)
        cutoffcnt3 = -1;
      else if(cutoffcnt3 <= 8-4) {
        Cutoff.Cutting = 1;
        return 0;
      }
    }

    if(cutoffcnt4 >= 0)
    {
      Cutoff.Processing = 1;
      cutoff_processing_prev = 20+4+8;
      if(++cutoffcnt4 > 20)
        cutoffcnt4 = -1;
      else if((cutoffcnt4 % 5) != 0) {
        Cutoff.Cutting = 1;
        return 0;
      }
    }

    if(cutoffcnt5 >= 0)
    {
      Cutoff.Processing = 1;
      cutoff_processing_prev = 12+4+8;
      if(++cutoffcnt5 > 12)
        cutoffcnt5 = -1;
      else if((cutoffcnt5 % 3) != 0) {
        Cutoff.Cutting = 1;
        return 0;
      }
    }

    if(cutoffcnt6 >= 0)
    {
      Cutoff.Processing = 1;
      cutoff_processing_prev = 12+4+8;
      if(++cutoffcnt6 > 12)
        cutoffcnt6 = -1;
      else if((cutoffcnt6 % 3) == 0) {
        Cutoff.Cutting = 1;
        return 0;
      }
    }

  }
  Cutoff.Cutting = 0;
  return 1;
}

STATIC_INLINE uint8_t ecu_cutoff_inj_act(uint8_t cy_count, uint8_t cylinder, float rpm)
{
  float cutoffrpm = gEcuParams.cutoffRPM;

  //Just for safety purpose...
  if(rpm >= cutoffrpm + 250) {
    return 0;
  }

  return 1;
}

STATIC_INLINE void ecu_coil_saturate(uint8_t cy_count, uint8_t cylinder)
{
  if(cy_count == ECU_CYLINDERS_COUNT && cylinder < ECU_CYLINDERS_COUNT) {
    gIgnPorts[cylinder]->BSRR = gIgnPins[cylinder];
  } else if(cy_count == ECU_CYLINDERS_COUNT / 2) {
    if(cylinder == 0) {
      gIgnPorts[0]->BSRR = gIgnPins[0];
      gIgnPorts[3]->BSRR = gIgnPins[3];
    } else if(cylinder == 1) {
      gIgnPorts[1]->BSRR = gIgnPins[1];
      gIgnPorts[2]->BSRR = gIgnPins[2];
    }
  } else if(cy_count == 1) {
    gIgnPorts[0]->BSRR = gIgnPins[0];
    gIgnPorts[1]->BSRR = gIgnPins[1];
    gIgnPorts[2]->BSRR = gIgnPins[2];
    gIgnPorts[3]->BSRR = gIgnPins[3];
  }
}

STATIC_INLINE void ecu_coil_ignite(uint8_t cy_count, uint8_t cylinder)
{
  if(cy_count == ECU_CYLINDERS_COUNT && cylinder < ECU_CYLINDERS_COUNT) {
    gIgnPorts[cylinder]->BSRR = gIgnPins[cylinder] << 16;
  } else if(cy_count == ECU_CYLINDERS_COUNT / 2) {
    if(cylinder == 0) {
      gIgnPorts[0]->BSRR = gIgnPins[0] << 16;
      gIgnPorts[3]->BSRR = gIgnPins[3] << 16;
    } else if(cylinder == 1) {
      gIgnPorts[1]->BSRR = gIgnPins[1] << 16;
      gIgnPorts[2]->BSRR = gIgnPins[2] << 16;
    }
  } else if(cy_count == 1) {
    gIgnPorts[0]->BSRR = gIgnPins[0] << 16;
    gIgnPorts[1]->BSRR = gIgnPins[1] << 16;
    gIgnPorts[2]->BSRR = gIgnPins[2] << 16;
    gIgnPorts[3]->BSRR = gIgnPins[3] << 16;
  }
}

STATIC_INLINE void ecu_inject(uint8_t cy_count, uint8_t cylinder, uint32_t time)
{
  if(cy_count == ECU_CYLINDERS_COUNT && cylinder < ECU_CYLINDERS_COUNT) {
    injector_enable(cylinder, time);
  } else if(cy_count == ECU_CYLINDERS_COUNT / 2) {
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
  sEcuTable *table = &gEcuTable[ecu_get_table()];
  sCspsData csps = csps_data();
  static float oldanglesbeforeignite[ECU_CYLINDERS_COUNT] = {0,0,0,0};
  static float oldanglesbeforeinject[ECU_CYLINDERS_COUNT] = {0,0,0,0};
  static uint8_t saturated[ECU_CYLINDERS_COUNT] = { 1,1,1,1 };
  static uint8_t ignited[ECU_CYLINDERS_COUNT] = { 1,1,1,1 };
  static uint8_t injected[ECU_CYLINDERS_COUNT] = { 1,1,1,1 };
  static uint8_t injection[ECU_CYLINDERS_COUNT] = { 1,1,1,1 };
  static uint8_t inj_was_phased = 0;
  static uint8_t ign_was_phased = 0;
  float angle_injection[ECU_CYLINDERS_COUNT];
  float angle_ignition[ECU_CYLINDERS_COUNT];
  float anglesbeforeignite[ECU_CYLINDERS_COUNT];
  float anglesbeforeinject[ECU_CYLINDERS_COUNT];

  float angle = csps_getphasedangle(csps);
  float rpm = csps_getrpm(csps);
  float found = csps_isfound();
  float uspa = csps_getuspa(csps);
  float period = csps_getperiod(csps);
  float time_sat;
  float time_pulse;
  float angle_ignite;
  float saturate;
  float angles_injection_per_turn;
  float angles_ignition_per_turn;
  float inj_phase;
  float inj_pulse;
  float inj_angle;
  float cy_ignition[ECU_CYLINDERS_COUNT];
  float cy_injection[ECU_CYLINDERS_COUNT];
  uint8_t phased_ignition;
  uint8_t phased_injection;
  uint8_t cy_count_ignition;
  uint8_t cy_count_injection;
  uint8_t individual_coils;
  uint8_t single_coil;
  uint8_t use_tsps;
  uint8_t injector_channel;
  uint8_t start_allowed = gParameters.StartAllowed;

  injector_channel = table->inj_channel;
  single_coil = gEcuParams.isSingleCoil;
  individual_coils = gEcuParams.isIndividualCoils;
  use_tsps = gEcuParams.useTSPS;
  phased_injection = use_tsps && csps_isphased(csps);
  phased_ignition = phased_injection && individual_coils && !single_coil;
  cy_count_injection = phased_injection ? ECU_CYLINDERS_COUNT : ECU_CYLINDERS_COUNT / 2;
  cy_count_ignition = phased_ignition ? ECU_CYLINDERS_COUNT : ECU_CYLINDERS_COUNT / 2;
  angle_ignite = gParameters.IgnitionAngle;

  if(single_coil) {
    time_sat = period * 0.5f * 0.65f;
    time_pulse = period * 0.5f * 0.35f;
  } else {
    time_sat = gParameters.IgnitionPulse * 1000.0f;
    time_pulse = 2000;
  }

  inj_phase = gParameters.InjectionPhase;
  inj_pulse = gParameters.InjectionPulse;

  if(phased_ignition) {
    angles_ignition_per_turn = 720.0f;
    cy_ignition[0] = table->cy_corr_ignition[0] + angle_ignite;
    cy_ignition[1] = table->cy_corr_ignition[1] + angle_ignite;
    cy_ignition[2] = table->cy_corr_ignition[2] + angle_ignite;
    cy_ignition[3] = table->cy_corr_ignition[3] + angle_ignite;
  } else {
    angles_ignition_per_turn = 360.0f;
    saturated[2] = ignited[2] = 1;
    saturated[3] = ignited[3] = 1;
    cy_ignition[0] = (table->cy_corr_ignition[0] + table->cy_corr_ignition[3]) * 0.5f + angle_ignite;
    cy_ignition[1] = (table->cy_corr_ignition[1] + table->cy_corr_ignition[2]) * 0.5f + angle_ignite;
  }

  if(phased_injection) {
    angles_injection_per_turn = 720.0f;
    cy_injection[0] = (table->cy_corr_injection[0] + 1.0f) * inj_pulse;
    cy_injection[1] = (table->cy_corr_injection[1] + 1.0f) * inj_pulse;
    cy_injection[2] = (table->cy_corr_injection[2] + 1.0f) * inj_pulse;
    cy_injection[3] = (table->cy_corr_injection[3] + 1.0f) * inj_pulse;
  } else {
    angles_injection_per_turn = 360.0f;
    inj_pulse *= 0.5;
    injection[2] = injected[2] = 1;
    injection[3] = injected[3] = 1;
    cy_injection[0] = ((table->cy_corr_injection[0] + table->cy_corr_injection[3]) * 0.5f + 1.0f) * inj_pulse;
    cy_injection[1] = ((table->cy_corr_injection[1] + table->cy_corr_injection[2]) * 0.5f + 1.0f) * inj_pulse;
  }

  if(inj_was_phased != phased_injection) {
    inj_was_phased = phased_injection;
    for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
      oldanglesbeforeinject[i] = 0.0f;
    }
  }

  if(ign_was_phased != phased_ignition) {
    ign_was_phased = phased_ignition;
    for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
      oldanglesbeforeignite[i] = 0.0f;
    }
  }

  while(inj_phase > angles_injection_per_turn * 0.5f) {
    inj_phase -= angles_injection_per_turn;
  }

  if(phased_ignition) {
    for(int i = 0; i < cy_count_ignition; i++) {
      angle_ignition[i] = csps_getphasedangle_cy(csps, i, angle);
    }
  } else {
    angle_ignition[0] = csps_getangle14(csps);
    angle_ignition[1] = csps_getangle23from14(angle_ignition[0]);
  }

  if(phased_injection) {
    for(int i = 0; i < cy_count_injection; i++) {
      angle_injection[i] = csps_getphasedangle_cy(csps, i, angle);
    }
  } else {
    angle_injection[0] = csps_getangle14(csps);
    angle_injection[1] = csps_getangle23from14(angle_injection[0]);
  }

  if(found && start_allowed)
  {
    IGN_NALLOW_GPIO_Port->BSRR = IGN_NALLOW_Pin << 16;
    gInjChPorts[injector_channel]->BSRR = gInjChPins[injector_channel];
    gInjChPorts[injector_channel ^ 1]->BSRR = gInjChPins[injector_channel ^ 1] << 16;

    if(period < time_sat + time_pulse) {
      time_sat = period * ((float)time_sat / (float)(time_sat + time_pulse));
    }

    saturate = time_sat / uspa;
    inj_angle = inj_pulse / uspa;

    //Ignition part
    for(int i = 0; i < cy_count_ignition; i++)
    {
      if(angle_ignition[i] < -cy_ignition[i])
        anglesbeforeignite[i] = -angle_ignition[i] - cy_ignition[i];
      else
        anglesbeforeignite[i] = angles_ignition_per_turn - angle_ignition[i] - cy_ignition[i];

      if(anglesbeforeignite[i] - oldanglesbeforeignite[i] > 0.0f && anglesbeforeignite[i] - oldanglesbeforeignite[i] < 180.0f)
        anglesbeforeignite[i] = oldanglesbeforeignite[i];

      if(anglesbeforeignite[i] - saturate < 0.0f)
      {
        if(!saturated[i] && !ignited[i])
        {
          saturated[i] = 1;

          if(single_coil) {
            ecu_coil_saturate(1, 0);
          } else {
            if(ecu_cutoff_ign_act(cy_count_ignition, i, rpm))
              ecu_coil_saturate(cy_count_ignition, i);
          }
        }
      }

      if(oldanglesbeforeignite[i] - anglesbeforeignite[i] < -90.0f)
      {
        if(!ignited[i] && saturated[i])
        {
          ignited[i] = 1;
          saturated[i] = 0;

          if(single_coil) {
            if(ecu_cutoff_ign_act(cy_count_ignition, i, rpm))
              ecu_coil_ignite(1, 0);
          } else {
            ecu_coil_ignite(cy_count_ignition, i);
          }
        }
      }
      else ignited[i] = 0;

      oldanglesbeforeignite[i] = anglesbeforeignite[i];
    }

    //Injection part
    for(int i = 0; i < cy_count_injection; i++)
    {
      if(angle_injection[i] < inj_phase)
        anglesbeforeinject[i] = -angle_injection[i] + inj_phase;
      else
        anglesbeforeinject[i] = angles_injection_per_turn - angle_injection[i] + inj_phase;

      if(oldanglesbeforeinject[i] - anglesbeforeinject[i] > 0.0f && oldanglesbeforeinject[i] - anglesbeforeinject[i] > 180.0f)
        anglesbeforeinject[i] = oldanglesbeforeinject[i];

      if(anglesbeforeinject[i] - inj_angle < 0.0f)
      {
        if(!injection[i])
        {
          injection[i] = 1;
          if(ecu_cutoff_inj_act(cy_count_injection, i, rpm) && cy_injection[i] > 0.0f)
            ecu_inject(cy_count_injection, i, cy_injection[i]);
        }
      }

      if(oldanglesbeforeinject[i] - anglesbeforeinject[i] < -90.0f)
      {
        if(!injected[i] && injection[i])
        {
          injection[i] = 0;
          injector_isenabled(i, &injected[i]);
          injected[i] ^= 1;
        }
      }
      else injected[i] = 0;

      oldanglesbeforeinject[i] = anglesbeforeinject[i];
    }
  } else {
    IGN_NALLOW_GPIO_Port->BSRR = IGN_NALLOW_Pin;
    for(int i = 0; i < ITEMSOF(gInjChPins); i++)
      gInjChPorts[i]->BSRR = gInjChPins[i] << 16;
    for(int i = 0; i < ITEMSOF(gIgnPorts); i++)
      gIgnPorts[i]->BSRR = gIgnPins[i] << 16;

  }
}

static void ecu_fuelpump_process(void)
{
  static uint32_t active_last = 0;
  static uint8_t active = 0;
  static uint8_t was_rotating = 1;
  uint32_t now = Delay_Tick;
  uint8_t rotates = csps_isrotates();
  uint8_t time_to_last = DelayDiff(now, active_last) < 1000000;

  if(gForceParameters.Enable.FuelPumpRelay) {
    active = gForceParameters.FuelPumpRelay > 0;
    active_last = now;
  } else {
    if((was_rotating && time_to_last) || rotates) {
      if(rotates) {
        active_last = now;
        was_rotating = 1;
      }
      active = 1;
    } else {
      active = 0;
      was_rotating = 0;
    }
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

  if(gForceParameters.Enable.FanRelay) {
    out_set_fan(gForceParameters.FanRelay > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  } else if(status != HAL_OK) {
    out_set_fan(GPIO_PIN_SET);
  } else if(fan_state != GPIO_PIN_RESET) {
    if(engine_temp < temp_low) {
      out_set_fan(GPIO_PIN_RESET);
    }
  } else if(engine_temp >= temp_high) {
    out_set_fan(GPIO_PIN_SET);
  }
}

#define CHECK_STATUS(status, iserror, cod, link) \
  if(((status).link)) { \
    gCheckBitmap[cod >> 3] |= 1 << (cod & 7); \
    iserror |= 1; \
  }

static void ecu_checkengine_loop(void)
{
  static uint32_t hal_error_last = 0;
  static uint8_t was_error = 0;
  uint32_t now = Delay_Tick;
  uint32_t hal_now = HAL_GetTick();
  uint8_t running = csps_isrunning();
  uint8_t iserror = 0;
  uint8_t status_reset = gStatusReset;

  memset(gCheckBitmap, 0, sizeof(gCheckBitmap));

  CHECK_STATUS(gStatus, iserror, CheckFlashLoadFailure, Flash.Struct.Load != HAL_OK);
  CHECK_STATUS(gStatus, iserror, CheckFlashSaveFailure, Flash.Struct.Save != HAL_OK);
  CHECK_STATUS(gStatus, iserror, CheckFlashInitFailure, Flash.Struct.Init != HAL_OK);
  CHECK_STATUS(gStatus, iserror, CheckBkpsramSaveFailure, Bkpsram.Struct.Save != HAL_OK);
  CHECK_STATUS(gStatus, iserror, CheckBkpsramLoadFailure, Bkpsram.Struct.Load != HAL_OK);

  CHECK_STATUS(gStatus, iserror, CheckSensorMapFailure, Sensors.Struct.Map != HAL_OK);
  CHECK_STATUS(gStatus, iserror, CheckSensorKnockFailure, Sensors.Struct.Knock != HAL_OK);
  CHECK_STATUS(gStatus, iserror, CheckSensorCspsFailure, Sensors.Struct.Csps != HAL_OK);
  CHECK_STATUS(gStatus, iserror, CheckSensorTspsFailure, Sensors.Struct.Tsps != HAL_OK);
  CHECK_STATUS(gStatus, iserror, CheckSensorAirTempFailure, Sensors.Struct.AirTemp != HAL_OK);
  CHECK_STATUS(gStatus, iserror, CheckSensorEngineTempFailure, Sensors.Struct.EngineTemp != HAL_OK);
  CHECK_STATUS(gStatus, iserror, CheckSensorTPSFailure, Sensors.Struct.ThrottlePos != HAL_OK);
  CHECK_STATUS(gStatus, iserror, CheckSensorRefVoltageFailure, Sensors.Struct.ReferenceVoltage != HAL_OK);
  CHECK_STATUS(gStatus, iserror, CheckSensorPwrVoltageFailure, Sensors.Struct.PowerVoltage != HAL_OK);
  CHECK_STATUS(gStatus, iserror, CheckSensorLambdaFailure, Sensors.Struct.Lambda != HAL_OK);
  CHECK_STATUS(gStatus, iserror, CheckOutputDriverFailure, OutputStatus != HAL_OK);

  CHECK_STATUS(gStatus, iserror, CheckInjector4OpenCircuit, OutputDiagnostic.Injectors.Diagnostic.Data.InjCy4 == OutputDiagOpenCircuit);
  CHECK_STATUS(gStatus, iserror, CheckInjector4ShortToBatOrOverheat, OutputDiagnostic.Injectors.Diagnostic.Data.InjCy4 == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(gStatus, iserror, CheckInjector4ShortToGND, OutputDiagnostic.Injectors.Diagnostic.Data.InjCy4 == OutputDiagShortToGnd);
  CHECK_STATUS(gStatus, iserror, CheckInjector3OpenCircuit, OutputDiagnostic.Injectors.Diagnostic.Data.InjCy3 == OutputDiagOpenCircuit);
  CHECK_STATUS(gStatus, iserror, CheckInjector3ShortToBatOrOverheat, OutputDiagnostic.Injectors.Diagnostic.Data.InjCy3 == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(gStatus, iserror, CheckInjector3ShortToGND, OutputDiagnostic.Injectors.Diagnostic.Data.InjCy3 == OutputDiagShortToGnd);
  CHECK_STATUS(gStatus, iserror, CheckInjector2OpenCircuit, OutputDiagnostic.Injectors.Diagnostic.Data.InjCy2 == OutputDiagOpenCircuit);
  CHECK_STATUS(gStatus, iserror, CheckInjector2ShortToBatOrOverheat, OutputDiagnostic.Injectors.Diagnostic.Data.InjCy2 == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(gStatus, iserror, CheckInjector2ShortToGND, OutputDiagnostic.Injectors.Diagnostic.Data.InjCy2 == OutputDiagShortToGnd);
  CHECK_STATUS(gStatus, iserror, CheckInjector1OpenCircuit, OutputDiagnostic.Injectors.Diagnostic.Data.InjCy1 == OutputDiagOpenCircuit);
  CHECK_STATUS(gStatus, iserror, CheckInjector1ShortToBatOrOverheat, OutputDiagnostic.Injectors.Diagnostic.Data.InjCy1 == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(gStatus, iserror, CheckInjector1ShortToGND, OutputDiagnostic.Injectors.Diagnostic.Data.InjCy1 == OutputDiagShortToGnd);
  CHECK_STATUS(gStatus, iserror, CheckInjectorCommunicationFailure, OutputDiagnostic.Injectors.Availability != HAL_OK);

  //CHECK_STATUS(gStatus, iserror, CheckCheckEngineOpenCirtuit, OutputDiagnostic.Outs1.Diagnostic.Data.CheckEngine == OutputDiagOpenCircuit);
  CHECK_STATUS(gStatus, iserror, CheckCheckEngineShortToBatOrOverheat, OutputDiagnostic.Outs1.Diagnostic.Data.CheckEngine == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(gStatus, iserror, CheckCheckEngineShortToGND, OutputDiagnostic.Outs1.Diagnostic.Data.CheckEngine == OutputDiagShortToGnd);
  //CHECK_STATUS(gStatus, iserror, CheckSpeedMeterOpenCirtuit, OutputDiagnostic.Outs1.Diagnostic.Data.Speedmeeter == OutputDiagOpenCircuit);
  CHECK_STATUS(gStatus, iserror, CheckSpeedMeterShortToBatOrOverheat, OutputDiagnostic.Outs1.Diagnostic.Data.Speedmeeter == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(gStatus, iserror, CheckSpeedMeterShortToGND, OutputDiagnostic.Outs1.Diagnostic.Data.Speedmeeter == OutputDiagShortToGnd);
  //CHECK_STATUS(gStatus, iserror, CheckTachometerOpenCirtuit, OutputDiagnostic.Outs1.Diagnostic.Data.Tachometer == OutputDiagOpenCircuit);
  CHECK_STATUS(gStatus, iserror, CheckTachometerShortToBatOrOverheat, OutputDiagnostic.Outs1.Diagnostic.Data.Tachometer == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(gStatus, iserror, CheckTachometerShortToGND, OutputDiagnostic.Outs1.Diagnostic.Data.Tachometer == OutputDiagShortToGnd);
  CHECK_STATUS(gStatus, iserror, CheckFuelPumpOpenCirtuit, OutputDiagnostic.Outs1.Diagnostic.Data.FuelPumpRelay == OutputDiagOpenCircuit);
  CHECK_STATUS(gStatus, iserror, CheckFuelPumpShortToBatOrOverheat, OutputDiagnostic.Outs1.Diagnostic.Data.FuelPumpRelay == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(gStatus, iserror, CheckFuelPumpShortToGND, OutputDiagnostic.Outs1.Diagnostic.Data.FuelPumpRelay == OutputDiagShortToGnd);
  CHECK_STATUS(gStatus, iserror, CheckOutputs1CommunicationFailure, OutputDiagnostic.Outs1.Availability != HAL_OK);

  //CHECK_STATUS(gStatus, iserror, CheckOutRsvd2OpenCirtuit, OutputDiagnostic.Outs2.Diagnostic.Data.OutRsvd2 == OutputDiagOpenCircuit);
  CHECK_STATUS(gStatus, iserror, CheckOutRsvd2ShortToBatOrOverheat, OutputDiagnostic.Outs2.Diagnostic.Data.OutRsvd2 == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(gStatus, iserror, CheckOutRsvd2ShortToGND, OutputDiagnostic.Outs2.Diagnostic.Data.OutRsvd2 == OutputDiagShortToGnd);
  //CHECK_STATUS(gStatus, iserror, CheckOutRsvd1OpenCirtuit, OutputDiagnostic.Outs2.Diagnostic.Data.OutRsvd1 == OutputDiagOpenCircuit);
  CHECK_STATUS(gStatus, iserror, CheckOutRsvd1ShortToBatOrOverheat, OutputDiagnostic.Outs2.Diagnostic.Data.OutRsvd1 == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(gStatus, iserror, CheckOutRsvd1ShortToGND, OutputDiagnostic.Outs2.Diagnostic.Data.OutRsvd1 == OutputDiagShortToGnd);
  //CHECK_STATUS(gStatus, iserror, CheckStarterRelayOpenCirtuit, OutputDiagnostic.Outs2.Diagnostic.Data.StarterRelay == OutputDiagOpenCircuit);
  CHECK_STATUS(gStatus, iserror, CheckStarterRelayShortToBatOrOverheat, OutputDiagnostic.Outs2.Diagnostic.Data.StarterRelay == OutputDiagShortToBatOrOvertemp);
  //CHECK_STATUS(gStatus, iserror, CheckStarterRelayShortToGND, OutputDiagnostic.Outs2.Diagnostic.Data.StarterRelay == OutputDiagShortToGnd);
  CHECK_STATUS(gStatus, iserror, CheckFamRelayOpenCirtuit, OutputDiagnostic.Outs2.Diagnostic.Data.FanRelay == OutputDiagOpenCircuit);
  CHECK_STATUS(gStatus, iserror, CheckFanRelayShortToBatOrOverheat, OutputDiagnostic.Outs2.Diagnostic.Data.FanRelay == OutputDiagShortToBatOrOvertemp);
  //CHECK_STATUS(gStatus, iserror, CheckFanRelayShortToGND, OutputDiagnostic.Outs2.Diagnostic.Data.FanRelay == OutputDiagShortToGnd);
  CHECK_STATUS(gStatus, iserror, CheckOutputs2CommunicationFailure, OutputDiagnostic.Outs2.Availability != HAL_OK);

  CHECK_STATUS(gStatus, iserror, CheckIdleValveFailure, OutputDiagnostic.IdleValvePosition.Status != HAL_OK);
  CHECK_STATUS(gStatus, iserror, CheckIdleValveDriverFailure, IdleValvePosition != HAL_OK);
  CHECK_STATUS(gStatus, iserror, CheckInjectionUnderflow, InjectionUnderflow != HAL_OK);

  if(gEcuParams.useLambdaSensor) {
    CHECK_STATUS(gStatus, iserror, CheckLambdaCommunicationFailure, O2Status != HAL_OK);
    CHECK_STATUS(gStatus, iserror, CheckLambdaVMShortToBat, O2Diagnostic.VM == O2DiagShortToBat);
    CHECK_STATUS(gStatus, iserror, CheckLambdaVMLowBattery, O2Diagnostic.VM == O2DiagNoPower);
    CHECK_STATUS(gStatus, iserror, CheckLambdaVMShortToGND, O2Diagnostic.VM == O2DiagShortToGnd);
    CHECK_STATUS(gStatus, iserror, CheckLambdaUNShortToBat, O2Diagnostic.UN == O2DiagShortToBat);
    CHECK_STATUS(gStatus, iserror, CheckLambdaUNLowBattery, O2Diagnostic.UN == O2DiagNoPower);
    CHECK_STATUS(gStatus, iserror, CheckLambdaUNShortToGND, O2Diagnostic.UN == O2DiagShortToGnd);
    CHECK_STATUS(gStatus, iserror, CheckLambdaIAIPShortToBat, O2Diagnostic.IAIP == O2DiagShortToBat);
    CHECK_STATUS(gStatus, iserror, CheckLambdaIAIPLowBattery, O2Diagnostic.IAIP == O2DiagNoPower);
    CHECK_STATUS(gStatus, iserror, CheckLambdaIAIPShortToGND, O2Diagnostic.IAIP == O2DiagShortToGnd);
    CHECK_STATUS(gStatus, iserror, CheckLambdaDIAHGDShortToBat, O2Diagnostic.DIAHGD == O2DiagShortToBat);
    CHECK_STATUS(gStatus, iserror, CheckLambdaDIAHGDOpenCirtuit, O2Diagnostic.DIAHGD == O2DiagNoPower);
    CHECK_STATUS(gStatus, iserror, CheckLambdaDIAHGDShortToGND, O2Diagnostic.DIAHGD == O2DiagShortToGnd);
  }
  if(gEcuParams.useKnockSensor) {
    CHECK_STATUS(gStatus, iserror, CheckKnockDetonationFound, KnockStatus == KnockStatusDedonation);
    CHECK_STATUS(gStatus, iserror, CheckKnockLowNoiseLevel, KnockStatus == KnockStatusLowNoise);
  }

  if(iserror) {
    was_error = 1;
    hal_error_last = now;
  }

  if(was_error) {
    if(HAL_DelayDiff(hal_now, hal_error_last) < 60000) {
      iserror = 1;
    } else {
      was_error = 0;
    }
  }

  for(int i = 0; i < CHECK_BITMAP_SIZE; i++) {
    gEcuCriticalBackup.CheckBitmapRecorded[i] |= gCheckBitmap[i];
  }

  gEcuIsError = iserror;

  if(status_reset) {
    memset(&gStatus, 0, sizeof(gStatus));
    memset(&gCheckBitmap, 0, sizeof(gCheckBitmap));
    memset(&gEcuCriticalBackup.CheckBitmapRecorded, 0, sizeof(gEcuCriticalBackup.CheckBitmapRecorded));
    gEcuIsError = 0;
    was_error = 0;
    iserror = 0;
    gStatusReset = 0;
  }

  if(gForceParameters.Enable.CheckEngine) {
    out_set_checkengine(gForceParameters.CheckEngine > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    was_error = 0;
    iserror = 0;
  } else {

    if(!running || was_error) {
      out_set_checkengine(GPIO_PIN_SET);
    } else {
      out_set_checkengine(GPIO_PIN_RESET);
    }
  }
}


static void ecu_drag_loop(void)
{
  uint32_t now = Delay_Tick;
  float rpm = gParameters.RPM;
  float speed = gParameters.Speed;
  if(Drag.Ready)
  {
    if(Drag.Completed)
    {
      Drag.StopTime = 0;
      Drag.Ready = 0;
      Drag.Started = 0;
    }
    else
    {
      if(Drag.Started)
      {
        if(DelayDiff(now, Drag.TimeLast) >= DRAG_POINTS_DISTANCE)
        {
          Drag.TimeLast = now;
          if(Drag.PointsCount < DRAG_MAX_POINTS)
          {
            Drag.StopTime = now;
            Drag.Points[Drag.PointsCount].RPM = rpm;
            Drag.Points[Drag.PointsCount].Speed = speed;
            Drag.Points[Drag.PointsCount].Acceleration = gParameters.Acceleration;
            Drag.Points[Drag.PointsCount].Pressure = gParameters.ManifoldAirPressure;
            Drag.Points[Drag.PointsCount].Ignition = gParameters.IgnitionAngle;
            Drag.Points[Drag.PointsCount].Mixture = gParameters.FuelRatio;
            Drag.Points[Drag.PointsCount].CycleAirFlow = gParameters.CyclicAirFlow;
            Drag.Points[Drag.PointsCount].MassAirFlow = gParameters.MassAirFlow;
            Drag.Points[Drag.PointsCount].Throttle = gParameters.ThrottlePosition;
            Drag.Points[Drag.PointsCount].Time = DelayDiff(now, Drag.StartTime);
            Drag.PointsCount++;

            if(Drag.FromSpeed < Drag.ToSpeed)
            {
              if(speed >= Drag.ToSpeed)
              {
                Drag.Started = 0;
                Drag.Status = 0;
                Drag.Ready = 0;
                Drag.Completed = 1;
                Drag.TimeLast = 0;
              }
              else if(speed < Drag.FromSpeed - 3.0f)
              {
                Drag.Started = 0;
                Drag.Status = 4;
                Drag.Ready = 0;
                Drag.Completed = 1;
                Drag.TimeLast = 0;
              }
            }
            else if(Drag.FromSpeed > Drag.ToSpeed)
            {
              if(speed <= Drag.ToSpeed)
              {
                Drag.Started = 0;
                Drag.Status = 0;
                Drag.Ready = 0;
                Drag.Completed = 1;
                Drag.TimeLast = 0;
              }
              else if(speed > Drag.FromSpeed + 3.0f)
              {
                Drag.Started = 0;
                Drag.Status = 4;
                Drag.Ready = 0;
                Drag.Completed = 1;
                Drag.TimeLast = 0;
              }
            }
          }
          else
          {
            Drag.Started = 0;
            Drag.Status = 5;
            Drag.Ready = 0;
            Drag.Completed = 1;
            Drag.TimeLast = 0;
          }
        }
      }
      else
      {
        if(Drag.FromSpeed < Drag.ToSpeed)
        {
          if(speed >= Drag.FromSpeed)
          {
            if(Drag.TimeLast != 0)
            {
              Drag.TimeLast = now;
              Drag.Started = 1;
              Drag.Status = 0;
              Drag.PointsCount = 0;
              Drag.StartTime = now;
              Drag.StopTime = now;
              Drag.Points[Drag.PointsCount].RPM = rpm;
              Drag.Points[Drag.PointsCount].Speed = speed;
              Drag.Points[Drag.PointsCount].Acceleration = gParameters.Acceleration;
              Drag.Points[Drag.PointsCount].Pressure = gParameters.ManifoldAirPressure;
              Drag.Points[Drag.PointsCount].Ignition = gParameters.IgnitionAngle;
              Drag.Points[Drag.PointsCount].Mixture = gParameters.FuelRatio;
              Drag.Points[Drag.PointsCount].CycleAirFlow = gParameters.CyclicAirFlow;
              Drag.Points[Drag.PointsCount].MassAirFlow = gParameters.MassAirFlow;
              Drag.Points[Drag.PointsCount].Throttle = gParameters.ThrottlePosition;
              Drag.Points[Drag.PointsCount].Time = DelayDiff(now, Drag.StartTime);
              Drag.PointsCount++;
            }
          }
          else
          {
            Drag.TimeLast = now;
          }
        }
        else if(Drag.FromSpeed > Drag.ToSpeed)
        {
          if(speed <= Drag.FromSpeed)
          {
            if(Drag.TimeLast != 0)
            {
              Drag.TimeLast = now;
              Drag.Started = 1;
              Drag.PointsCount = 0;
              Drag.StartTime = now;
              Drag.StopTime = now;
              Drag.Points[Drag.PointsCount].RPM = rpm;
              Drag.Points[Drag.PointsCount].Speed = speed;
              Drag.Points[Drag.PointsCount].Acceleration = gParameters.Acceleration;
              Drag.Points[Drag.PointsCount].Pressure = gParameters.ManifoldAirPressure;
              Drag.Points[Drag.PointsCount].Ignition = gParameters.IgnitionAngle;
              Drag.Points[Drag.PointsCount].Mixture = gParameters.FuelRatio;
              Drag.Points[Drag.PointsCount].CycleAirFlow = gParameters.CyclicAirFlow;
              Drag.Points[Drag.PointsCount].MassAirFlow = gParameters.MassAirFlow;
              Drag.Points[Drag.PointsCount].Throttle = gParameters.ThrottlePosition;
              Drag.Points[Drag.PointsCount].Time = DelayDiff(now, Drag.StartTime);
              Drag.PointsCount++;
            }
          }
          else
          {
            Drag.TimeLast = now;
          }
        }
      }
    }
  }
}

static void ecu_mem_loop(void)
{
  int8_t flashstatus = 0;
  if(!Mem.issaving && !Mem.isloading)
  {
    if(Mem.savereq && !Mem.isloading) {
      Mem.issaving = 1;
      //CanDeinit = 0;
    }
    else if(Mem.loadreq && !Mem.issaving)
      Mem.isloading = 1;
  }

  if(Mem.issaving)
  {
    //CanDeinit = 0;
    flashstatus = config_save_all(&gEcuParams, gEcuTable, ITEMSOF(gEcuTable));
    if(flashstatus)
    {
      PK_SaveConfigAcknowledge.ErrorCode = flashstatus == 1 ? 0 : 1;
      PK_SendCommand(Mem.savereqsrc, &PK_SaveConfigAcknowledge, sizeof(PK_SaveConfigAcknowledge));
      gStatus.Flash.Struct.Save = flashstatus;
      Mem.issaving = 0;
      //CanDeinit = 1;
      Mem.savereq = 0;
    }
  }
  else if(Mem.isloading)
  {
    flashstatus = config_load_all(&gEcuParams, gEcuTable, ITEMSOF(gEcuTable));
    if(flashstatus)
    {
      PK_RestoreConfigAcknowledge.ErrorCode = flashstatus == 1 ? 0 : 1;
      PK_SendCommand(Mem.loadreqsrc, &PK_RestoreConfigAcknowledge, sizeof(PK_RestoreConfigAcknowledge));
      gStatus.Flash.Struct.Load = flashstatus;
      Mem.isloading = 0;
      Mem.loadreq = 0;
    }
  }
}

static void ecu_bluetooth_loop(void)
{
  static uint8_t bt_was_enabled = 0;
  uint8_t enabled = gEcuParams.isBluetoothEnabled;
  uint16_t pin = gEcuParams.bluetoothPin;
  const char *name = gEcuParams.bluetoothName;

  if(bt_was_enabled != enabled) {
    bt_was_enabled = enabled;
    if(enabled) {
      bluetooth_enable(name, pin);
    } else {
      bluetooth_disable();
    }
  }
}

static void ecu_diagnostic_loop(void)
{
  sOutputDiagnostic output_diagnostic;
  sO2Diagnostic o2_diagnostic;

  if(outputs_get_diagnostic(&output_diagnostic) == HAL_OK) {
    gStatus.OutputStatus = HAL_OK;
    gStatus.OutputDiagnostic = output_diagnostic;
  } else {
    gStatus.OutputStatus = HAL_ERROR;
  }

  if(sens_get_o2_diagnostic(&o2_diagnostic) == HAL_OK) {
    gStatus.O2Status = HAL_OK;
    gStatus.O2Diagnostic = o2_diagnostic;
  } else {
    gStatus.O2Status = HAL_ERROR;
  }

}

void ecu_init(void)
{
  ecu_config_init();

  ecu_set_table(gEcuParams.startupTableNumber);

  ecu_pid_init();

  ecu_init_post_init();

  gParameters.StartAllowed = 1;
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
  ecu_drag_loop();
  ecu_mem_loop();
  ecu_bluetooth_loop();
  ecu_diagnostic_loop();
  ecu_checkengine_loop();
}

void ecu_parse_command(eTransChannels xChaSrc, uint8_t * msgBuf, uint32_t length)
{
  static uint32_t pclastsent = 0;
  uint32_t now = Delay_Tick;
  uint32_t offset;
  uint32_t size;
  uint32_t table;
  uint32_t tablesize;
  uint32_t configsize;
  uint32_t dragpoint;
  if(xChaSrc == etrPC)
  {
    if(DelayDiff(now, pclastsent) > 100000)
    {
      pclastsent = now;
      PK_SendCommand(xChaSrc, &PK_PcConnected, sizeof(PK_PcConnected));
    }
  }
  switch(msgBuf[0])
  {
    case PK_PingID :
      PK_Copy(&PK_Ping, msgBuf);
      PK_Pong.RandomPong = PK_Ping.RandomPing;
      PK_SendCommand(xChaSrc, &PK_Pong, sizeof(PK_Pong));
      break;

    case PK_PongID :
      PK_Copy(&PK_Pong, msgBuf);
      (void)PK_Pong.RandomPong;
      break;

    case PK_GeneralStatusRequestID :
      //PK_Copy(&PK_GeneralStatusRequest, msgBuf);
      PK_GeneralStatusResponse.RPM = gParameters.RPM;
      PK_GeneralStatusResponse.Pressure = gParameters.ManifoldAirPressure;
      PK_GeneralStatusResponse.Voltage = gParameters.PowerVoltage;
      PK_GeneralStatusResponse.EngineTemp = gParameters.EngineTemp;
      PK_GeneralStatusResponse.FuelUsage = gParameters.FuelConsumption;
      PK_GeneralStatusResponse.check = gEcuIsError;
      PK_GeneralStatusResponse.tablenum = ecu_get_table();
      strcpy(PK_GeneralStatusResponse.tablename, gEcuTable[ecu_get_table()].name);
      PK_SendCommand(xChaSrc, &PK_GeneralStatusResponse, sizeof(PK_GeneralStatusResponse));
      break;

    case PK_TableMemoryRequestID :
      PK_Copy(&PK_TableMemoryRequest, msgBuf);
      offset = PK_TableMemoryData.offset = PK_TableMemoryRequest.offset;
      size = PK_TableMemoryData.size = PK_TableMemoryRequest.size;
      table = PK_TableMemoryData.table = PK_TableMemoryRequest.table;
      tablesize = PK_TableMemoryData.tablesize = PK_TableMemoryRequest.tablesize;
      PK_TableMemoryData.ErrorCode = 0;

      if(tablesize != sizeof(sEcuTable))
        PK_TableMemoryData.ErrorCode = 1;

      if(size + offset > sizeof(sEcuTable))
        PK_TableMemoryData.ErrorCode = 2;

      if(size > PACKET_TABLE_MAX_SIZE || size > sizeof(sEcuTable))
        PK_TableMemoryData.ErrorCode = 3;

      if(table >= TABLE_SETUPS_MAX)
        PK_TableMemoryData.ErrorCode = 4;

      if(PK_TableMemoryData.ErrorCode == 0)
      {
        memcpy(&PK_TableMemoryData.data[0], &((uint8_t*)&gEcuTable[table])[offset], size);
        memset(&PK_TableMemoryData.data[size], 0, sizeof(PK_TableMemoryData.data) - size);
      }
      else
      {
        memset(&PK_TableMemoryData.data[0], 0, sizeof(PK_TableMemoryData.data));
      }
      PK_SendCommand(xChaSrc, &PK_TableMemoryData, sizeof(PK_TableMemoryData));
      break;

    case PK_TableMemoryDataID :
      PK_Copy(&PK_TableMemoryData, msgBuf);
      offset = PK_TableMemoryAcknowledge.offset = PK_TableMemoryData.offset;
      size = PK_TableMemoryAcknowledge.size = PK_TableMemoryData.size;
      table = PK_TableMemoryAcknowledge.table = PK_TableMemoryData.table;
      tablesize = PK_TableMemoryAcknowledge.tablesize = PK_TableMemoryData.tablesize;
      PK_TableMemoryAcknowledge.ErrorCode = 0;

      if(tablesize != sizeof(sEcuTable))
        PK_TableMemoryAcknowledge.ErrorCode = 1;

      if(size + offset > sizeof(sEcuTable))
        PK_TableMemoryAcknowledge.ErrorCode = 2;

      if(size > PACKET_TABLE_MAX_SIZE || size > sizeof(sEcuTable))
        PK_TableMemoryAcknowledge.ErrorCode = 3;

      if(table >= TABLE_SETUPS_MAX)
        PK_TableMemoryAcknowledge.ErrorCode = 4;

      if(PK_TableMemoryAcknowledge.ErrorCode == 0)
      {
        memcpy(&((uint8_t*)&gEcuTable[table])[offset], &PK_TableMemoryData.data[0], size);
      }

      PK_SendCommand(xChaSrc, &PK_TableMemoryAcknowledge, sizeof(PK_TableMemoryAcknowledge));
      break;

    case PK_ConfigMemoryRequestID :
      PK_Copy(&PK_ConfigMemoryRequest, msgBuf);
      offset = PK_ConfigMemoryData.offset = PK_ConfigMemoryRequest.offset;
      size = PK_ConfigMemoryData.size = PK_ConfigMemoryRequest.size;
      configsize = PK_ConfigMemoryData.configsize = PK_ConfigMemoryRequest.configsize;
      PK_ConfigMemoryData.ErrorCode = 0;

      if(configsize != sizeof(gEcuParams))
        PK_ConfigMemoryData.ErrorCode = 1;

      if(size + offset > sizeof(gEcuParams))
        PK_ConfigMemoryData.ErrorCode = 2;

      if(size > sizeof(gEcuParams) || size > PACKET_CONFIG_MAX_SIZE)
        PK_ConfigMemoryData.ErrorCode = 3;

      if(PK_ConfigMemoryData.ErrorCode == 0)
      {
        memcpy(&PK_ConfigMemoryData.data[0], &((uint8_t*)&gEcuParams)[offset], size);
        memset(&PK_ConfigMemoryData.data[size], 0, sizeof(PK_ConfigMemoryData.data) - size);
      }
      else
      {
        memset(&PK_ConfigMemoryData.data[0], 0, sizeof(PK_ConfigMemoryData.data));
      }
      PK_SendCommand(xChaSrc, &PK_ConfigMemoryData, sizeof(PK_ConfigMemoryData));
      break;

    case PK_ConfigMemoryDataID :
      PK_Copy(&PK_ConfigMemoryData, msgBuf);
      offset = PK_ConfigMemoryAcknowledge.offset = PK_ConfigMemoryData.offset;
      size = PK_ConfigMemoryAcknowledge.size = PK_ConfigMemoryData.size;
      configsize = PK_ConfigMemoryAcknowledge.configsize = PK_ConfigMemoryData.configsize;
      PK_ConfigMemoryAcknowledge.ErrorCode = 0;

      if(configsize != sizeof(gEcuParams))
        PK_ConfigMemoryData.ErrorCode = 1;

      if(size + offset > sizeof(gEcuParams))
        PK_ConfigMemoryData.ErrorCode = 2;

      if(size > sizeof(gEcuParams) || size > PACKET_CONFIG_MAX_SIZE)
        PK_ConfigMemoryData.ErrorCode = 3;

      if(PK_ConfigMemoryAcknowledge.ErrorCode == 0)
      {
        memcpy(&((uint8_t*)&gEcuParams)[offset], &PK_ConfigMemoryData.data[0], size);
      }

      PK_SendCommand(xChaSrc, &PK_ConfigMemoryAcknowledge, sizeof(PK_ConfigMemoryAcknowledge));
      break;

    case PK_CriticalMemoryRequestID :
      PK_Copy(&PK_CriticalMemoryRequest, msgBuf);
      offset = PK_CriticalMemoryData.offset = PK_CriticalMemoryRequest.offset;
      size = PK_CriticalMemoryData.size = PK_CriticalMemoryRequest.size;
      configsize = PK_CriticalMemoryData.configsize = PK_CriticalMemoryRequest.configsize;
      PK_CriticalMemoryData.ErrorCode = 0;

      if(configsize != sizeof(gEcuCriticalBackup))
        PK_CriticalMemoryData.ErrorCode = 1;

      if(size + offset > sizeof(gEcuCriticalBackup))
        PK_CriticalMemoryData.ErrorCode = 2;

      if(size > sizeof(gEcuCriticalBackup) || size > PACKET_CONFIG_MAX_SIZE)
        PK_CriticalMemoryData.ErrorCode = 3;

      if(PK_CriticalMemoryData.ErrorCode == 0)
      {
        memcpy(&PK_CriticalMemoryData.data[0], &((uint8_t*)&gEcuCriticalBackup)[offset], size);
        memset(&PK_CriticalMemoryData.data[size], 0, sizeof(PK_CriticalMemoryData.data) - size);
      }
      else
      {
        memset(&PK_CriticalMemoryData.data[0], 0, sizeof(PK_CriticalMemoryData.data));
      }
      PK_SendCommand(xChaSrc, &PK_CriticalMemoryData, sizeof(PK_CriticalMemoryData));
      break;

    case PK_CriticalMemoryDataID :
      PK_Copy(&PK_CriticalMemoryData, msgBuf);
      offset = PK_CriticalMemoryAcknowledge.offset = PK_CriticalMemoryData.offset;
      size = PK_CriticalMemoryAcknowledge.size = PK_CriticalMemoryData.size;
      configsize = PK_CriticalMemoryAcknowledge.configsize = PK_CriticalMemoryData.configsize;
      PK_CriticalMemoryAcknowledge.ErrorCode = 0;

      if(configsize != sizeof(gEcuCriticalBackup))
        PK_CriticalMemoryData.ErrorCode = 1;

      if(size + offset > sizeof(gEcuCriticalBackup))
        PK_CriticalMemoryData.ErrorCode = 2;

      if(size > sizeof(gEcuCriticalBackup) || size > PACKET_CONFIG_MAX_SIZE)
        PK_CriticalMemoryData.ErrorCode = 3;

      if(PK_CriticalMemoryAcknowledge.ErrorCode == 0)
      {
        memcpy(&((uint8_t*)&gEcuCriticalBackup)[offset], &PK_CriticalMemoryData.data[0], size);
      }

      PK_SendCommand(xChaSrc, &PK_CriticalMemoryAcknowledge, sizeof(PK_CriticalMemoryAcknowledge));
      break;

    case PK_CorrectionsMemoryRequestID :
      PK_Copy(&PK_CorrectionsMemoryRequest, msgBuf);
      offset = PK_CorrectionsMemoryData.offset = PK_CorrectionsMemoryRequest.offset;
      size = PK_CorrectionsMemoryData.size = PK_CorrectionsMemoryRequest.size;
      configsize = PK_CorrectionsMemoryData.configsize = PK_CorrectionsMemoryRequest.configsize;
      PK_CorrectionsMemoryData.ErrorCode = 0;

      if(configsize != sizeof(gEcuCorrections))
        PK_CorrectionsMemoryData.ErrorCode = 1;

      if(size + offset > sizeof(gEcuCorrections))
        PK_CorrectionsMemoryData.ErrorCode = 2;

      if(size > sizeof(gEcuCorrections) || size > PACKET_CONFIG_MAX_SIZE)
        PK_CorrectionsMemoryData.ErrorCode = 3;

      if(PK_CorrectionsMemoryData.ErrorCode == 0)
      {
        memcpy(&PK_CorrectionsMemoryData.data[0], &((uint8_t*)&gEcuCorrections)[offset], size);
        memset(&PK_CorrectionsMemoryData.data[size], 0, sizeof(PK_CorrectionsMemoryData.data) - size);
      }
      else
      {
        memset(&PK_CorrectionsMemoryData.data[0], 0, sizeof(PK_CorrectionsMemoryData.data));
      }
      PK_SendCommand(xChaSrc, &PK_CorrectionsMemoryData, sizeof(PK_CorrectionsMemoryData));
      break;

    case PK_CorrectionsMemoryDataID :
      PK_Copy(&PK_CorrectionsMemoryData, msgBuf);
      offset = PK_CorrectionsMemoryAcknowledge.offset = PK_CorrectionsMemoryData.offset;
      size = PK_CorrectionsMemoryAcknowledge.size = PK_CorrectionsMemoryData.size;
      configsize = PK_CorrectionsMemoryAcknowledge.configsize = PK_CorrectionsMemoryData.configsize;
      PK_CorrectionsMemoryAcknowledge.ErrorCode = 0;

      if(configsize != sizeof(gEcuCorrections))
        PK_CorrectionsMemoryData.ErrorCode = 1;

      if(size + offset > sizeof(gEcuCorrections))
        PK_CorrectionsMemoryData.ErrorCode = 2;

      if(size > sizeof(gEcuCorrections) || size > PACKET_CONFIG_MAX_SIZE)
        PK_CorrectionsMemoryData.ErrorCode = 3;

      if(PK_CorrectionsMemoryAcknowledge.ErrorCode == 0)
      {
        memcpy(&((uint8_t*)&gEcuCorrections)[offset], &PK_CorrectionsMemoryData.data[0], size);
      }

      PK_SendCommand(xChaSrc, &PK_CorrectionsMemoryAcknowledge, sizeof(PK_CorrectionsMemoryAcknowledge));
      break;

    case PK_SaveConfigID :
      if(!Mem.savereq)
      {
        Mem.savereqsrc = xChaSrc;
        Mem.savereq = 1;
        //CanDeinit = 0;
      }
      else if(Mem.loadreq)
      {
        PK_SaveConfigAcknowledge.ErrorCode = 3;
        PK_SendCommand(xChaSrc, &PK_SaveConfigAcknowledge, sizeof(PK_SaveConfigAcknowledge));
      }
      else
      {
        PK_SaveConfigAcknowledge.ErrorCode = 2;
        PK_SendCommand(xChaSrc, &PK_SaveConfigAcknowledge, sizeof(PK_SaveConfigAcknowledge));
      }
      break;


    case PK_RestoreConfigID :
      if(!Mem.loadreq)
      {
        Mem.loadreqsrc = xChaSrc;
        Mem.loadreq = 1;
      }
      else if(Mem.savereq)
      {
        PK_RestoreConfigAcknowledge.ErrorCode = 3;
        PK_SendCommand(xChaSrc, &PK_RestoreConfigAcknowledge, sizeof(PK_RestoreConfigAcknowledge));
      }
      else
      {
        PK_RestoreConfigAcknowledge.ErrorCode = 2;
        PK_SendCommand(xChaSrc, &PK_RestoreConfigAcknowledge, sizeof(PK_RestoreConfigAcknowledge));
      }
      break;

    case PK_DragStartID :
      PK_Copy(&PK_DragStart, msgBuf);
      Drag.FromSpeed = PK_DragStartAcknowledge.FromSpeed = PK_DragStart.FromSpeed;
      Drag.ToSpeed = PK_DragStartAcknowledge.ToSpeed = PK_DragStart.ToSpeed;
      Drag.Started = 0;
      Drag.Ready = 0;
      Drag.Completed = 0;
      Drag.PointsCount = 0;
      Drag.StartTime = 0;
      Drag.TimeLast = 0;
      if(fabsf(Drag.FromSpeed - Drag.ToSpeed) > 5.0f)
      {
        Drag.Status = PK_DragStartAcknowledge.ErrorCode = 0;
        Drag.Ready = 1;
      }
      else Drag.Status = PK_DragStartAcknowledge.ErrorCode = 1;
      PK_SendCommand(xChaSrc, &PK_DragStartAcknowledge, sizeof(PK_DragStartAcknowledge));
      break;

    case PK_DragStopID :
      PK_Copy(&PK_DragStop, msgBuf);
      PK_DragStopAcknowledge.FromSpeed = PK_DragStop.FromSpeed;
      PK_DragStopAcknowledge.ToSpeed = PK_DragStop.ToSpeed;
      Drag.Status = PK_DragStopAcknowledge.ErrorCode = 0;
      Drag.Started = 0;
      Drag.Ready = 0;
      Drag.Completed = 0;
      Drag.PointsCount = 0;
      Drag.StartTime = 0;
      if(Drag.FromSpeed != PK_DragStop.FromSpeed || Drag.ToSpeed != PK_DragStop.ToSpeed)
      {
        PK_DragStopAcknowledge.ErrorCode = 2;
      }
      PK_SendCommand(xChaSrc, &PK_DragStopAcknowledge, sizeof(PK_DragStopAcknowledge));
      break;

    case PK_DragUpdateRequestID :
      PK_Copy(&PK_DragUpdateRequest, msgBuf);
      PK_DragUpdateResponse.ErrorCode = 0;
      PK_DragUpdateResponse.FromSpeed = Drag.FromSpeed;
      PK_DragUpdateResponse.ToSpeed = Drag.ToSpeed;
      PK_DragUpdateResponse.Data.Pressure = gParameters.ManifoldAirPressure;
      PK_DragUpdateResponse.Data.RPM = gParameters.RPM;
      PK_DragUpdateResponse.Data.Mixture = gParameters.FuelRatio;
      PK_DragUpdateResponse.Data.Acceleration = gParameters.Acceleration;
      PK_DragUpdateResponse.Data.Ignition = gParameters.IgnitionAngle;
      PK_DragUpdateResponse.Data.CycleAirFlow = gParameters.CyclicAirFlow;
      PK_DragUpdateResponse.Data.MassAirFlow = gParameters.MassAirFlow;
      PK_DragUpdateResponse.Data.Throttle = gParameters.ThrottlePosition;
      PK_DragUpdateResponse.TotalPoints = Drag.PointsCount;
      PK_DragUpdateResponse.Started = Drag.Started;
      PK_DragUpdateResponse.Completed = Drag.Completed;
      PK_DragUpdateResponse.Data.Time = Drag.StartTime > 0 ? DelayDiff(Drag.StopTime, Drag.StartTime) : 0;
      if(Drag.Status > 0) PK_DragUpdateResponse.ErrorCode = Drag.Status + 10;
      PK_SendCommand(xChaSrc, &PK_DragUpdateResponse, sizeof(PK_DragUpdateResponse));
      break;

    case PK_DragPointRequestID :
      PK_Copy(&PK_DragPointRequest, msgBuf);
      PK_DragPointResponse.FromSpeed = PK_DragStop.FromSpeed;
      PK_DragPointResponse.ToSpeed = PK_DragStop.ToSpeed;
      PK_DragPointResponse.ErrorCode = 0;
      dragpoint = PK_DragPointResponse.PointNumber = PK_DragPointRequest.PointNumber;
      if(dragpoint >= Drag.PointsCount)
      {
        PK_DragPointResponse.Point.Pressure = 0;
        PK_DragPointResponse.Point.RPM = 0;
        PK_DragPointResponse.Point.Mixture = 0;
        PK_DragPointResponse.Point.Acceleration = 0;
        PK_DragPointResponse.Point.Ignition = 0;
        PK_DragPointResponse.Point.Time = 0;
        PK_DragPointResponse.ErrorCode = 3;
      }
      else
      {
        PK_DragPointResponse.Point = Drag.Points[dragpoint];
        if(Drag.FromSpeed != PK_DragPointRequest.FromSpeed || Drag.ToSpeed != PK_DragPointRequest.ToSpeed)
        {
          PK_DragPointResponse.ErrorCode = 2;
        }
      }

      PK_SendCommand(xChaSrc, &PK_DragPointResponse, sizeof(PK_DragPointResponse));
      break;

    case PK_ConfigMemoryAcknowledgeID :
      PK_Copy(&PK_ConfigMemoryAcknowledge, msgBuf);
      if(PK_ConfigMemoryAcknowledge.ErrorCode != 0)
      {

      }
      break;

    case PK_TableMemoryAcknowledgeID :
      PK_Copy(&PK_TableMemoryAcknowledge, msgBuf);
      if(PK_TableMemoryAcknowledge.ErrorCode != 0)
      {

      }
      break;

    case PK_CriticalMemoryAcknowledgeID :
      PK_Copy(&PK_CriticalMemoryAcknowledge, msgBuf);
      if(PK_TableMemoryAcknowledge.ErrorCode != 0)
      {

      }
      break;

    case PK_CorrectionsMemoryAcknowledgeID :
      PK_Copy(&PK_CorrectionsMemoryAcknowledge, msgBuf);
      if(PK_TableMemoryAcknowledge.ErrorCode != 0)
      {

      }
      break;

    case PK_FuelSwitchID :
      PK_Copy(&PK_FuelSwitch, msgBuf);
      if(PK_FuelSwitch.FuelSwitchPos < 3)
        ecu_set_table(PK_FuelSwitch.FuelSwitchPos);
      break;

    case PK_ParametersRequestID :
      //PK_Copy(&PK_ParametersRequest, msgBuf);
      PK_ParametersResponse.Parameters = gParameters;
      PK_SendCommand(xChaSrc, &PK_ParametersResponse, sizeof(PK_ParametersResponse));
      break;

    case PK_ForceParametersDataID :
      PK_Copy(&PK_ForceParametersData, msgBuf);
      gForceParameters = PK_ForceParametersData.Parameters;
      PK_ForceParametersDataAcknowledge.ErrorCode = 0;
      PK_SendCommand(xChaSrc, &PK_ForceParametersDataAcknowledge, sizeof(PK_ForceParametersDataAcknowledge));
      break;

    case PK_StatusRequestID :
      //PK_Copy(&PK_StatusRequest, msgBuf);
      memcpy(PK_StatusResponse.CheckBitmap, gCheckBitmap, CHECK_BITMAP_SIZE);
      memcpy(PK_StatusResponse.CheckBitmapRecorded, gEcuCriticalBackup.CheckBitmapRecorded, CHECK_BITMAP_SIZE);
      PK_SendCommand(xChaSrc, &PK_StatusResponse, sizeof(PK_StatusResponse));
      break;

    case PK_ResetStatusRequestID :
      //PK_Copy(&PK_StatusRequest, msgBuf);
      gStatusReset = 1;
      memset(&gStatus, 0, sizeof(gStatus));
      memset(&gCheckBitmap, 0, sizeof(gCheckBitmap));
      memset(&gEcuCriticalBackup.CheckBitmapRecorded, 0, sizeof(gEcuCriticalBackup.CheckBitmapRecorded));
      PK_ResetStatusResponse.ErrorCode = 0;
      PK_SendCommand(xChaSrc, &PK_ResetStatusResponse, sizeof(PK_ResetStatusResponse));
      break;

    default:
      break;
  }
}
