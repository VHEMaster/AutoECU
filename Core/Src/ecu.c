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
#include "can.h"
#include "kline.h"

#include <string.h>
#include <float.h>
#include "arm_math.h"
#include "math_fast.h"

typedef float (*math_interpolate_2d_set_func_t)(sMathInterpolateInput input_x, sMathInterpolateInput input_y,
    uint32_t y_size, float (*table)[], float new_value);
typedef float (*math_interpolate_2d_func_t)(sMathInterpolateInput input_x, sMathInterpolateInput input_y,
    uint32_t y_size, const float (*table)[]);

#define ENRICHMENT_STATES_COUNT 6

typedef volatile struct {
    uint8_t savereq;
    uint8_t loadreq;
    uint8_t issaving;
    uint8_t isloading;
    uint8_t lock;
    uint8_t critical_lock;

    eTransChannels savereqsrc;
    eTransChannels loadreqsrc;
}sMem;

typedef volatile struct {
    uint8_t Processing;
    uint8_t Cutting;
    uint8_t FuelCutoff;
}sCutoff;

typedef volatile struct {
    uint8_t Thresholded;
    uint8_t Shifting;
    uint8_t Tilled;

}sShift;

typedef volatile struct {
    uint8_t IgnitionEnabled;
    uint8_t InjectionEnabled;
    uint16_t Count;
    uint32_t Period;
    uint32_t IgnitionPulse;
    uint32_t InjectionPulse;
    uint32_t StartedTime;
    uint16_t CompletedCount;
    uint8_t InjectionTriggered;
} sIgnitionInjectionTest;

union {
    struct {
        uint8_t is_not_running : 1;
        uint8_t is_idle : 1;
        uint8_t is_enrichment : 1;
        uint8_t is_fuel_cutoff : 1;
        uint8_t is_use_lambda : 1;
        uint8_t is_knock_zone : 1;
        uint8_t is_adsorber : 1;
        uint8_t is_lambda_teach : 1;

        uint8_t not_used1 : 1;
        uint8_t is_idle_last : 1;
        uint8_t is_idle_lock_last : 1;
        uint8_t is_knock_zone_last : 1;
        uint8_t is_adsorber_last : 1;
        uint8_t is_knock : 1;
        uint8_t is_lambda_last : 1;
        uint8_t is_lambda_now : 1;
    } Bits;
    struct {
        uint8_t byte[2];
    } Bytes;
} gDiagWorkingMode;

union {
    struct {
        uint8_t csps_error : 1;
        uint8_t not_used1 : 1;
        uint8_t eeprom_error : 1;
        uint8_t lambda_heater : 1;
        uint8_t tsps_error : 1;
        uint8_t reset_error : 1;
        uint8_t ram_error : 1;
        uint8_t flash_error : 1;

        uint8_t low_voltage : 1;
        uint8_t not_used2 : 1;
        uint8_t not_used3 : 1;
        uint8_t engine_temp_low : 1;
        uint8_t lambda_low : 1;
        uint8_t tps_low : 1;
        uint8_t maf_low : 1;
        uint8_t low_noise : 1;

        uint8_t high_voltage : 1;
        uint8_t not_used4 : 1;
        uint8_t not_used5 : 1;
        uint8_t engine_temp_high : 1;
        uint8_t lambda_high : 1;
        uint8_t tps_high : 1;
        uint8_t maf_high : 1;
        uint8_t high_noise : 1;

        uint8_t knock_error : 1;
        uint8_t no_immo : 1;
        uint8_t not_used6 : 1;
        uint8_t lambda_no_activity : 1;
        uint8_t lambda_no_on_lean : 1;
        uint8_t lambda_no_on_rich : 1;
        uint8_t speed_error : 1;
        uint8_t idle_error : 1;

    } Bits;
    struct {
        uint8_t byte[4];
    } Bytes;
} gDiagErrors;
static GPIO_TypeDef * const gIgnPorts[ECU_CYLINDERS_COUNT] = { IGN_1_GPIO_Port, IGN_2_GPIO_Port, IGN_3_GPIO_Port, IGN_4_GPIO_Port };
static const uint16_t gIgnPins[ECU_CYLINDERS_COUNT] = { IGN_1_Pin, IGN_2_Pin, IGN_3_Pin, IGN_4_Pin };

static GPIO_TypeDef * const gInjPorts[ECU_CYLINDERS_COUNT] = { INJ_1_GPIO_Port, INJ_2_GPIO_Port, INJ_3_GPIO_Port, INJ_4_GPIO_Port };
static const uint16_t gInjPins[ECU_CYLINDERS_COUNT] = { INJ_1_Pin, INJ_2_Pin, INJ_3_Pin, INJ_4_Pin };

static GPIO_TypeDef * const gInjChPorts[2] = { INJ_CH1_GPIO_Port, INJ_CH2_GPIO_Port };
static const uint16_t gInjChPins[2] = { INJ_CH1_Pin, INJ_CH2_Pin};

static sEcuTable gEcuTable[TABLE_SETUPS_MAX];
static sEcuParams gEcuParams;
static sEcuCorrections gEcuCorrections;
static sEcuCorrectionsProgress gEcuCorrectionsProgress;
static sEcuCriticalBackup gEcuCriticalBackup;
static uint8_t volatile gStatusReset = 0;
static sStatus gStatus = {{{0}}};
static sParameters gParameters = {0};
static sForceParameters gForceParameters = {0};
static sIgnitionInjectionTest gIITest = {0};
static uint8_t gCheckBitmap[CHECK_BITMAP_SIZE] = {0};

static volatile uint8_t gEcuIdleValveCalibrate = 0;
static volatile uint8_t gEcuIdleValveCalibrateOk = 0;
static volatile uint8_t gEcuInitialized = 0;
static volatile uint8_t gEcuIsError = 0;
static uint8_t gCanTestStarted = 0;

static volatile int8_t gCriticalStatus = 0;
static volatile int8_t gBackupStatus = 0;

static sMathPid gPidIdleIgnition = {0};
static sMathPid gPidIdleAirFlow = {0};
static sMathPid gPidShortTermCorr = {0};

static const float gKnockDetectStartAngle = -100;
static const float gKnockDetectEndAngle = 70;

static volatile uint32_t gParametersRequestFillLast = 0;

static sDrag Drag = {0};
static sMem Mem = {0};
static sCutoff Cutoff = {0};
static sShift Shift = {0};

static volatile uint8_t gIgnCanShutdown = 0;
#ifndef SIMULATION
static volatile HAL_StatusTypeDef gIgnState = GPIO_PIN_SET;
static volatile uint8_t gIgnShutdownReady = 0;

static int8_t ecu_shutdown_process(void);
#endif

static int8_t ecu_can_process_message(const sCanMessage *message);

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
    gStatus.Bkpsram.Struct.CorrsLoad = HAL_ERROR;
    config_default_corrections(&gEcuCorrections);
    while(!(status = config_save_corrections(&gEcuCorrections))) {}
    if(status < 0) {
      gStatus.Bkpsram.Struct.CorrsSave = HAL_ERROR;
    }
  }

  while(!(status = config_load_critical_backup(&gEcuCriticalBackup))) {}
  if(status < 0) {
    gStatus.Bkpsram.Struct.CriticalLoad = HAL_ERROR;
    config_default_critical_backup(&gEcuCriticalBackup);
    while(!(status = config_save_critical_backup(&gEcuCriticalBackup))) {}
    if(status < 0) {
      gStatus.Bkpsram.Struct.CriticalSave = HAL_ERROR;
    }
  }
}

static float ecu_get_air_destiny(float pressure, float temperature)
{
  //Output in mg/cc
  const float M = 29.0f;
  const float R = 8314.46261815324f;
  float mg_cc3 = (pressure * M) / (R * (temperature + 273.15f));
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
  sCspsData csps = csps_data();
  float rpm = csps_getrpm(csps);
  sMathInterpolateInput ipRpm = math_interpolate_input(rpm, table->rotates, table->rotates_count);

  math_pid_set_clamp(&gPidIdleIgnition, table->idle_ign_deviation_min, table->idle_ign_deviation_max);
  math_pid_set_clamp(&gPidIdleAirFlow, -256, 256);
  math_pid_set_clamp(&gPidShortTermCorr, -0.25f, 0.25f);

  if(isidle) {
    math_pid_set_koffs(&gPidIdleIgnition, math_interpolate_1d(ipRpm, table->idle_ign_to_rpm_pid_p),
        math_interpolate_1d(ipRpm, table->idle_ign_to_rpm_pid_i), math_interpolate_1d(ipRpm, table->idle_ign_to_rpm_pid_d));
    math_pid_set_koffs(&gPidIdleAirFlow, math_interpolate_1d(ipRpm, table->idle_valve_to_massair_pid_p),
        math_interpolate_1d(ipRpm, table->idle_valve_to_massair_pid_i), math_interpolate_1d(ipRpm, table->idle_valve_to_massair_pid_d));
  } else {
    math_pid_set_koffs(&gPidIdleIgnition, math_interpolate_1d(ipRpm, table->idle_ign_to_rpm_pid_p), 0, 0);
    math_pid_set_koffs(&gPidIdleAirFlow, math_interpolate_1d(ipRpm, table->idle_valve_to_massair_pid_p), 0, 0);
  }
  math_pid_set_koffs(&gPidShortTermCorr, table->short_term_corr_pid_p, table->short_term_corr_pid_i, table->short_term_corr_pid_d);
}

static void ecu_pid_init(void)
{
  math_pid_init(&gPidIdleIgnition);
  math_pid_init(&gPidIdleAirFlow);
  math_pid_init(&gPidShortTermCorr);
  ecu_pid_update(0);
}

#ifdef SIMULATION
float gDebugMap = 75000;
float gDebugAirTemp = 20.0f;
float gDebugEngineTemp = 90.0f;
float gDebugThrottle = 60;
float gDebugReferenceVoltage = 5.1f;
float gDebugPowerVoltage = 14.4f;
#endif

static void ecu_update(void)
{
  static uint32_t adaptation_last = 0;
  static uint32_t phased_last = 0;
  static float fuel_consumed = 0;
  static float km_driven = 0;
  static uint32_t updated_last = 0;
  static float idle_angle_correction = 0;
  static float idle_valve_pos_correction = 0;
  uint32_t now = Delay_Tick;
  uint32_t hal_now = HAL_GetTick();
  float adapt_diff = DelayDiff(now, adaptation_last);
  float diff = DelayDiff(now, updated_last);
  uint32_t table_number = gParameters.CurrentTable;
  sEcuTable *table = &gEcuTable[table_number];
  uint8_t calibration = gEcuParams.performAdaptation;
  sMathInterpolateInput ipRpm = {0};
  sMathInterpolateInput ipMap = {0};
  sMathInterpolateInput ipEngineTemp = {0};
  sMathInterpolateInput ipAirTemp = {0};
  sMathInterpolateInput ipSpeed = {0};
  sMathInterpolateInput ipThr = {0};
  sMathInterpolateInput ipVoltages = {0};

  sMathInterpolateInput ipFilling = {0};

  sMathInterpolateInput ipEnrichmentMap = {0};
  sMathInterpolateInput ipEnrichmentThr = {0};

  float rpm;
  float pressure;
  float fuel_ratio;
  float lambda_value;
  float lambda_temperature;
  float lambda_heatervoltage;
  float lambda_temperaturevoltage;
  float air_temp;
  float engine_temp;
  float throttle;
  float power_voltage;
  float reference_voltage;
  float speed;
  float acceleration;
  float idle_valve_position;
  float uspa;
  float period;

  float fuel_ratio_diff;
  float wish_fuel_ratio;
  float filling;
  float throttle_by_pressure;
  float effective_volume;
  float ignition_angle;
  float ignition_time;
  float injector_lag;
  float cycle_air_flow;
  float mass_air_flow;
  float injection_time;
  float injection_phase;
  float injection_phase_duration;
  float knock_threshold;
  float air_destiny;
  float filling_relative;
  float fuel_flow_per_us;
  float knock_noise_level;
  float knock_zone;

  float min, max;
  static uint32_t prev_halfturns = 0;
  uint32_t halfturns;
  uint32_t halfturns_performed = 0;
  static float enrichment_map_states[ENRICHMENT_STATES_COUNT] = {0};
  static float enrichment_thr_states[ENRICHMENT_STATES_COUNT] = {0};
  static float enrichment_status_map = 0.0f;
  static float enrichment_status_thr = 0.0f;
  static float short_term_correction = 0.0f;
  float short_term_correction_pid = 0.0f;
  float enrichment_map_value;
  float enrichment_thr_value;
  float enrichment_by_map_hpf;
  float enrichment_by_thr_hpf;
  static float enrichment = 0.0f;
  static float enrichment_status = 0.0f;
  float enrichment_proportion;
  float fuel_amount_per_cycle;

  float warmup_mixture;
  float warmup_mix_koff;
  float warmup_mix_corr;
  float air_temp_mix_corr;
  float air_temp_ign_corr;

  float fuel_pressure;
  float fuel_abs_pressure;
  float fuel_consumption_per_distance;
  float fuel_consumption_hourly;
  float km_drive = 0;
  float fuel_consumption = 0;

  float idle_rpm_pid_act;
  float idle_reg_rpm;
  float idle_wish_rpm;
  float idle_wish_massair;
  float idle_wish_ignition;
  float idle_rpm_shift;
  float idle_table_valve_pos;
  float idle_wish_valve_pos;
  float injection_dutycycle;

  float calib_cur_progress;
  float percentage;
  float filling_diff;
  float map_diff_thr;
  float lpf_calculation;
  float fill_correction_map;
  float map_correction_thr;
  float idle_valve_pos_adaptation;
  float idle_valve_pos_dif;
  float ignition_correction;
  float ignition_corr_final;
  float wish_fault_rpm;
  float start_mixture;
  float tsps_rel_pos = 0;

  HAL_StatusTypeDef knock_status;
  uint8_t rotates;
  uint8_t running;
  uint8_t phased;
  uint8_t idle_flag;
  uint8_t idle_corr_flag;
  uint8_t o2_valid = 0;
  uint8_t use_tsps = gEcuParams.useTSPS;
  uint8_t shift_processing = Shift.Shifting;
  uint8_t cutoff_processing = Cutoff.Processing;
  sCspsData csps = csps_data();

  math_interpolate_2d_set_func_t corr_math_interpolate_2d_set_func = math_interpolate_2d_set;
  math_interpolate_2d_func_t corr_math_interpolate_2d_func = math_interpolate_2d;

  if(calibration == 2) {
    corr_math_interpolate_2d_func = math_interpolate_2d_point;
    corr_math_interpolate_2d_set_func = math_interpolate_2d_set_point;
  }

  gStatus.AdcStatus = sens_get_adc_status();

  halfturns = csps_gethalfturns(csps);
  running = csps_isrunning();
  rotates = csps_isrotates();
  phased = csps_isphased(csps);
  uspa = csps_getuspa(csps);
  period = csps_getperiod(csps);
  enrichment_proportion = table->enrichment_proportion_map_vs_thr;
  fuel_pressure = table->fuel_pressure;

  gStatus.Sensors.Struct.Csps = csps_iserror() == 0 ? HAL_OK : HAL_ERROR;
  rpm = csps_getrpm(csps);
  speed = speed_getspeed();
  acceleration = speed_getacceleration();
  knock_status = Knock_GetStatus();

#ifdef SIMULATION
  pressure = gDebugMap;
  air_temp = gDebugAirTemp;
  engine_temp = gDebugEngineTemp;
  throttle = gDebugThrottle;
  reference_voltage = gDebugReferenceVoltage;
  power_voltage = gDebugPowerVoltage;
#else
  gStatus.Sensors.Struct.PowerVoltage = sens_get_power_voltage(&power_voltage);
  gStatus.Sensors.Struct.ReferenceVoltage = sens_get_reference_voltage(&reference_voltage);
  gStatus.Sensors.Struct.ThrottlePos = sens_get_throttle_position(&throttle);
  gStatus.Sensors.Struct.Map = sens_get_map(&pressure);
  gStatus.Sensors.Struct.AirTemp = sens_get_air_temperature(&air_temp);
  gStatus.Sensors.Struct.EngineTemp = sens_get_engine_temperature(&engine_temp);
#endif

  tsps_rel_pos = csps_gettspsrelpos() - gEcuParams.tspsRelPos;

  if(gStatus.Sensors.Struct.EngineTemp != HAL_OK)
    engine_temp = 40.0f;
  if(gStatus.Sensors.Struct.AirTemp != HAL_OK)
    air_temp = 0.0f;

  fuel_ratio = table->fuel_afr;
  lambda_value = 1.0f;
  lambda_temperature = 0;
  lambda_temperaturevoltage = 0;
  lambda_heatervoltage = 0;
  if(gEcuParams.useLambdaSensor) {
    gStatus.Sensors.Struct.Lambda = sens_get_o2_labmda(&lambda_value, &o2_valid);
    if(gStatus.Sensors.Struct.Lambda == HAL_OK) {
      sens_get_o2_temperature(&lambda_temperature);
    }
    sens_get_o2_temperaturevoltage(&lambda_temperaturevoltage);
    sens_get_o2_heatervoltage(&lambda_heatervoltage);
    fuel_ratio = lambda_value * table->fuel_afr;
  } else {
    gStatus.Sensors.Struct.Lambda = HAL_OK;
  }

  if(running && use_tsps && !phased) {
    if(DelayDiff(now, phased_last) > 500000) {
      gStatus.Sensors.Struct.Tsps = HAL_ERROR;
      phased_last = now;
    }
  } else {
    phased_last = now;
    gStatus.Sensors.Struct.Tsps = HAL_OK;
  }

  if(running && use_tsps && phased && fabsf(tsps_rel_pos) >= gEcuParams.tspsDesyncThr) {
    gStatus.TspsSyncStatus = HAL_ERROR;
  } else {
    gStatus.TspsSyncStatus = HAL_OK;
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

  throttle_by_pressure = math_interpolate_2d(ipRpm, ipThr, TABLE_ROTATES_MAX, table->map_by_thr);
  map_correction_thr = corr_math_interpolate_2d_func(ipRpm, ipThr, TABLE_ROTATES_MAX, gEcuCorrections.map_by_thr);

  throttle_by_pressure *= map_correction_thr + 1.0f;

  if(gStatus.Sensors.Struct.ThrottlePos == HAL_OK && gStatus.Sensors.Struct.Map != HAL_OK) {
    pressure = throttle_by_pressure;
  }

  fuel_abs_pressure = fuel_pressure;
  fuel_flow_per_us = table->injector_performance * 1.66666667e-8f * table->fuel_mass_per_cc; // perf / 60.000.000
  if(table->is_fuel_pressure_const) {
    fuel_abs_pressure += (1.0f - pressure * 0.00001f);
    fuel_flow_per_us *= fast_rsqrt(fuel_pressure / fuel_abs_pressure);
  }

  ipEngineTemp = math_interpolate_input(engine_temp, table->engine_temps, table->engine_temp_count);
  ipAirTemp = math_interpolate_input(air_temp, table->air_temps, table->air_temp_count);
  ipSpeed = math_interpolate_input(speed, table->idle_rpm_shift_speeds, table->idle_speeds_shift_count);
  ipMap = math_interpolate_input(pressure, table->pressures, table->pressures_count);
  ipVoltages = math_interpolate_input(power_voltage, table->voltages, table->voltages_count);

  filling = math_interpolate_2d(ipRpm, ipMap, TABLE_ROTATES_MAX, table->fill_by_map);

  fill_correction_map = corr_math_interpolate_2d_func(ipRpm, ipMap, TABLE_ROTATES_MAX, gEcuCorrections.fill_by_map);

  filling *= fill_correction_map + 1.0f;
  filling_relative = filling;

  effective_volume = filling * gEcuParams.engineVolume;

  air_destiny = ecu_get_air_destiny(pressure, air_temp);

  cycle_air_flow = effective_volume * 0.25f * air_destiny;
  mass_air_flow = rpm * 0.03333333f * cycle_air_flow * 0.001f * 3.6f; // rpm / 60 * 2

  if(halfturns != prev_halfturns) {
    idle_valve_pos_correction = math_pid_update(&gPidIdleAirFlow, mass_air_flow, now);
    idle_angle_correction = math_pid_update(&gPidIdleIgnition, rpm, now);
  }

  while(halfturns != prev_halfturns) {
    halfturns_performed++;
    prev_halfturns++;
    for(int i = ENRICHMENT_STATES_COUNT - 2; i >= 0; i--)
    {
      enrichment_map_states[i] = enrichment_map_states[i + 1];
      enrichment_thr_states[i] = enrichment_thr_states[i + 1];
    }
    enrichment_map_states[ENRICHMENT_STATES_COUNT - 1] = pressure;
    enrichment_thr_states[ENRICHMENT_STATES_COUNT - 1] = throttle;

    if(running) {

      if(gStatus.Sensors.Struct.Map == HAL_OK) {
        max = enrichment_map_states[ENRICHMENT_STATES_COUNT - 1];
        min = enrichment_map_states[0];
        if(min > max) enrichment_map_value = 0.0f; else enrichment_map_value = max - min;
        ipEnrichmentMap = math_interpolate_input(enrichment_map_value, table->pressures, table->pressures_count);
        enrichment_map_value = math_interpolate_1d(ipEnrichmentMap, table->enrichment_by_map_sens);
        enrichment_by_map_hpf = math_interpolate_1d(ipRpm, table->enrichment_by_map_hpf);
        enrichment_status_map *= 1.0f - enrichment_by_map_hpf;
        if(enrichment_map_value > enrichment_status_map)
          enrichment_status_map = enrichment_map_value;
      } else {
        enrichment_status_map = 0.0f;
      }

      if(gStatus.Sensors.Struct.ThrottlePos == HAL_OK) {
        max = enrichment_thr_states[ENRICHMENT_STATES_COUNT - 1];
        min = enrichment_thr_states[0];
        if(min > max) enrichment_thr_value = 0.0f; else enrichment_thr_value = max - min;
        ipEnrichmentThr = math_interpolate_input(enrichment_thr_value, table->throttles, table->throttles_count);
        enrichment_thr_value = math_interpolate_1d(ipEnrichmentThr, table->enrichment_by_thr_sens);
        enrichment_by_thr_hpf = math_interpolate_1d(ipRpm, table->enrichment_by_thr_hpf);
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
  ignition_correction = corr_math_interpolate_2d_func(ipRpm, ipFilling, TABLE_ROTATES_MAX, gEcuCorrections.ignitions);
  air_temp_ign_corr = math_interpolate_2d(ipFilling, ipAirTemp, TABLE_FILLING_MAX, table->air_temp_ign_corr);

  idle_wish_rpm = math_interpolate_1d(ipEngineTemp, table->idle_wish_rotates);
  idle_wish_massair = math_interpolate_1d(ipEngineTemp, table->idle_wish_massair);
  idle_wish_ignition = math_interpolate_1d(ipRpm, table->idle_wish_ignition);

  idle_rpm_shift = math_interpolate_1d(ipSpeed, table->idle_rpm_shift);
  knock_noise_level = math_interpolate_1d(ipRpm, table->knock_noise_level);
  knock_threshold = math_interpolate_1d(ipRpm, table->knock_threshold);

  ignition_corr_final = ignition_correction;

  if(idle_flag && running) {
    ignition_angle = idle_wish_ignition;

    if(out_get_fan(NULL) != GPIO_PIN_RESET || gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.FanRelay == OutputDiagShortToGnd) {
      ignition_corr_final += table->idle_ign_fan_corr;

      if(ignition_corr_final > table->idle_ign_deviation_max)
        ignition_corr_final = table->idle_ign_deviation_max;
      else if(ignition_corr_final < table->idle_ign_deviation_min)
        ignition_corr_final = table->idle_ign_deviation_min;

    }
  }

  ignition_angle += air_temp_ign_corr;
  ignition_angle += ignition_corr_final;

  if(idle_flag && running) {
    if(gForceParameters.Enable.WishIdleIgnitionAngle) {
      math_pid_reset(&gPidIdleIgnition);
      ignition_angle = gForceParameters.WishIdleIgnitionAngle;
      idle_angle_correction = 0;
    }
  }

  wish_fuel_ratio = math_interpolate_2d(ipRpm, ipFilling, TABLE_ROTATES_MAX, table->fuel_mixtures);
  injection_phase = math_interpolate_2d(ipRpm, ipFilling, TABLE_ROTATES_MAX, table->injection_phase);
  air_temp_mix_corr = math_interpolate_2d(ipFilling, ipAirTemp, TABLE_FILLING_MAX, table->air_temp_mix_corr);

  start_mixture = math_interpolate_1d(ipEngineTemp, table->start_mixtures);

  if(gForceParameters.Enable.InjectionPhase)
    injection_phase = gForceParameters.InjectionPhase;
  warmup_mix_corr = math_interpolate_1d(ipEngineTemp, table->warmup_mix_corrs);
  warmup_mix_koff = math_interpolate_1d(ipEngineTemp, table->warmup_mix_koffs);
  if(warmup_mix_koff > 0.0f) {
    warmup_mixture = math_interpolate_1d(ipEngineTemp, table->warmup_mixtures);
    if(warmup_mixture < wish_fuel_ratio) {
      wish_fuel_ratio = warmup_mixture * warmup_mix_koff + wish_fuel_ratio * (1.0f - warmup_mix_koff);
    }
  }

  if(!running) {
    wish_fuel_ratio = start_mixture;
  }

  injector_lag = math_interpolate_1d(ipVoltages, table->injector_lag);

  ignition_time = math_interpolate_1d(ipVoltages, table->ignition_time);
  ignition_time *= math_interpolate_1d(ipRpm, table->ignition_time_rpm_mult);

  if(gForceParameters.Enable.IgnitionPulse)
    ignition_time = gForceParameters.IgnitionPulse;

  if(shift_processing && !cutoff_processing) {
    ignition_angle = gEcuParams.shiftAngle;
    wish_fuel_ratio = gEcuParams.shiftMixture;
    ignition_time *= 3.0f;
  }

  if(cutoff_processing) {
    ignition_angle = gEcuParams.cutoffAngle;
    wish_fuel_ratio = gEcuParams.cutoffMixture;
  }

  if(gForceParameters.Enable.WishFuelRatio)
    wish_fuel_ratio = gForceParameters.WishFuelRatio;

  math_pid_set_target(&gPidShortTermCorr, wish_fuel_ratio);

  if(!gEcuParams.useLambdaSensor || !o2_valid) {
    fuel_ratio = wish_fuel_ratio;
    lambda_value = wish_fuel_ratio / table->fuel_afr;
  }

  if(!running || !gEcuParams.useLambdaSensor || !o2_valid) {
    short_term_correction = 0.0f;
    math_pid_reset(&gPidShortTermCorr);
  } else {
    for(int i = 0; i < halfturns_performed; i++) {
      short_term_correction_pid = math_pid_update(&gPidShortTermCorr, fuel_ratio, 1000);
    }
    if(gEcuParams.useShortTermCorr && !calibration && !gForceParameters.Enable.InjectionPulse && !cutoff_processing && !shift_processing) {
      short_term_correction = -short_term_correction_pid;
    } else {
      math_pid_reset(&gPidShortTermCorr);
      short_term_correction = 0;
    }
  }

  fuel_amount_per_cycle = cycle_air_flow * 0.001f / wish_fuel_ratio;
  injection_time = fuel_amount_per_cycle / fuel_flow_per_us;
  injection_time *= warmup_mix_corr + 1.0f;
  injection_time *= air_temp_mix_corr + 1.0f;
  injection_time *= enrichment + 1.0f;
  if(idle_flag)
    injection_time *= gEcuCorrections.idle_correction + 1.0f;
  else
    injection_time *= gEcuCorrections.long_term_correction + 1.0f;
  injection_time *= short_term_correction + 1.0f;

  if(injection_time > 100.0f)
    injection_time += injector_lag * 1000.0f;
  else injection_time = 0;

  if(gForceParameters.Enable.InjectionPulse)
    injection_time = gForceParameters.InjectionPulse;

  if(Cutoff.FuelCutoff)
    injection_time = 0;

  if(gStatus.OilPressure.is_error && rpm >= gEcuParams.oilPressureCutoffRPM) {
    injection_time = 0;
  } else if(gStatus.OilPressure.is_error && gStatus.OilPressure.error_time > 10000) {
    injection_time = 0;
  }

  idle_wish_rpm += idle_rpm_shift;

  if(gForceParameters.Enable.WishIdleRPM)
    idle_wish_rpm = gForceParameters.WishIdleRPM;

  if(idle_wish_rpm < wish_fault_rpm) {
    idle_wish_rpm = wish_fault_rpm;
    idle_wish_massair *= wish_fault_rpm / idle_wish_rpm;
  }

  idle_rpm_pid_act = math_interpolate_1d(ipEngineTemp, table->idle_rpm_pid_act);
  idle_reg_rpm = idle_wish_rpm * (idle_rpm_pid_act + 1.0f);

  if(gForceParameters.Enable.WishIdleMassAirFlow)
    idle_wish_massair = gForceParameters.WishIdleMassAirFlow;

  if(!running) {
    idle_table_valve_pos = math_interpolate_1d(ipEngineTemp, table->idle_valve_initial);
    idle_valve_pos_adaptation = 0;
    idle_valve_pos_correction = 0;
  } else {
    idle_table_valve_pos = math_interpolate_2d(ipRpm, ipEngineTemp, TABLE_ROTATES_MAX, table->idle_valve_to_rpm);
    idle_valve_pos_adaptation = corr_math_interpolate_2d_func(ipRpm, ipEngineTemp, TABLE_ROTATES_MAX, gEcuCorrections.idle_valve_to_rpm);
  }

  idle_table_valve_pos *= idle_valve_pos_adaptation + 1.0f;
  idle_wish_valve_pos = idle_table_valve_pos;

  idle_corr_flag = idle_flag && rpm <= idle_reg_rpm;

  ecu_pid_update(idle_corr_flag);

  math_pid_set_target(&gPidIdleAirFlow, idle_wish_massair);
  math_pid_set_target(&gPidIdleIgnition, idle_wish_rpm);

  if(!idle_corr_flag) {
    idle_valve_pos_correction = 0;
    idle_angle_correction = 0;
  }

  idle_wish_valve_pos += idle_valve_pos_correction;
  ignition_angle += idle_angle_correction;

  if(gForceParameters.Enable.IgnitionAngle)
    ignition_angle = gForceParameters.IgnitionAngle;
  if(gForceParameters.Enable.IgnitionOctane)
    ignition_angle += gForceParameters.IgnitionOctane;

  if(gForceParameters.Enable.WishIdleValvePosition) {
    math_pid_reset(&gPidIdleAirFlow);
    idle_valve_pos_correction = 0;
    idle_wish_valve_pos = gForceParameters.WishIdleValvePosition;
  }

  injection_dutycycle = injection_time / (period * 2.0f);

  if(injection_dutycycle > 1.0f) {
    injection_time = period * 2.0f;
  }

  fuel_amount_per_cycle = injection_time * fuel_flow_per_us;

  if(idle_wish_valve_pos > 255.0f)
    idle_wish_valve_pos = 255.0f;
  else if(idle_wish_valve_pos < 0.0f)
    idle_wish_valve_pos = 0.0f;

  if(!gIgnCanShutdown)
    out_set_idle_valve(roundf(idle_wish_valve_pos));

  gStatus.Knock.Voltage = 0.0f;
  gStatus.Knock.Filtered = 0.0f;
  for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
    gStatus.Knock.Denoised[i]  = gStatus.Knock.Voltages[i] - knock_noise_level;
    gStatus.Knock.Detonates[i]  = gStatus.Knock.Denoised[i] - knock_threshold;
    if(gStatus.Knock.Denoised[i] < 0.0f)
      gStatus.Knock.Denoised[i] = 0.0f;
    if(gStatus.Knock.Detonates[i] < 0.0f)
      gStatus.Knock.Detonates[i] = 0.0f;
    if(gStatus.Knock.Voltage < gStatus.Knock.Voltages[i])
      gStatus.Knock.Voltage = gStatus.Knock.Voltages[i];
    if(gStatus.Knock.Filtered < gStatus.Knock.Denoised[i])
      gStatus.Knock.Filtered = gStatus.Knock.Denoised[i];
    if(gStatus.Knock.Detonate < gStatus.Knock.Detonates[i])
      gStatus.Knock.Detonate = gStatus.Knock.Detonates[i];
  }

  if(gStatus.Knock.StatusVoltage < gStatus.Knock.Voltage)
    gStatus.Knock.StatusVoltage = gStatus.Knock.Voltage;
  if(gStatus.Knock.StatusFiltered < gStatus.Knock.Filtered)
    gStatus.Knock.StatusFiltered = gStatus.Knock.Filtered;
  if(gStatus.Knock.StatusDetonate < gStatus.Knock.Detonate)
    gStatus.Knock.StatusDetonate = gStatus.Knock.Detonate;
  if(enrichment_status < enrichment)
    enrichment_status = enrichment;

  if(gParametersRequestFillLast == 0 || DelayDiff(now, gParametersRequestFillLast) > 500000) {
    gStatus.Knock.StatusVoltage = gStatus.Knock.Voltage;
    gStatus.Knock.StatusFiltered = gStatus.Knock.Filtered;
    gStatus.Knock.StatusDetonate = gStatus.Knock.Detonate;
    enrichment_status = enrichment;
    gParametersRequestFillLast = now;
  }

  if(gStatus.Sensors.Struct.Map != HAL_OK && gStatus.Sensors.Struct.ThrottlePos != HAL_OK) {
    running = 0;
    rotates = 0;
  }

  km_drive = speed * 2.77777778e-10f * diff; // speed / (60 * 60 * 1.000.000) * diff
  km_driven += km_drive;

  if(!rotates) {
    running = 0;
    injection_time = 0;
    cycle_air_flow = 0;
    mass_air_flow = 0;
    injection_dutycycle = 0;
    ignition_time = 0;
    effective_volume = 0;
    fuel_consumption_per_distance = 0;
    fuel_consumption_hourly = 0;
  } else {
    fuel_consumption = fuel_amount_per_cycle / table->fuel_mass_per_cc * (diff / period) * 0.001f * 2.0f;
    fuel_consumed += fuel_consumption;

    fuel_consumption_per_distance = fuel_consumption / km_drive * 100.0f;
    fuel_consumption_hourly = fuel_consumption * 3600000.0f;
  }

  if(!running) {
    ignition_angle = math_interpolate_1d(ipEngineTemp, table->ignition_initial);
    if(gStatus.Sensors.Struct.ThrottlePos == HAL_OK && throttle >= 80.0f)
      injection_time = 0;
  }

  if(uspa > 0.0f && injection_time > 0.0f) {
    injection_phase_duration = injection_time / uspa;
  } else {
    injection_phase_duration = 0;
  }

  if(injection_dutycycle > 0.90f) {
    gStatus.InjectionUnderflow = HAL_ERROR;
  } else {
    gStatus.InjectionUnderflow = HAL_OK;
  }

  if(gEcuParams.useKnockSensor && gStatus.Sensors.Struct.Knock == HAL_OK &&
      !gForceParameters.Enable.IgnitionAngle && !gForceParameters.Enable.InjectionPulse) {
    if(csps_isrunning()) {
      knock_zone = math_interpolate_2d_clamp(ipRpm, ipFilling, TABLE_ROTATES_MAX, table->knock_zone, 0.0f, 1.0f);

      for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
        if(gStatus.Knock.Updated[i]) {
          if(gStatus.Knock.Detonates[i] > 0.0f) {
            gStatus.Knock.GeneralStatus |= KnockStatusDedonation;
            gStatus.Knock.DetonationLast = hal_now;
            gStatus.Knock.Advances[i] += gStatus.Knock.Detonates[i];
            if(gStatus.Knock.Advances[i] > 1.0f)
              gStatus.Knock.Advances[i] = 1.0f;
          } else {
            if(gStatus.Knock.Advances[i] > 0.0f) {
              gStatus.Knock.Advances[i] -= gStatus.Knock.Period[i] * 0.000001f * 0.1f * knock_zone * knock_zone; //10 seconds to restore if knock_zone is 1.0
              if(gStatus.Knock.Advances[i] < 0.0f)
                gStatus.Knock.Advances[i] = 0.0f;
            }
            if(gStatus.Knock.Voltages[i] < knock_noise_level * 0.5f) {
              gStatus.Knock.GeneralStatus |= KnockStatusLowNoise;
              gStatus.Knock.LowNoiseLast = hal_now;
            }
          }
          gStatus.Knock.Updated[i] = 0;
        }
      }
    } else {
      knock_zone = 0.0f;
      gStatus.Knock.Advance = 0.0f;
      gStatus.Knock.GeneralStatus = KnockStatusOk;
    }
  } else {
    knock_zone = 0.0f;
    gStatus.Knock.Advance = 0.0f;
    gStatus.Knock.GeneralStatus = KnockStatusOk;
  }

  gStatus.Knock.Advance = 0;
  for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
    if(gStatus.Knock.Advances[i] > gStatus.Knock.Advance)
      gStatus.Knock.Advance = gStatus.Knock.Advances[i];
    if(!calibration) {
      gEcuCorrections.knock_ignition_correctives[i] = table->knock_ign_corr_max * knock_zone * gStatus.Knock.Advances[i];
      gEcuCorrections.knock_injection_correctives[i] = table->knock_inj_corr_max * knock_zone * gStatus.Knock.Advances[i];
    } else {
      gEcuCorrections.knock_ignition_correctives[i] = 0.0f;
      gEcuCorrections.knock_injection_correctives[i] = 0.0f;
    }
  }

  if(gStatus.Knock.DetonationLast && HAL_DelayDiff(hal_now, gStatus.Knock.DetonationLast) > 10000) {
    gStatus.Knock.GeneralStatus &= ~KnockStatusDedonation;
    gStatus.Knock.DetonationLast = 0;
  }
  if(gStatus.Knock.LowNoiseLast && HAL_DelayDiff(hal_now, gStatus.Knock.LowNoiseLast) > 10000) {
    gStatus.Knock.GeneralStatus &= ~KnockStatusLowNoise;
    gStatus.Knock.LowNoiseLast = 0;
  }

  fuel_ratio_diff = fuel_ratio / wish_fuel_ratio;

  if(adapt_diff >= period * 2.0f) {
    adaptation_last = now;
    if(running) {
      if(calibration && corr_math_interpolate_2d_set_func) {
        lpf_calculation = adapt_diff * 0.000001f;
        if(lpf_calculation > 0.1f)
          lpf_calculation = 0.1f;

        if(gEcuParams.useLambdaSensor && gStatus.Sensors.Struct.Lambda == HAL_OK &&
            o2_valid && !gForceParameters.Enable.InjectionPulse) {
          gEcuCorrections.long_term_correction = 0.0f;
          gEcuCorrections.idle_correction = 0.0f;
          short_term_correction = 0.0f;
          short_term_correction_pid = 0.0f;

          filling_diff = (fuel_ratio_diff) - 1.0f;
          if(gStatus.Sensors.Struct.Map == HAL_OK) {
            fill_correction_map += filling_diff * lpf_calculation;
            corr_math_interpolate_2d_set_func(ipRpm, ipMap, TABLE_ROTATES_MAX, gEcuCorrections.fill_by_map, fill_correction_map);

            percentage = (filling_diff + 1.0f);
            if(percentage > 1.0f) percentage = 1.0f / percentage;
            calib_cur_progress = corr_math_interpolate_2d_func(ipRpm, ipMap, TABLE_ROTATES_MAX, gEcuCorrectionsProgress.progress_fill_by_map);
            calib_cur_progress = (percentage * lpf_calculation) + (calib_cur_progress * (1.0f - lpf_calculation));
            corr_math_interpolate_2d_set_func(ipRpm, ipMap, TABLE_ROTATES_MAX, gEcuCorrectionsProgress.progress_fill_by_map, calib_cur_progress);
          }
        }

        if(gStatus.Sensors.Struct.ThrottlePos == HAL_OK && gStatus.Sensors.Struct.Map == HAL_OK) {
          map_diff_thr = (pressure / throttle_by_pressure) - 1.0f;
          map_correction_thr += map_diff_thr * lpf_calculation;
          corr_math_interpolate_2d_set_func(ipRpm, ipThr, TABLE_ROTATES_MAX, gEcuCorrections.map_by_thr, map_correction_thr);

          percentage = (map_diff_thr + 1.0f);
          if(percentage > 1.0f) percentage = 1.0f / percentage;
          calib_cur_progress = corr_math_interpolate_2d_func(ipRpm, ipThr, TABLE_ROTATES_MAX, gEcuCorrectionsProgress.progress_map_by_thr);
          calib_cur_progress = (percentage * lpf_calculation) + (calib_cur_progress * (1.0f - lpf_calculation));
          corr_math_interpolate_2d_set_func(ipRpm, ipThr, TABLE_ROTATES_MAX, gEcuCorrectionsProgress.progress_map_by_thr, calib_cur_progress);
        }

        if(idle_flag && gStatus.Sensors.Struct.Map == HAL_OK && gStatus.Sensors.Struct.ThrottlePos == HAL_OK && !gForceParameters.Enable.WishIdleValvePosition) {
          idle_valve_pos_dif = (idle_wish_valve_pos / idle_table_valve_pos) - 1.0f;
          idle_valve_pos_adaptation += idle_valve_pos_dif * lpf_calculation;

          corr_math_interpolate_2d_set_func(ipRpm, ipEngineTemp, TABLE_ROTATES_MAX, gEcuCorrections.idle_valve_to_rpm, idle_valve_pos_adaptation);

          percentage = (idle_valve_pos_dif + 1.0f);
          if(percentage > 1.0f) percentage = 1.0f / percentage;
          calib_cur_progress = corr_math_interpolate_2d_func(ipRpm, ipEngineTemp, TABLE_ROTATES_MAX, gEcuCorrectionsProgress.progress_idle_valve_to_rpm);
          calib_cur_progress = (percentage * lpf_calculation) + (calib_cur_progress * (1.0f - lpf_calculation));
          corr_math_interpolate_2d_set_func(ipRpm, ipEngineTemp, TABLE_ROTATES_MAX, gEcuCorrectionsProgress.progress_idle_valve_to_rpm, calib_cur_progress);
        }

        if(gEcuParams.useKnockSensor && gStatus.Sensors.Struct.Knock == HAL_OK) {
          calib_cur_progress = corr_math_interpolate_2d_func(ipRpm, ipFilling, TABLE_ROTATES_MAX, gEcuCorrectionsProgress.progress_ignitions);

          lpf_calculation *= 0.2f; //5 sec for LPF of ignition

          if(gStatus.Knock.Detonate > 0.0f) {
            calib_cur_progress = 0.0f;

            ignition_correction = table->knock_ign_corr_max * lpf_calculation + ignition_correction * (1.0f - lpf_calculation);
            corr_math_interpolate_2d_set_func(ipRpm, ipFilling, TABLE_ROTATES_MAX, gEcuCorrections.ignitions, ignition_correction);
          } else {
            calib_cur_progress = (1.0f * lpf_calculation) + (calib_cur_progress * (1.0f - lpf_calculation));
          }
          corr_math_interpolate_2d_set_func(ipRpm, ipFilling, TABLE_ROTATES_MAX, gEcuCorrectionsProgress.progress_ignitions, calib_cur_progress);
        }
      } else {
        if(gEcuParams.useLambdaSensor && gStatus.Sensors.Struct.Lambda == HAL_OK && !gForceParameters.Enable.InjectionPulse && o2_valid) {
          lpf_calculation = adapt_diff * 0.000001f * 0.016666667f;
          filling_diff = (fuel_ratio_diff) - 1.0f;
          if(gEcuParams.useLongTermCorr) {
            if(!idle_flag && throttle > 10.0f) {
              gEcuCorrections.long_term_correction += (filling_diff + short_term_correction) * lpf_calculation;
            }
            else if(idle_flag || throttle < 5.0f) {
              gEcuCorrections.idle_correction += (filling_diff + short_term_correction) * lpf_calculation;
            }
          } else {
            gEcuCorrections.long_term_correction = 0;
            gEcuCorrections.idle_correction = 0;
          }
        }
      }
    }
  }

  if(gEcuCorrections.long_term_correction > 0.25f)
    gEcuCorrections.long_term_correction = 0.25f;
  if(gEcuCorrections.long_term_correction < -0.25f)
    gEcuCorrections.long_term_correction = -0.25f;

  if(gEcuCorrections.idle_correction > 0.25f)
    gEcuCorrections.idle_correction = 0.25f;
  if(gEcuCorrections.idle_correction < -0.25f)
    gEcuCorrections.idle_correction = -0.25f;

  idle_valve_position = out_get_idle_valve();

  gEcuCriticalBackup.idle_valve_position = idle_valve_position;

  if(fuel_consumed > 0.1f) {
    gEcuCriticalBackup.fuel_consumed += fuel_consumed;
    fuel_consumed = 0;
  }

  if(km_driven > 0.1f) {
    gEcuCriticalBackup.km_driven += km_driven;
    km_driven = 0;
  }

  if((!idle_flag || (rpm > idle_reg_rpm && engine_temp > 55.0f) || (rpm < idle_reg_rpm && engine_temp > 80.0f)) &&
      running && gStatus.Sensors.Struct.Map == HAL_OK && gStatus.Sensors.Struct.ThrottlePos == HAL_OK) {
    float map_tps_relation = pressure / throttle_by_pressure;
    if(map_tps_relation > 1.10f || map_tps_relation < 0.80f) {
      if(gStatus.MapTpsRelation.is_error) {
        gStatus.MapTpsRelation.error_time += HAL_DelayDiff(hal_now, gStatus.MapTpsRelation.error_last);
      } else {
        gStatus.MapTpsRelation.error_time = 0;
      }
      gStatus.MapTpsRelation.is_error = 1;
    } else {
      gStatus.MapTpsRelation.is_error = 0;
    }
  } else {
    gStatus.MapTpsRelation.error_time = 0;
  }
  gStatus.MapTpsRelation.error_last = hal_now;

  min = 0.93f;
  max = 1.07f;

  if(gEcuParams.useKnockSensor && gStatus.Sensors.Struct.Knock == HAL_OK) {
    max *= (table->knock_inj_corr_max * knock_zone) + 1.0f;
  }

  if(gStatus.Sensors.Struct.Lambda == HAL_OK && o2_valid && running) {
    if(idle_flag) {
      if(fuel_ratio_diff < min) {
        if(gStatus.RichIdleMixture.is_error) {
          gStatus.RichIdleMixture.error_time += HAL_DelayDiff(hal_now, gStatus.RichIdleMixture.error_last);
        } else {
          gStatus.RichIdleMixture.error_time = 0;
        }
        gStatus.RichIdleMixture.is_error = 1;
      } else {
        gStatus.RichIdleMixture.is_error = 0;
      }
      if(fuel_ratio_diff > max) {
        if(gStatus.LeanIdleMixture.is_error) {
          gStatus.LeanIdleMixture.error_time += HAL_DelayDiff(hal_now, gStatus.LeanIdleMixture.error_last);
        } else {
          gStatus.LeanIdleMixture.error_time = 0;
        }
        gStatus.LeanIdleMixture.is_error = 1;
      } else {
        gStatus.LeanIdleMixture.is_error = 0;
      }
    } else {
      if(fuel_ratio_diff < min) {
        if(gStatus.RichMixture.is_error) {
          gStatus.RichMixture.error_time += HAL_DelayDiff(hal_now, gStatus.RichMixture.error_last);
        } else {
          gStatus.RichMixture.error_time = 0;
        }
        gStatus.RichMixture.is_error = 1;
      } else {
        gStatus.RichMixture.is_error = 0;
      }
      if(fuel_ratio_diff > max) {
        if(gStatus.LeanMixture.is_error) {
          gStatus.LeanMixture.error_time += HAL_DelayDiff(hal_now, gStatus.LeanMixture.error_last);
        } else {
          gStatus.LeanMixture.error_time = 0;
        }
        gStatus.LeanMixture.is_error = 1;
      } else {
        gStatus.LeanMixture.is_error = 0;
      }
    }
  } else {
    gStatus.RichMixture.error_time = 0;
    gStatus.LeanMixture.error_time = 0;
    gStatus.RichIdleMixture.error_time = 0;
    gStatus.LeanIdleMixture.error_time = 0;
  }
  gStatus.RichMixture.error_last = hal_now;
  gStatus.LeanMixture.error_last = hal_now;
  gStatus.RichIdleMixture.error_last = hal_now;
  gStatus.LeanIdleMixture.error_last = hal_now;

  gParameters.AdcKnockVoltage = gStatus.Knock.Voltage;
  gParameters.AdcAirTemp = adc_get_voltage(AdcChAirTemperature);
  gParameters.AdcEngineTemp = adc_get_voltage(AdcChEngineTemperature);
  gParameters.AdcManifoldAirPressure = adc_get_voltage(AdcChManifoldAbsolutePressure);
  gParameters.AdcThrottlePosition = adc_get_voltage(AdcChThrottlePosition);
  gParameters.AdcPowerVoltage = adc_get_voltage(AdcChPowerVoltage);
  gParameters.AdcReferenceVoltage = adc_get_voltage(AdcMcuChReferenceVoltage);
  gParameters.AdcLambdaUR = adc_get_voltage(AdcChO2UR);
  gParameters.AdcLambdaUA = adc_get_voltage(AdcChO2UA);

  gParameters.KnockSensor = gStatus.Knock.StatusVoltage;
  gParameters.KnockSensorFiltered = gStatus.Knock.StatusFiltered;
  gParameters.KnockSensorDetonate = gStatus.Knock.StatusDetonate;
  gParameters.KnockZone = knock_zone;
  gParameters.KnockAdvance = gStatus.Knock.Advance;
  gParameters.AirTemp = air_temp;
  gParameters.EngineTemp = engine_temp;
  gParameters.ManifoldAirPressure = pressure;
  gParameters.ThrottlePosition = throttle;
  gParameters.ReferenceVoltage = reference_voltage;
  gParameters.PowerVoltage = power_voltage;
  gParameters.FuelRatio = fuel_ratio;
  gParameters.FuelRatioDiff = fuel_ratio_diff;
  gParameters.LambdaValue = lambda_value;
  gParameters.LambdaTemperature = lambda_temperature;
  gParameters.LambdaTemperatureVoltage = lambda_temperaturevoltage;
  gParameters.LambdaHeaterVoltage = lambda_heatervoltage;
  gParameters.ShortTermCorrection = short_term_correction;
  gParameters.LongTermCorrection = gEcuCorrections.long_term_correction;
  gParameters.IdleCorrection = gEcuCorrections.idle_correction;

  gParameters.IdleFlag = idle_flag;
  gParameters.IdleCorrFlag = idle_corr_flag;
  gParameters.RPM = rpm;
  gParameters.Acceleration = acceleration;
  gParameters.Speed = speed;
  gParameters.MassAirFlow = mass_air_flow;
  gParameters.CyclicAirFlow = cycle_air_flow;
  gParameters.EffectiveVolume = effective_volume;
  gParameters.AirDestiny = air_destiny;
  gParameters.RelativeFilling = filling_relative;
  gParameters.WishFuelRatio = wish_fuel_ratio;
  gParameters.IdleValvePosition = idle_valve_position;
  gParameters.IdleRegThrRPM = idle_reg_rpm;
  gParameters.WishIdleRPM = idle_wish_rpm;
  gParameters.WishIdleMassAirFlow = idle_wish_massair;
  gParameters.WishIdleValvePosition = idle_wish_valve_pos;
  gParameters.WishIdleIgnitionAngle = idle_wish_ignition;
  gParameters.IgnitionAngle = ignition_angle;
  gParameters.InjectionPhase = injection_phase;
  gParameters.InjectionPhaseDuration = injection_phase_duration;
  gParameters.InjectionPulse = injection_time;
  gParameters.InjectionDutyCycle = injection_dutycycle;
  gParameters.InjectionEnrichment = enrichment;
  gParameters.InjectionLag = injector_lag;
  gParameters.IgnitionPulse = ignition_time;
  gParameters.IdleSpeedShift = idle_rpm_shift;

  gParameters.DrivenKilometers = gEcuCriticalBackup.km_driven + km_driven;
  gParameters.FuelConsumed = gEcuCriticalBackup.fuel_consumed + fuel_consumed;
  gParameters.FuelConsumption = fuel_consumption_per_distance;
  gParameters.FuelHourly = fuel_consumption_hourly;
  gParameters.TspsRelativePosition = tsps_rel_pos;

  gParameters.LambdaValid = o2_valid > 0;

  gParameters.OilSensor = sens_get_oil_pressure(NULL) != GPIO_PIN_RESET;
  gParameters.StarterSensor = sens_get_starter(NULL) != GPIO_PIN_RESET;
  gParameters.HandbrakeSensor = sens_get_handbrake(NULL) != GPIO_PIN_RESET;
  gParameters.ChargeSensor = sens_get_charge(NULL);
  gParameters.ClutchSensor = sens_get_clutch(NULL) != GPIO_PIN_RESET;
  gParameters.IgnSensor = sens_get_ign(NULL) != GPIO_PIN_RESET;

  gParameters.FuelPumpRelay = out_get_fuelpump(NULL) != GPIO_PIN_RESET;
  gParameters.FanRelay = out_get_fan(NULL) != GPIO_PIN_RESET;
  gParameters.CheckEngine = out_get_checkengine(NULL) != GPIO_PIN_RESET;
  gParameters.StarterRelay = out_get_starter(NULL) != GPIO_PIN_RESET;
  gParameters.Rsvd1Output = out_get_rsvd1(NULL) != GPIO_PIN_RESET;
  gParameters.IgnOutput = out_get_ign(NULL) != GPIO_PIN_RESET;

  gParameters.IsRunning = running;
  gParameters.IsCheckEngine = gEcuIsError;
  gParameters.InjectorChannel = table->inj_channel;

  strcpy(gParameters.CurrentTableName, table->name);

  gDiagWorkingMode.Bits.is_enrichment = enrichment > 0.02f;
  gDiagWorkingMode.Bits.is_idle = idle_flag && running;
  gDiagWorkingMode.Bits.is_idle_last = idle_flag && running;
  gDiagWorkingMode.Bits.is_idle_lock_last = 0;
  gDiagWorkingMode.Bits.is_knock = (gStatus.Knock.GeneralStatus & KnockStatusDedonation) > 0;
  gDiagWorkingMode.Bits.is_knock_zone = !idle_flag && running;
  gDiagWorkingMode.Bits.is_knock_zone_last = !idle_flag && running;

  gDiagWorkingMode.Bits.is_lambda_last = o2_valid;
  gDiagWorkingMode.Bits.is_lambda_now = o2_valid;
  gDiagWorkingMode.Bits.is_lambda_teach = o2_valid && calibration;
  gDiagWorkingMode.Bits.is_not_running = !running;
  gDiagWorkingMode.Bits.is_use_lambda = o2_valid;

  updated_last = now;
}

static void ecu_init_post_init(void)
{
  Misc_EnableIdleValvePosition(gEcuCriticalBackup.idle_valve_position);
  if(gStatus.Bkpsram.Struct.CriticalLoad != HAL_OK) {
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
  static uint32_t save_correction_last = 0;
  int8_t status;
  uint32_t now = Delay_Tick;

  if(!CRC16_IsBusy()) {
    if(!Mem.critical_lock) {
      Mem.critical_lock = 1;
      status = config_save_critical_backup(&gEcuCriticalBackup);
      Mem.critical_lock = 0;
      gCriticalStatus = status < 0 ? status : 0;
    }
  }

  switch(state) {
    case 0:
      if(DelayDiff(now, save_correction_last) > 500000) {
        save_correction_last = now;
        state++;
      }
      break;
    case 1:
      if(!CRC16_IsBusy()) {
        status = config_save_corrections(&gEcuCorrections);
        if(status) {
          state = 0;
        }
        gBackupStatus = status < 0 ? status : 0;
      }
      break;

    default:
      state = 0;
      break;
  }

  if(gBackupStatus < 0) {
      gStatus.Bkpsram.Struct.CorrsSave = HAL_ERROR;
  }
  if(gCriticalStatus < 0) {
      gStatus.Bkpsram.Struct.CriticalSave = HAL_ERROR;
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
  int32_t mode = gEcuParams.cutoffMode;
  float cutoffrpm = gEcuParams.cutoffRPM;

  //Cutoff always enabled
  if(1)
  {
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

    if(cutoff_processing_prev)
    {
      cutoff_processing_prev--;
      Cutoff.Processing = 1;
    }
    else Cutoff.Processing = 0;

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
  gDiagWorkingMode.Bits.is_fuel_cutoff = 0;
  if(rpm >= cutoffrpm + 500 ||
      (gStatus.OilPressure.is_error && rpm >= gEcuParams.oilPressureCutoffRPM) ||
      (gStatus.OilPressure.is_error && gStatus.OilPressure.error_time > 10000)) {
    gDiagWorkingMode.Bits.is_fuel_cutoff = 1;
    Cutoff.FuelCutoff = 1;
    return 0;
  }

  Cutoff.FuelCutoff = 0;
  return 1;
}

STATIC_INLINE uint8_t ecu_shift_process(uint8_t cy_count, uint8_t cylinder, uint8_t mode, GPIO_PinState clutch, uint8_t reset)
{
  static int8_t shift_cnt1 = -1;
  static uint32_t shift_time_last = 0;
  uint32_t tick_count = 0;
  uint32_t now = Delay_Tick;
  uint8_t retval = 1;

  if(mode > 0 && !reset) {
    if(mode == 1) {
      if(clutch == GPIO_PIN_SET) {
        shift_cnt1 = 5;
        retval = 0;
      } else {
        if(shift_cnt1 >= 0) {
          shift_cnt1--;
          retval = 1;
        } else {
          retval = 2;
        }
      }
    } else if(mode > 1) {
      switch(mode) {
        case 2 : tick_count = 20000; break;
        case 3 : tick_count = 50000; break;
        case 4 : tick_count = 100000; break;
        default: break;
      }
      if(clutch == GPIO_PIN_SET) {
        if(DelayDiff(now, shift_time_last) < tick_count) {
          retval = 0;
        } else {
          retval = 1;
        }
      } else {
        retval = 2;
      }
    }
  } else {
    shift_cnt1 = -1;
    retval = 2;
  }

  if(retval > 0) {
    if(tick_count > 0) {
      shift_time_last += tick_count;
    } else {
      shift_time_last = now;
    }
  }

  return retval;
}

STATIC_INLINE uint8_t ecu_shift_ign_act(uint8_t cy_count, uint8_t cylinder, GPIO_PinState clutch, float rpm, float throttle)
{
  uint8_t mode = gEcuParams.shiftMode;
  float thrthr = gEcuParams.shiftThrThr;
  float rpmthr = gEcuParams.shiftRpmThr;
  float rpmtill = gEcuParams.shiftRpmTill;
  uint8_t shift_result = 1;

  if(mode > 0 && throttle >= thrthr) {
    if(clutch == GPIO_PIN_RESET) {
      if(!Shift.Shifting) {
        Shift.Tilled = 0;
        Shift.Shifting = 0;
        if(rpm >= rpmthr && throttle >= thrthr) {
          Shift.Thresholded = 1;
        }
      }
    } else {
      if(Shift.Thresholded) {
        if(rpm >= rpmtill && !Shift.Tilled) {
          Shift.Shifting = 1;
        } else {
          Shift.Tilled = 1;
          Shift.Shifting = 0;
          Shift.Thresholded = 0;
        }
      }
    }
  } else {
    Shift.Shifting = 0;
    Shift.Thresholded = 0;
    Shift.Tilled = 0;
  }

  if(Shift.Thresholded && Shift.Shifting && !Shift.Tilled) {
    shift_result = ecu_shift_process(cy_count, cylinder, mode, clutch, 0);
    if(shift_result > 1) {
      Shift.Shifting = 0;
      Shift.Thresholded = 0;
      Shift.Tilled = 0;
    }
  } else {
    shift_result = ecu_shift_process(cy_count, cylinder, mode, clutch, 1);
  }

  return shift_result > 0;
}

STATIC_INLINE uint8_t ecu_shift_inj_act(uint8_t cy_count, uint8_t cylinder, GPIO_PinState clutch, float rpm, float throttle)
{
  //TODO: currently not affecting injection directly
  return 1;
}

STATIC_INLINE void ecu_coil_saturate(uint8_t cy_count, uint8_t cylinder)
{
  if(cy_count == ECU_CYLINDERS_COUNT && cylinder < ECU_CYLINDERS_COUNT) {
    gIgnPorts[cylinder]->BSRR = gIgnPins[cylinder];
  } else if(cy_count == ECU_CYLINDERS_COUNT_HALF && cylinder < ECU_CYLINDERS_COUNT_HALF) {
    gIgnPorts[cylinder]->BSRR = gIgnPins[cylinder];
    gIgnPorts[ECU_CYLINDERS_COUNT - 1 - cylinder]->BSRR = gIgnPins[ECU_CYLINDERS_COUNT - 1 - cylinder];
  } else if(cy_count == 1) {
    for(int i = 0; i < ECU_CYLINDERS_COUNT; i++)
      gIgnPorts[i]->BSRR = gIgnPins[i];
  }
}

STATIC_INLINE void ecu_coil_ignite(uint8_t cy_count, uint8_t cylinder)
{
  if(cy_count == ECU_CYLINDERS_COUNT && cylinder < ECU_CYLINDERS_COUNT) {
    gIgnPorts[cylinder]->BSRR = gIgnPins[cylinder] << 16;
  } else if(cy_count == ECU_CYLINDERS_COUNT_HALF && cylinder < ECU_CYLINDERS_COUNT_HALF) {
    gIgnPorts[cylinder]->BSRR = gIgnPins[cylinder] << 16;
    gIgnPorts[ECU_CYLINDERS_COUNT - 1 - cylinder]->BSRR = gIgnPins[ECU_CYLINDERS_COUNT - 1 - cylinder] << 16;
  } else if(cy_count == 1) {
    for(int i = 0; i < ECU_CYLINDERS_COUNT; i++)
      gIgnPorts[i]->BSRR = gIgnPins[i] << 16;
  }
}

STATIC_INLINE void ecu_inject(uint8_t cy_count, uint8_t cylinder, uint32_t time)
{
  if(cy_count == ECU_CYLINDERS_COUNT && cylinder < ECU_CYLINDERS_COUNT) {
    injector_enable(cylinder, time);
  } else if(cy_count == ECU_CYLINDERS_COUNT_HALF) {
    injector_enable(cylinder, time);
    injector_enable(ECU_CYLINDERS_COUNT - 1 - cylinder, time);
  } else if(cy_count == 1) {
    for(int i = 0; i < ECU_CYLINDERS_COUNT; i++)
      injector_enable(i, time);
  }
}

static void ecu_process(void)
{
  sEcuTable *table = &gEcuTable[ecu_get_table()];
  sCspsData csps = csps_data();
  uint32_t now = Delay_Tick;
  static uint32_t last = 0;
  uint32_t diff = DelayDiff(now, last);
  last = now;

  static GPIO_PinState clutch = GPIO_PIN_RESET;
  static uint8_t was_found = 0;
  static uint8_t last_start_triggered = 0;
  static uint32_t last_start = 0;
  static uint32_t turns_count_last = 0;
  static float oldanglesbeforeignite[ECU_CYLINDERS_COUNT] = {0,0,0,0};
  static float oldanglesbeforeinject[ECU_CYLINDERS_COUNT] = {0,0,0,0};
  static uint8_t saturated[ECU_CYLINDERS_COUNT] = { 1,1,1,1 };
  static uint8_t ignited[ECU_CYLINDERS_COUNT] = { 1,1,1,1 };
  static uint8_t injected[ECU_CYLINDERS_COUNT] = { 1,1,1,1 };
  static uint8_t ignition_ready[ECU_CYLINDERS_COUNT] = { 0,0,0,0 };
  static uint8_t injection[ECU_CYLINDERS_COUNT] = { 1,1,1,1 };
  static uint8_t ign_prepare[ECU_CYLINDERS_COUNT] = { 0,0,0,0 };
  static uint8_t knock_process[ECU_CYLINDERS_COUNT] = { 0,0,0,0 };
  static uint8_t knock_busy = 0;
  static uint8_t knock_cylinder = 0;
  static uint8_t inj_was_phased = 0;
  static uint8_t ign_was_phased = 0;
  float angle_injection[ECU_CYLINDERS_COUNT];
  float angle_ignition[ECU_CYLINDERS_COUNT];
  static float ignition_saturate[ECU_CYLINDERS_COUNT];
  static float ignition_ignite[ECU_CYLINDERS_COUNT];
  static uint32_t ignition_saturate_time[ECU_CYLINDERS_COUNT];
  static uint32_t ignition_ignite_time[ECU_CYLINDERS_COUNT];
  static float anglesbeforeignite[ECU_CYLINDERS_COUNT];
  float anglesbeforeinject[ECU_CYLINDERS_COUNT];

  float throttle;
  float angle = csps_getphasedangle(csps);
  float rpm = csps_getrpm(csps);
  float found = csps_isfound();
  float uspa_koff = 0.8f;
  static float uspa = 1000.0f;
  float uspa_raw = csps_getuspa(csps);
  float period = csps_getperiod(csps);
  float time_sat;
  float time_pulse;
  float angle_ignite_koff = 0.1f;
  float inj_phase_koff = 0.0f;
  float angle_ignite_param;
  float inj_phase_param;
  float inj_phase_temp;
  static float angle_ignite = 10.0f;
  static float inj_phase = 200.0f;
  float saturate;
  float angles_injection_per_turn;
  float angles_ignition_per_turn;
  float inj_pulse;
  float inj_angle;
  static float cy_ignition[ECU_CYLINDERS_COUNT] = {10,10,10,10};
  float cy_injection[ECU_CYLINDERS_COUNT];
  float var, var2, knock;
  uint8_t injection_phase_by_end;
  uint8_t phased_ignition;
  uint8_t phased_injection;
  uint8_t cy_count_ignition;
  uint8_t cy_count_injection;
  uint8_t individual_coils;
  uint8_t single_coil;
  uint8_t use_tsps;
  uint8_t injector_channel;
  uint8_t start_allowed = gParameters.StartAllowed;
  uint8_t cutoff_inj_act, cutoff_ign_act;
  uint8_t shift_inj_act, shift_ign_act;
  uint8_t shiftEnabled = gEcuParams.shiftMode > 0;
  uint8_t is_phased = csps_isphased(csps);
  HAL_StatusTypeDef throttleStatus = HAL_OK;
  GPIO_PinState clutch_pin;
  uint32_t clutch_time;
  uint32_t turns_count = csps_getturns(csps);

  if(shiftEnabled) {
    clutch_pin = sens_get_clutch(&clutch_time);
    if(clutch_pin != clutch)
    {
      if(clutch_time > 500) {
        clutch = clutch_pin;
      }
    }
    throttleStatus = sens_get_throttle_position(&throttle);
  }
  if(throttleStatus != HAL_OK)
    shiftEnabled = 0;
  if(!shiftEnabled)
    clutch = GPIO_PIN_RESET;

  injector_channel = table->inj_channel;
  single_coil = gEcuParams.isSingleCoil;
  individual_coils = gEcuParams.isIndividualCoils;
  use_tsps = gEcuParams.useTSPS;
  phased_injection = use_tsps && is_phased;
  phased_ignition = phased_injection && individual_coils && !single_coil;
  cy_count_injection = phased_injection ? ECU_CYLINDERS_COUNT : ECU_CYLINDERS_COUNT_HALF;
  cy_count_ignition = phased_ignition ? ECU_CYLINDERS_COUNT : ECU_CYLINDERS_COUNT_HALF;
  angle_ignite_param = gParameters.IgnitionAngle;
  injection_phase_by_end = table->is_fuel_phase_by_end;
  inj_phase_param = gParameters.InjectionPhase;
  inj_pulse = gParameters.InjectionPulse;


  if(found != was_found) {
    was_found = found;
    last_start_triggered = 1;
    last_start = now;
    turns_count_last = turns_count;
  }

  if(last_start_triggered) {
    if(found && use_tsps && !is_phased) {
      if(DelayDiff(now, last_start) < 1000000 && turns_count - turns_count_last < 3) {
        start_allowed = 0;
      } else {
        last_start_triggered = 0;
      }
    } else {
      last_start_triggered = 0;
    }
  }

  if(single_coil) {
    angle_ignite_koff = 0.005f;
  } else if(!phased_ignition) {
    angle_ignite_koff = 0.01f;
  }

  //TODO: remove this filter when fixed the bug of incorrect injection pulses
  inj_phase_koff = diff * 0.0001f;
  if(inj_phase_koff > 0.8f)
    inj_phase_koff = 0.8f;
  if(inj_phase_koff < 0.000001f)
    inj_phase_koff = 0.000001f;

  uspa_koff = diff * 0.002f;

  uspa = uspa_raw * uspa_koff + uspa * (1.0f - uspa_koff);
  angle_ignite = angle_ignite_param * angle_ignite_koff + angle_ignite * (1.0f - angle_ignite_koff);
  inj_phase = inj_phase_param * inj_phase_koff + inj_phase * (1.0f - inj_phase_koff);
  inj_phase_temp = inj_phase;

  if(single_coil) {
    time_sat = period * 0.5f * 0.65f;
    time_pulse = period * 0.5f * 0.35f;
  } else {
    time_sat = gParameters.IgnitionPulse;
    time_pulse = 2500;
  }

  if(phased_ignition) {
    angles_ignition_per_turn = 720.0f;
    for(int i = 0; i < cy_count_ignition; i++) {
      var = angle_ignite;
      var += table->cy_corr_ignition[i];
      var += gEcuCorrections.knock_ignition_correctives[i];
      if(!saturated[i] && !ignited[i]) {
        var2 = anglesbeforeignite[i] - var + cy_ignition[i];
        if(var2 > 0) {
          cy_ignition[i] = var;
        }
      }
    }
  } else {
    angles_ignition_per_turn = 360.0f;
    knock_busy = 0;
    for(int i = 0; i < ECU_CYLINDERS_COUNT_HALF; i++) {
      saturated[ECU_CYLINDERS_COUNT - 1 - i] = 1;
      ignited[ECU_CYLINDERS_COUNT - 1 - i] = 1;
      ignition_saturate_time[ECU_CYLINDERS_COUNT - 1 - i] = 0;
      knock_process[ECU_CYLINDERS_COUNT - 1 - i] = 0;
    }
    for(int i = 0; i < ECU_CYLINDERS_COUNT_HALF; i++) {
      var = angle_ignite;
      var += (table->cy_corr_ignition[i] + table->cy_corr_ignition[ECU_CYLINDERS_COUNT - 1 - i]) * 0.5f;
      var += (gEcuCorrections.knock_ignition_correctives[i] + gEcuCorrections.knock_ignition_correctives[ECU_CYLINDERS_COUNT - 1 - i]) * 0.5f;
      if(!saturated[i] && !ignited[i]) {
        var2 = anglesbeforeignite[i] - var + cy_ignition[i];
        if(var2 > 0) {
          cy_ignition[i] = var;
        }
      }
    }
  }

  if(phased_injection) {
    angles_injection_per_turn = 720.0f;
    for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
      cy_injection[i] = (table->cy_corr_injection[i] + 1.0f) * inj_pulse;
      cy_injection[i] *= gEcuCorrections.knock_injection_correctives[i] + 1.0f;
    }
  } else {
    angles_injection_per_turn = 360.0f;
    inj_pulse *= 0.5;
    for(int i = 0; i < ECU_CYLINDERS_COUNT_HALF; i++) {
      injection[ECU_CYLINDERS_COUNT - 1 - i] = 1;
      injected[ECU_CYLINDERS_COUNT - 1 - i] = 1;
      cy_injection[i] = ((table->cy_corr_injection[i] + table->cy_corr_injection[ECU_CYLINDERS_COUNT - 1 - i]) * 0.5f + 1.0f) * inj_pulse;
      cy_injection[i] *= (gEcuCorrections.knock_injection_correctives[i] + gEcuCorrections.knock_injection_correctives[ECU_CYLINDERS_COUNT - 1 - i]) * 0.5f + 1.0f;
    }
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
      ignition_saturate_time[i] = 0;
      ignition_ignite_time[i] = 0;
      ignition_ready[i] = 0;
    }
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

  if((found || gIITest.StartedTime) && start_allowed && !gIgnCanShutdown)
  {
    IGN_NALLOW_GPIO_Port->BSRR = IGN_NALLOW_Pin << 16;
    gInjChPorts[injector_channel]->BSRR = gInjChPins[injector_channel];
    gInjChPorts[injector_channel ^ 1]->BSRR = gInjChPins[injector_channel ^ 1] << 16;

    //Ignition/Injection Test
    if(gIITest.StartedTime) {
      Knock_SetState(0);
      if(DelayDiff(now, gIITest.StartedTime) >= gIITest.Period) {
        gIITest.InjectionTriggered = 0;
        gIITest.CompletedCount++;
        if(gIITest.CompletedCount >= gIITest.Count) {
          memset(knock_process, 0, sizeof(knock_process));
          knock_busy = 0;
          Knock_SetState(1);
          gIITest.StartedTime = 0;
          gIITest.IgnitionEnabled = 0;
          gIITest.InjectionEnabled = 0;
          return;
        } else {
          gIITest.StartedTime += gIITest.Period;
          if(!gIITest.StartedTime)
            gIITest.StartedTime++;
        }
      }

      if(DelayDiff(now, gIITest.StartedTime) < gIITest.IgnitionPulse) {
        if(single_coil) {
          if(gIITest.IgnitionEnabled) {
            ecu_coil_saturate(1, 0);
          }
        } else if(individual_coils) {
          for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
            if(((gIITest.IgnitionEnabled >> i) & 1)) {
              ecu_coil_saturate(ECU_CYLINDERS_COUNT, i);
            } else {
              ecu_coil_ignite(ECU_CYLINDERS_COUNT, i);
            }
          }
        } else {
          for(int i = 0; i < ECU_CYLINDERS_COUNT_HALF; i++) {
            if(((gIITest.IgnitionEnabled >> (ECU_CYLINDERS_COUNT - 1 - i)) & 1)) {
              ecu_coil_saturate(ECU_CYLINDERS_COUNT_HALF, i);
            } else {
              ecu_coil_ignite(ECU_CYLINDERS_COUNT_HALF, i);
            }
          }
        }
      } else {
        for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
          ecu_coil_ignite(ECU_CYLINDERS_COUNT, i);
        }
      }

      if(!gIITest.InjectionTriggered) {
        gIITest.InjectionTriggered = 1;
        for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
          if(((gIITest.InjectionEnabled >> i) & 1)) {
            ecu_inject(cy_count_injection, i, gIITest.InjectionPulse);
          }
        }
      }
    } else if(found) {
      if(period < time_sat + time_pulse) {
        time_sat = period * ((float)time_sat / (float)(time_sat + time_pulse));
      }

      saturate = time_sat / uspa;
      inj_angle = inj_pulse / uspa;

      if(inj_angle < diff / uspa * 1.5f)
        inj_angle = diff / uspa * 1.5f;

      if(!injection_phase_by_end) {
        inj_phase_temp += inj_angle;
      }

      while(inj_phase_temp > angles_injection_per_turn * 0.5f) {
        inj_phase_temp -= angles_injection_per_turn;
      }

      //Knock part
      for(int i = 0; i < cy_count_ignition; i++)
      {
        if(knock_busy) {
          if(knock_process[i]) {
            if(angle_ignition[i] < gKnockDetectStartAngle || angle_ignition[i] >= gKnockDetectEndAngle) {
              Knock_SetState(0);
              knock_process[i] = 0;
              knock_cylinder = i;
              knock_busy = 0;
            }
          }
        } else {
          if(angle_ignition[i] >= gKnockDetectStartAngle && angle_ignition[i] < gKnockDetectEndAngle) {
            knock_busy = 1;
            knock = adc_get_voltage(AdcChKnock);
            if(cy_count_ignition == ECU_CYLINDERS_COUNT) {
              gStatus.Knock.Voltages[knock_cylinder] = knock;
              gStatus.Knock.Updated[knock_cylinder] = 1;
              gStatus.Knock.Period[knock_cylinder] = DelayDiff(now, gStatus.Knock.LastTime[knock_cylinder]);
              gStatus.Knock.LastTime[knock_cylinder] = now;
            } else if(cy_count_ignition == ECU_CYLINDERS_COUNT_HALF) {
              gStatus.Knock.Voltages[knock_cylinder] = knock;
              gStatus.Knock.Voltages[ECU_CYLINDERS_COUNT - 1 - knock_cylinder] = knock;
              gStatus.Knock.Updated[knock_cylinder] = 1;
              gStatus.Knock.Updated[ECU_CYLINDERS_COUNT - 1 - knock_cylinder] = 1;
              gStatus.Knock.Period[knock_cylinder] = DelayDiff(now, gStatus.Knock.LastTime[knock_cylinder]);
              gStatus.Knock.Period[ECU_CYLINDERS_COUNT - 1 - knock_cylinder] = DelayDiff(now, gStatus.Knock.LastTime[ECU_CYLINDERS_COUNT - 1 - knock_cylinder]);
              gStatus.Knock.LastTime[knock_cylinder] = now;
              gStatus.Knock.LastTime[ECU_CYLINDERS_COUNT - 1 - knock_cylinder] = now;
            }
            knock_process[i] = 1;
            Knock_SetState(1);
          }
        }
      }

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
          if(!saturated[i] && !ignited[i] && (ignition_ignite_time[i] == 0 || DelayDiff(now, ignition_ignite_time[i]) >= ignition_ignite[i]))
          {
            if(cy_count_ignition == ECU_CYLINDERS_COUNT) {
              ignition_ready[i] = 1;
            } else if(cy_count_ignition == ECU_CYLINDERS_COUNT_HALF) {
              ignition_ready[i] = 1;
              ignition_ready[ECU_CYLINDERS_COUNT - 1 - i] = 1;
            }
            ignition_ignite_time[i] = 0;
            ignition_ignite[i] = 0;
            saturated[i] = 1;

            if(single_coil) {
              ecu_coil_saturate(1, 0);
            } else {
              shift_ign_act = !shiftEnabled || ecu_shift_ign_act(cy_count_ignition, i, clutch, rpm, throttle);
              cutoff_ign_act = ecu_cutoff_ign_act(cy_count_ignition, i, rpm);
              if(shift_ign_act && cutoff_ign_act)
                ecu_coil_saturate(cy_count_ignition, i);
            }

            ignition_saturate_time[i] = now;
            ignition_saturate[i] = time_sat;
          }
        }

        if(ign_prepare[i] || oldanglesbeforeignite[i] - anglesbeforeignite[i] < -90.0f)
        {
          if((!ignited[i] && saturated[i]) || ign_prepare[i])
          {
            ign_prepare[i] = 1;
            if(ignition_saturate_time[i] == 0 || DelayDiff(now, ignition_saturate_time[i]) >= ignition_saturate[i]) {
              ign_prepare[i] = 0;
              ignited[i] = 1;
              saturated[i] = 0;
              ignition_saturate_time[i] = 0;
              ignition_saturate[i] = 0;

              if(single_coil) {
                shift_ign_act = !shiftEnabled || ecu_shift_ign_act(cy_count_ignition, i, clutch, rpm, throttle);
                cutoff_ign_act = ecu_cutoff_ign_act(cy_count_ignition, i, rpm);
                if(shift_ign_act && cutoff_ign_act)
                  ecu_coil_ignite(1, 0);
              } else {
                ecu_coil_ignite(cy_count_ignition, i);
              }
              ignition_ignite[i] = time_pulse;
              ignition_ignite_time[i] = now;
            }
          }
        } else {
          ignited[i] = 0;
        }

        oldanglesbeforeignite[i] = anglesbeforeignite[i];
      }

      //Injection part
      for(int i = 0; i < cy_count_injection; i++)
      {
        if(angle_injection[i] < inj_phase_temp)
          anglesbeforeinject[i] = -angle_injection[i] + inj_phase_temp;
        else
          anglesbeforeinject[i] = angles_injection_per_turn - angle_injection[i] + inj_phase_temp;

        if(oldanglesbeforeinject[i] - anglesbeforeinject[i] > 0.0f && oldanglesbeforeinject[i] - anglesbeforeinject[i] > 180.0f)
          anglesbeforeinject[i] = oldanglesbeforeinject[i];

        if(anglesbeforeinject[i] - inj_angle < 0.0f)
        {
          if(!injection[i])
          {
            injection[i] = 1;
            shift_inj_act = !shiftEnabled || ecu_shift_inj_act(cy_count_ignition, i, clutch, rpm, throttle);
            cutoff_inj_act = ecu_cutoff_inj_act(cy_count_ignition, i, rpm);
            if(ignition_ready[i] && cutoff_inj_act && shift_inj_act && cy_injection[i] > 0.0f)
              ecu_inject(cy_count_injection, i, cy_injection[i]);
          }
        }

        if(oldanglesbeforeinject[i] - anglesbeforeinject[i] < -90.0f)
        {
          if(!injected[i] && injection[i])
          {
            //TODO: this is probably a bug, but works well, without extra pulses on duty cycle >= 1.0
            injection[i] = 0;
            injector_isenabled(i, &injected[i]);
            injected[i] ^= 1;
          }
        }
        else injected[i] = 0;

        oldanglesbeforeinject[i] = anglesbeforeinject[i];
      }
    }
  } else {
    for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
      oldanglesbeforeignite[i] = 0.0f;
      ignition_saturate_time[i] = 0;
      ignition_ignite_time[i] = 0;
      saturated[i] = 1;
      ignited[i] = 1;
      injection[i] = 0;
      injected[i] = 0;
      ignition_ready[i] = 0;
      knock_process[i] = 0;
      knock_busy = 0;
    }

    Knock_SetState(0);

    IGN_NALLOW_GPIO_Port->BSRR = IGN_NALLOW_Pin;
    for(int i = 0; i < ITEMSOF(gInjChPins); i++)
      gInjChPorts[i]->BSRR = gInjChPins[i] << 16;
    for(int i = 0; i < ITEMSOF(gIgnPorts); i++)
      gIgnPorts[i]->BSRR = gIgnPins[i] << 16;
    for(int i = 0; i < ITEMSOF(gInjPorts); i++)
      gInjPorts[i]->BSRR = gInjPins[i];

  }
}

static void ecu_fuelpump_process(void)
{
  static uint32_t active_last = 0;
  static uint8_t active = 0;
  static uint8_t was_rotating = 1;
  uint32_t now = Delay_Tick;
  uint8_t rotates = csps_isrotates();
  uint8_t can_shutdown = gIgnCanShutdown;
  uint8_t time_to_last = DelayDiff(now, active_last) < 1000000;

  if(gForceParameters.Enable.FuelPumpRelay) {
    active = gForceParameters.FuelPumpRelay > 0;
    active_last = now;
  } else {
    if(((was_rotating && time_to_last) || rotates) && !can_shutdown) {
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

#define CHECK_STATUS(iserror, cod, link) \
  if((link)) { \
    gCheckBitmap[cod >> 3] |= 1 << (cod & 7); \
    iserror |= 1; \
  }

static void ecu_checkengine_loop(void)
{
  static uint32_t hal_error_last = 0;
  static uint8_t was_error = 0;
  uint32_t hal_now = HAL_GetTick();
  uint8_t running = csps_isrunning();
  uint8_t iserror = 0;
  uint8_t status_reset = gStatusReset;

  memset(gCheckBitmap, 0, sizeof(gCheckBitmap));

  CHECK_STATUS(iserror, CheckFlashLoadFailure, gStatus.Flash.Struct.Load != HAL_OK);
  CHECK_STATUS(iserror, CheckFlashSaveFailure, gStatus.Flash.Struct.Save != HAL_OK);
  CHECK_STATUS(iserror, CheckFlashInitFailure, gStatus.Flash.Struct.Init != HAL_OK);
  CHECK_STATUS(iserror, CheckBkpsramSaveFailure, gStatus.Bkpsram.Struct.CriticalSave != HAL_OK || gStatus.Bkpsram.Struct.CorrsSave != HAL_OK)
#ifdef DEBUG
  CHECK_STATUS(iserror, CheckBkpsramLoadFailure, gStatus.Bkpsram.Struct.CriticalLoad != HAL_OK || gStatus.Bkpsram.Struct.CorrsLoad != HAL_OK);
#endif

  CHECK_STATUS(iserror, CheckSensorMapFailure, gStatus.Sensors.Struct.Map != HAL_OK);
  CHECK_STATUS(iserror, CheckSensorKnockFailure, gStatus.Sensors.Struct.Knock != HAL_OK);
  CHECK_STATUS(iserror, CheckSensorCspsFailure, gStatus.Sensors.Struct.Csps != HAL_OK);
  CHECK_STATUS(iserror, CheckSensorTspsFailure, gStatus.Sensors.Struct.Tsps != HAL_OK);
  CHECK_STATUS(iserror, CheckSensorAirTempFailure, gStatus.Sensors.Struct.AirTemp != HAL_OK);
  CHECK_STATUS(iserror, CheckSensorEngineTempFailure, gStatus.Sensors.Struct.EngineTemp != HAL_OK);
  CHECK_STATUS(iserror, CheckSensorTPSFailure, gStatus.Sensors.Struct.ThrottlePos != HAL_OK);
  CHECK_STATUS(iserror, CheckSensorRefVoltageFailure, gStatus.Sensors.Struct.ReferenceVoltage != HAL_OK);
  CHECK_STATUS(iserror, CheckSensorPwrVoltageFailure, gStatus.Sensors.Struct.PowerVoltage != HAL_OK);
  CHECK_STATUS(iserror, CheckSensorLambdaFailure, gStatus.Sensors.Struct.Lambda != HAL_OK);
  CHECK_STATUS(iserror, CheckOutputDriverFailure, gStatus.OutputStatus != HAL_OK);

  CHECK_STATUS(iserror, CheckCanInitFailure, gStatus.CanInitStatus != HAL_OK);
  CHECK_STATUS(iserror, CheckCanTestFailure, gStatus.CanTestStatus != HAL_OK);
  CHECK_STATUS(iserror, CheckKlineProtocolFailure, gStatus.KlineProtocolStatus != HAL_OK);
  CHECK_STATUS(iserror, CheckKlineLoopbackFailure, gStatus.KlineLoopbackStatus != HAL_OK);

  CHECK_STATUS(iserror, CheckInjector4OpenCircuit, gStatus.OutputDiagnostic.Injectors.Diagnostic.Data.InjCy4 == OutputDiagOpenCircuit);
  CHECK_STATUS(iserror, CheckInjector4ShortToBatOrOverheat, gStatus.OutputDiagnostic.Injectors.Diagnostic.Data.InjCy4 == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(iserror, CheckInjector4ShortToGND, gStatus.OutputDiagnostic.Injectors.Diagnostic.Data.InjCy4 == OutputDiagShortToGnd);
  CHECK_STATUS(iserror, CheckInjector3OpenCircuit, gStatus.OutputDiagnostic.Injectors.Diagnostic.Data.InjCy3 == OutputDiagOpenCircuit);
  CHECK_STATUS(iserror, CheckInjector3ShortToBatOrOverheat, gStatus.OutputDiagnostic.Injectors.Diagnostic.Data.InjCy3 == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(iserror, CheckInjector3ShortToGND, gStatus.OutputDiagnostic.Injectors.Diagnostic.Data.InjCy3 == OutputDiagShortToGnd);
  CHECK_STATUS(iserror, CheckInjector2OpenCircuit, gStatus.OutputDiagnostic.Injectors.Diagnostic.Data.InjCy2 == OutputDiagOpenCircuit);
  CHECK_STATUS(iserror, CheckInjector2ShortToBatOrOverheat, gStatus.OutputDiagnostic.Injectors.Diagnostic.Data.InjCy2 == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(iserror, CheckInjector2ShortToGND, gStatus.OutputDiagnostic.Injectors.Diagnostic.Data.InjCy2 == OutputDiagShortToGnd);
  CHECK_STATUS(iserror, CheckInjector1OpenCircuit, gStatus.OutputDiagnostic.Injectors.Diagnostic.Data.InjCy1 == OutputDiagOpenCircuit);
  CHECK_STATUS(iserror, CheckInjector1ShortToBatOrOverheat, gStatus.OutputDiagnostic.Injectors.Diagnostic.Data.InjCy1 == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(iserror, CheckInjector1ShortToGND, gStatus.OutputDiagnostic.Injectors.Diagnostic.Data.InjCy1 == OutputDiagShortToGnd);
  CHECK_STATUS(iserror, CheckInjectorCommunicationFailure, gStatus.OutputDiagnostic.Injectors.Availability != HAL_OK);

  CHECK_STATUS(iserror, CheckCheckEngineOpenCirtuit, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.CheckEngine == OutputDiagOpenCircuit);
  CHECK_STATUS(iserror, CheckCheckEngineShortToBatOrOverheat, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.CheckEngine == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(iserror, CheckCheckEngineShortToGND, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.CheckEngine == OutputDiagShortToGnd);
  CHECK_STATUS(iserror, CheckSpeedMeterOpenCirtuit, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.Speedmeeter == OutputDiagOpenCircuit);
  CHECK_STATUS(iserror, CheckSpeedMeterShortToBatOrOverheat, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.Speedmeeter == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(iserror, CheckSpeedMeterShortToGND, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.Speedmeeter == OutputDiagShortToGnd);
  CHECK_STATUS(iserror, CheckTachometerOpenCirtuit, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.Tachometer == OutputDiagOpenCircuit);
  CHECK_STATUS(iserror, CheckTachometerShortToBatOrOverheat, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.Tachometer == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(iserror, CheckTachometerShortToGND, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.Tachometer == OutputDiagShortToGnd);
  CHECK_STATUS(iserror, CheckFuelPumpOpenCirtuit, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.FuelPumpRelay == OutputDiagOpenCircuit);
  CHECK_STATUS(iserror, CheckFuelPumpShortToBatOrOverheat, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.FuelPumpRelay == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(iserror, CheckFuelPumpShortToGND, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.FuelPumpRelay == OutputDiagShortToGnd);
  CHECK_STATUS(iserror, CheckOutputs1CommunicationFailure, gStatus.OutputDiagnostic.Outs1.Availability != HAL_OK);

  CHECK_STATUS(iserror, CheckOutIgnOpenCirtuit, gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.OutIgn == OutputDiagOpenCircuit);
  CHECK_STATUS(iserror, CheckOutIgnShortToBatOrOverheat, gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.OutIgn == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(iserror, CheckOutIgnShortToGND, gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.OutIgn == OutputDiagShortToGnd);
  //CHECK_STATUS(iserror, CheckOutRsvd1OpenCirtuit, gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.OutRsvd1 == OutputDiagOpenCircuit);
  CHECK_STATUS(iserror, CheckOutRsvd1ShortToBatOrOverheat, gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.OutRsvd1 == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(iserror, CheckOutRsvd1ShortToGND, gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.OutRsvd1 == OutputDiagShortToGnd);
  //CHECK_STATUS(iserror, CheckStarterRelayOpenCirtuit, gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.StarterRelay == OutputDiagOpenCircuit);
  CHECK_STATUS(iserror, CheckStarterRelayShortToBatOrOverheat, gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.StarterRelay == OutputDiagShortToBatOrOvertemp);
  //CHECK_STATUS(iserror, CheckStarterRelayShortToGND, gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.StarterRelay == OutputDiagShortToGnd);
  CHECK_STATUS(iserror, CheckFanRelayOpenCirtuit, gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.FanRelay == OutputDiagOpenCircuit);
  CHECK_STATUS(iserror, CheckFanRelayShortToBatOrOverheat, gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.FanRelay == OutputDiagShortToBatOrOvertemp);
  //CHECK_STATUS(iserror, CheckFanRelayShortToGND, gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.FanRelay == OutputDiagShortToGnd);
  CHECK_STATUS(iserror, CheckOutputs2CommunicationFailure, gStatus.OutputDiagnostic.Outs2.Availability != HAL_OK);

  CHECK_STATUS(iserror, CheckIdleValveFailure, gStatus.OutputDiagnostic.IdleValvePosition.Status != HAL_OK);
  CHECK_STATUS(iserror, CheckIdleValveDriverFailure, gStatus.IdleValvePosition != HAL_OK);
  CHECK_STATUS(iserror, CheckInjectionUnderflow, gStatus.InjectionUnderflow != HAL_OK);
  CHECK_STATUS(iserror, CheckAdcFailure, gStatus.AdcStatus != HAL_OK);

  if(gEcuParams.useLambdaSensor) {
    CHECK_STATUS(iserror, CheckLambdaCommunicationFailure, gStatus.O2Status != HAL_OK);
    CHECK_STATUS(iserror, CheckLambdaVMShortToBat, gStatus.O2Diagnostic.VM == O2DiagShortToBat);
    CHECK_STATUS(iserror, CheckLambdaVMLowBattery, gStatus.O2Diagnostic.VM == O2DiagNoPower);
    CHECK_STATUS(iserror, CheckLambdaVMShortToGND, gStatus.O2Diagnostic.VM == O2DiagShortToGnd);
    CHECK_STATUS(iserror, CheckLambdaUNShortToBat, gStatus.O2Diagnostic.UN == O2DiagShortToBat);
    CHECK_STATUS(iserror, CheckLambdaUNLowBattery, gStatus.O2Diagnostic.UN == O2DiagNoPower);
    CHECK_STATUS(iserror, CheckLambdaUNShortToGND, gStatus.O2Diagnostic.UN == O2DiagShortToGnd);
    CHECK_STATUS(iserror, CheckLambdaIAIPShortToBat, gStatus.O2Diagnostic.IAIP == O2DiagShortToBat);
    CHECK_STATUS(iserror, CheckLambdaIAIPLowBattery, gStatus.O2Diagnostic.IAIP == O2DiagNoPower);
    CHECK_STATUS(iserror, CheckLambdaIAIPShortToGND, gStatus.O2Diagnostic.IAIP == O2DiagShortToGnd);
    CHECK_STATUS(iserror, CheckLambdaDIAHGDShortToBat, gStatus.O2Diagnostic.DIAHGD == O2DiagShortToBat);
    CHECK_STATUS(iserror, CheckLambdaDIAHGDOpenCirtuit, gStatus.O2Diagnostic.DIAHGD == O2DiagNoPower);
    CHECK_STATUS(iserror, CheckLambdaDIAHGDShortToGND, gStatus.O2Diagnostic.DIAHGD == O2DiagShortToGnd);
    CHECK_STATUS(iserror, CheckEngineLeanMixture, gStatus.LeanMixture.is_error && gStatus.LeanMixture.error_time > 5000);
    CHECK_STATUS(iserror, CheckEngineRichMixture, gStatus.RichMixture.is_error && gStatus.RichMixture.error_time > 5000);
    CHECK_STATUS(iserror, CheckEngineLeanIdleMixture, gStatus.LeanIdleMixture.is_error && gStatus.LeanIdleMixture.error_time > 5000);
    CHECK_STATUS(iserror, CheckEngineRichIdleMixture, gStatus.RichIdleMixture.is_error && gStatus.RichIdleMixture.error_time > 5000);
  }
  if(gEcuParams.useKnockSensor) {
    CHECK_STATUS(iserror, CheckKnockDetonationFound, (gStatus.Knock.GeneralStatus & KnockStatusDedonation) > 0);
    CHECK_STATUS(iserror, CheckKnockLowNoiseLevel, (gStatus.Knock.GeneralStatus & KnockStatusLowNoise) > 0);
  }
  if(gEcuParams.useTSPS) {
    CHECK_STATUS(iserror, CheckTspsDesynchronized, gStatus.TspsSyncStatus != HAL_OK);
  }
  CHECK_STATUS(iserror, CheckSensorMapTpsMismatch, gStatus.MapTpsRelation.is_error && gStatus.MapTpsRelation.error_time > 5000);

  CHECK_STATUS(iserror, CheckNoOilPressure, gStatus.OilPressure.is_error);
  CHECK_STATUS(iserror, CheckNoBatteryCharge, gStatus.BatteryCharge.is_error);

  gDiagErrors.Bits.csps_error = gStatus.Sensors.Struct.Csps != HAL_OK;

  gDiagErrors.Bits.csps_error = gStatus.Sensors.Struct.Csps != HAL_OK;
  gDiagErrors.Bits.not_used1 = 0;
  gDiagErrors.Bits.eeprom_error = gStatus.Flash.Byte > 0;
  gDiagErrors.Bits.lambda_heater = gStatus.O2Diagnostic.DIAHGD > 0;
  gDiagErrors.Bits.tsps_error = gStatus.Sensors.Struct.Tsps != HAL_OK;
  gDiagErrors.Bits.reset_error = 0;
  gDiagErrors.Bits.ram_error = 0;
  gDiagErrors.Bits.flash_error = 0;

  gDiagErrors.Bits.low_voltage = gParameters.PowerVoltage < 7.0f;
  gDiagErrors.Bits.not_used2 = 0;
  gDiagErrors.Bits.not_used3 = 0;
  gDiagErrors.Bits.engine_temp_low = gStatus.Sensors.Struct.EngineTemp != HAL_OK;
  gDiagErrors.Bits.lambda_low = gStatus.Sensors.Struct.Lambda != HAL_OK;
  gDiagErrors.Bits.tps_low = gStatus.Sensors.Struct.ThrottlePos != HAL_OK;
  gDiagErrors.Bits.maf_low = gStatus.Sensors.Struct.Map != HAL_OK;
  gDiagErrors.Bits.low_noise = (gStatus.Knock.GeneralStatus & KnockStatusLowNoise) > 0;

  gDiagErrors.Bits.high_voltage = gParameters.PowerVoltage > 16.0f;
  gDiagErrors.Bits.not_used4 = 0;
  gDiagErrors.Bits.not_used5 = 0;
  gDiagErrors.Bits.engine_temp_high = gStatus.Sensors.Struct.EngineTemp != HAL_OK;
  gDiagErrors.Bits.lambda_high = gStatus.Sensors.Struct.Lambda != HAL_OK;
  gDiagErrors.Bits.tps_high = gStatus.Sensors.Struct.ThrottlePos != HAL_OK;
  gDiagErrors.Bits.maf_high = gStatus.Sensors.Struct.Map != HAL_OK;
  gDiagErrors.Bits.high_noise = (gStatus.Knock.GeneralStatus & KnockStatusDedonation) > 0;

  gDiagErrors.Bits.knock_error = gStatus.Sensors.Struct.Knock != HAL_OK;
  gDiagErrors.Bits.no_immo = 0;
  gDiagErrors.Bits.not_used6 = 0;
  gDiagErrors.Bits.lambda_no_activity = gStatus.Sensors.Struct.Lambda != HAL_OK;
  gDiagErrors.Bits.lambda_no_on_lean = gStatus.Sensors.Struct.Lambda != HAL_OK;
  gDiagErrors.Bits.lambda_no_on_rich = gStatus.Sensors.Struct.Lambda != HAL_OK;
  gDiagErrors.Bits.speed_error = 0;
  gDiagErrors.Bits.idle_error = gStatus.IdleValvePosition != HAL_OK || gStatus.OutputDiagnostic.IdleValvePosition.Status != HAL_OK;

  if(iserror) {
    was_error = 1;
    hal_error_last = hal_now;
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


static void ecu_drag_process(void)
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

static void ecu_mem_process(void)
{
  if(!Mem.lock)
  {
    Mem.lock = 1;
    if(!Mem.issaving && !Mem.isloading) {
      if(Mem.savereq && !Mem.isloading) {
        Mem.issaving = 1;
      } else if(Mem.loadreq && !Mem.issaving) {
        Mem.isloading = 1;
      } else {
        Mem.lock = 0;
      }
    } else {
      Mem.lock = 0;
    }
  }
}

static void ecu_mem_loop(void)
{
  int8_t flashstatus = 0;

  if(Mem.issaving)
  {
    flashstatus = config_save_all(&gEcuParams, gEcuTable, ITEMSOF(gEcuTable));
    if(flashstatus)
    {
      PK_SaveConfigAcknowledge.ErrorCode = flashstatus == 1 ? 0 : 1;
      PK_SendCommand(Mem.savereqsrc, &PK_SaveConfigAcknowledge, sizeof(PK_SaveConfigAcknowledge));
      gStatus.Flash.Struct.Save = flashstatus;
      Mem.issaving = 0;
      Mem.savereq = 0;
      Mem.lock = 0;
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
      Mem.lock = 0;
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

static void ecu_corrections_loop(void)
{
  static uint8_t old_correction = 0;
  static uint32_t prev_conversion = 0;
  uint32_t now = Delay_Tick;
  uint8_t perform_correction = gEcuParams.performAdaptation;

  if(perform_correction != old_correction) {
    old_correction = perform_correction;
    if(perform_correction) {
      memset(&gEcuCorrectionsProgress, 0, sizeof(gEcuCorrectionsProgress));
      memset(&gEcuCorrections, 0, sizeof(gEcuCorrections));
    }
    prev_conversion = now;
  }
  if(DelayDiff(now, prev_conversion) > 500000)
  {
    prev_conversion = now;
    for(int y = 0; y < TABLE_FILLING_MAX; y++) {
      for(int x = 0; x < TABLE_ROTATES_MAX; x++) {
        float value = gEcuCorrectionsProgress.progress_ignitions[y][x];
        if(value > 1.0f) value = 1.0f / value;
        gEcuCorrections.progress_ignitions[y][x] = value * 255.0f;
      }
    }
    for(int y = 0; y < TABLE_PRESSURES_MAX; y++) {
      for(int x = 0; x < TABLE_ROTATES_MAX; x++) {
        float value = gEcuCorrectionsProgress.progress_fill_by_map[y][x];
        if(value > 1.0f) value = 1.0f / value;
        gEcuCorrections.progress_fill_by_map[y][x] = value * 255.0f;
      }
    }
    for(int y = 0; y < TABLE_THROTTLES_MAX; y++) {
      for(int x = 0; x < TABLE_ROTATES_MAX; x++) {
        float value = gEcuCorrectionsProgress.progress_map_by_thr[y][x];
        if(value > 1.0f) value = 1.0f / value;
        gEcuCorrections.progress_map_by_thr[y][x] = value * 255.0f;
      }
    }
    for(int y = 0; y < TABLE_TEMPERATURES_MAX; y++) {
      for(int x = 0; x < TABLE_ROTATES_MAX; x++) {
        float value = gEcuCorrectionsProgress.progress_idle_valve_to_rpm[y][x];
        if(value > 1.0f) value = 1.0f / value;
        gEcuCorrections.progress_idle_valve_to_rpm[y][x] = value * 255.0f;
      }
    }
  }
}

#ifndef SIMULATION
static void ecu_ign_process(void)
{
  static uint32_t tick_ready = 0;
  static uint8_t shutdown_process = 0;
  int8_t status;
  GPIO_PinState state;
  uint32_t now = Delay_Tick;
  uint32_t time;

  state = sens_get_ign(&time);

  if(state == GPIO_PIN_SET) {
    gIgnShutdownReady = 0;
    gIgnCanShutdown = 0;
    gIgnState = GPIO_PIN_SET;
    out_set_ign(GPIO_PIN_SET);
  }
  else if(time >= 100000) {
    gIgnState = GPIO_PIN_RESET;
    gIgnCanShutdown = 1;
    tick_ready = now;
  }

  if(gIgnCanShutdown && !gIgnShutdownReady) {
    tick_ready = now;
  }
  else if(gIgnCanShutdown && gIgnShutdownReady) {
    if(DelayDiff(now, tick_ready) > 5000000) {
      out_set_ign(GPIO_PIN_RESET);
    }
  }

  if(shutdown_process || (gIgnCanShutdown && !gIgnShutdownReady)) {
    shutdown_process = 1;
    status = ecu_shutdown_process();
    if(status) {
      if(gIgnCanShutdown) {
        gIgnShutdownReady = 1;
      }
      shutdown_process = 0;
    }
  }
}

static int8_t ecu_shutdown_process(void)
{
  static int8_t stage = 0;
  static uint32_t last_idle_valve_moving = 0;
  uint32_t now = Delay_Tick;
  int8_t status = 0;
  uint8_t idle_valve_position;
  uint8_t is_idle_valve_moving = out_is_idle_valve_moving();

  switch(stage) {
    case 0:
      last_idle_valve_moving = now;
      stage++;
      break;
    case 1:
      if(is_idle_valve_moving) {
        last_idle_valve_moving = now;
      } else if(DelayDiff(now, last_idle_valve_moving) > 100000) {
        stage++;
      }
      break;
    case 2:
      if(!Mem.critical_lock) {
        Mem.critical_lock = 1;
        idle_valve_position = out_get_idle_valve();
        gEcuCriticalBackup.idle_valve_position = idle_valve_position;
        config_save_critical_backup(&gEcuCriticalBackup);
        Mem.critical_lock = 0;
        stage++;
      }
      break;
    case 3:
      if(!Mem.lock) {
        Mem.lock = 1;
        stage++;
      }
      break;
    case 4:
      Mem.lock = 0;
      stage = 0;
      status = 1;
      break;
    default:
      stage = 0;
      break;
  }

  return status;
}
#endif

static void ecu_config_process(void)
{
  uint32_t table_number = gParameters.CurrentTable;
  sEcuTable *table = &gEcuTable[table_number];
  int32_t knock_frequency;
  int32_t knock_gain;
  sCspsData data = csps_data();
  float rpm = csps_getrpm(data);
  sMathInterpolateInput ipRpm;

  O2_SetLambdaForceEnabled(gEcuParams.isLambdaForceEnabled);

  if(gEcuParams.useKnockSensor) {
    ipRpm =  math_interpolate_input(rpm, table->rotates, table->rotates_count);
    knock_frequency = roundf(math_interpolate_1d(ipRpm, table->knock_filter_frequency));
    knock_gain = roundf(math_interpolate_1d(ipRpm, table->knock_gain));
    Knock_SetBandpassFilterFrequency(knock_frequency);
    Knock_SetGainValue(knock_gain);
    Knock_SetIntegratorTimeConstant(gEcuParams.knockIntegratorTime);
  }
}

static void ecu_can_init(void)
{
  gStatus.CanInitStatus = can_start(0x100, 0x7F0);
  if(gStatus.CanInitStatus == HAL_OK) {
    gCanTestStarted = 1;
  }
}

static void ecu_kline_init(void)
{
  kline_start();
}

static void ecu_can_loop(void)
{
  static sCanMessage message = {0};
  int8_t status;
  if(gStatus.CanInitStatus == HAL_OK) {
    if(gCanTestStarted) {
      status = can_test();
      if(status != 0) {
        gCanTestStarted = 0;
        gStatus.CanTestStatus = status > 0 ? HAL_OK : HAL_ERROR;
      }
    } else {
      status = can_receive(&message);
      if(status > 0) {
        ecu_can_process_message(&message);
      }
      can_loop();
    }
  }
}

static void kline_send_error(sKlineMessage *tx_message, uint8_t req_code, uint8_t err_code)
{
  tx_message->data[0] = 0x7F;
  tx_message->data[1] = req_code;
  tx_message->data[2] = err_code;
  tx_message->length = 3;
}

static void ecu_kline_loop(void)
{
  static uint32_t tester_last = 0;
  static uint8_t session_on = 0;
  const char *string = NULL;
  uint32_t now = Delay_Tick;
  sKlineStatus kline_status = kline_getstatus();
  sKlineMessage rx_message;
  sKlineMessage tx_message;
  int8_t status;

  gStatus.KlineProtocolStatus = kline_status.error_protocol;
  gStatus.KlineLoopbackStatus = kline_status.error_loopback;

  status = kline_receive(&rx_message);
  if(status > 0 && rx_message.length > 0) {
    if(rx_message.dst == 0x10) {
      tx_message.dst = rx_message.src;
      tx_message.src = rx_message.dst;
      tx_message.length = 0;

      do {
        if(rx_message.data[0] == 0x81) { //startCommunication
          tx_message.data[tx_message.length++] = rx_message.data[0] | 0x40;
          tx_message.data[tx_message.length++] = 0x6B;
          tx_message.data[tx_message.length++] = 0x8F;
        } else if(rx_message.data[0] == 0x82) { //stopCommunication
          tx_message.data[tx_message.length++] = rx_message.data[0] | 0x40;
        } else if(rx_message.data[0] == 0x10) { //startDiagnosticSession
          if(rx_message.data[1] != 0x81) { kline_send_error(&tx_message, rx_message.data[0], 0x12); break; }
          tx_message.data[tx_message.length++] = rx_message.data[0] | 0x40;
          tx_message.data[tx_message.length++] = rx_message.data[1];
          switch(rx_message.data[2]) {
            case 0x0A:
              kline_setbaud(10400);
              kline_start();
              session_on = 1;
              break;
            case 0x26:
              kline_setbaud(38400);
              kline_start();
              session_on = 2;
              break;
            case 0x39:
              kline_setbaud(57600);
              kline_start();
              session_on = 2;
              break;
            case 0x73:
              kline_setbaud(115200);
              kline_start();
              session_on = 2;
              break;
            default:
              kline_send_error(&tx_message, rx_message.data[0], 0x12);
              break;
          }
        } else if(rx_message.data[0] == 0x20) { //stopDiagnosticSession
          tx_message.data[tx_message.length++] = rx_message.data[0] | 0x40;
          if(session_on > 1) {
            kline_setbaud(10400);
            kline_start();
          }
          session_on = 0;
        } else if(rx_message.data[0] == 0x3E) { //testerPresent
          if(rx_message.data[1] == 1) {
            tx_message.data[tx_message.length++] = rx_message.data[0] | 0x40;
          }
        } else if(rx_message.data[0] == 0x11) { //ecuReset
          if(rx_message.data[1] == 1) {
            tx_message.data[tx_message.length++] = rx_message.data[0] | 0x40;
            //TODO: Delay before reset?
            NVIC_SystemReset();
          } else {
            kline_send_error(&tx_message, rx_message.data[0], 0x12);
          }
        } else if(rx_message.data[0] == 0x1A) { //readEcuIdentification
          tx_message.data[tx_message.length++] = rx_message.data[0] | 0x40;
          tx_message.data[tx_message.length++] = rx_message.data[1];
          if(rx_message.data[1] == 0x80) { //ECUIdentificationDataTable
            string = "AutoECU";
          } else if(rx_message.data[1] == 0x90) { //VIN(Vehicle Identification Number)
            string = "VINNOTSET012345";
          } else if(rx_message.data[1] == 0x91) { //vehicleManufacturerECUHardwareNumber
            string = "AutoECU v0.1";
          } else if(rx_message.data[1] == 0x92) { //systemSupplierECUHardwareNumber
            string = "AutoECU v1.0";
          } else if(rx_message.data[1] == 0x94) { //systemSupplierECUSoftwareNumber
            string = "AutoECU v1.0";
          } else if(rx_message.data[1] == 0x97) { //systemNameOrEngineType
            string = "AutoECU v1.0";
          } else if(rx_message.data[1] == 0x98) { //repairShopCode
            string = "AutoECU v1.0";
          } else if(rx_message.data[1] == 0x99) { //ProgrammingDate
            string = "05.2022";
          } else if(rx_message.data[1] == 0x9A) { //vehicleManufacturerECUIdentifier
            string = "AutoECU v1.0";
          } else {
            kline_send_error(&tx_message, rx_message.data[0], 0x12);
            break;
          }

          if(string) {
            strcpy((char *)&tx_message.data[tx_message.length], string);
            tx_message.length += strlen(string);
          }
        } else if(rx_message.data[0] == 0x14) { //clearDiagnosticInformation
          tx_message.data[tx_message.length++] = rx_message.data[0] | 0x40;
          tx_message.data[tx_message.length++] = rx_message.data[1];
          tx_message.data[tx_message.length++] = rx_message.data[2];

          memset(&gStatus, 0, sizeof(gStatus));
          memset(&gCheckBitmap, 0, sizeof(gCheckBitmap));
          memset(&gEcuCriticalBackup.CheckBitmapRecorded, 0, sizeof(gEcuCriticalBackup.CheckBitmapRecorded));

        } else if(rx_message.data[0] == 0x18) { //readDTCByStatus
          tx_message.data[tx_message.length++] = rx_message.data[0] | 0x40;
          tx_message.data[tx_message.length++] = 0;
          for(int i = 0; i < CHECK_BITMAP_SIZE * 8 && tx_message.length < 200; i++) {
            if((gCheckBitmap[i >> 3] & (1 << (i & 7))) || (gEcuCriticalBackup.CheckBitmapRecorded[i >> 3] & (1 << (i & 7)))) {
              tx_message.data[1]++;
              tx_message.data[tx_message.length++] = (i >> 8) & 0xFF;
              tx_message.data[tx_message.length++] = i & 0xFF;
              tx_message.data[tx_message.length++] = 0xE0;
            }
          }

        } else if(rx_message.data[0] == 0x21) { //readDataByLocalIdentifier
          tx_message.data[tx_message.length++] = rx_message.data[0] | 0x40;
          tx_message.data[tx_message.length++] = rx_message.data[1];
          if(rx_message.data[1] == 0x01) { //afterSalesServiceRecordLocalIdentifier
            tx_message.data[tx_message.length++] = 0x39; //?????????? ???????????????????????? 1
            tx_message.data[tx_message.length++] = 0x18; //?????????? ???????????????????????? 2
            tx_message.data[tx_message.length++] = gDiagWorkingMode.Bytes.byte[0]; //?????????? ???????????? ???????????? 1
            tx_message.data[tx_message.length++] = gDiagWorkingMode.Bytes.byte[1]; //?????????? ???????????? ???????????? 2
            tx_message.data[tx_message.length++] = gDiagErrors.Bytes.byte[0]; //?????????? ???????????? ?????????????? ???????????????????????????? 1
            tx_message.data[tx_message.length++] = gDiagErrors.Bytes.byte[1]; //?????????? ???????????? ?????????????? ???????????????????????????? 2
            tx_message.data[tx_message.length++] = gDiagErrors.Bytes.byte[2]; //?????????? ???????????? ?????????????? ???????????????????????????? 3
            tx_message.data[tx_message.length++] = gDiagErrors.Bytes.byte[3]; //?????????? ???????????? ?????????????? ???????????????????????????? 4
            tx_message.data[tx_message.length++] = gParameters.EngineTemp <= -40 ? 0 : gParameters.EngineTemp + 40; //?????????????????????? ?????????????????????? ????????????????
            tx_message.data[tx_message.length++] = (gParameters.FuelRatio < 7.5f ? 7.5f : gParameters.FuelRatio > 21 ? 21 : gParameters.FuelRatio) / 14.7f * 256 - 128; //?????????????????????? ????????????/??????????????
            tx_message.data[tx_message.length++] = gParameters.ThrottlePosition > 100 ? 100 : gParameters.ThrottlePosition < 0 ? 0 : gParameters.ThrottlePosition; //?????????????????? ?????????????????????? ????????????????
            tx_message.data[tx_message.length++] = gParameters.RPM / 40.0f; //???????????????? ???????????????? ??????????????????
            tx_message.data[tx_message.length++] = gParameters.WishIdleRPM / 10.0f; //???????????????? ???????????????? ?????????????????? ???? ???????????????? ????????
            tx_message.data[tx_message.length++] = gParameters.WishIdleValvePosition > 255 ? 255 : gParameters.WishIdleValvePosition < 0 ? 0 : gParameters.WishIdleValvePosition; //???????????????? ?????????????????? ???????????????????? ?????????????????? ????????
            tx_message.data[tx_message.length++] = gParameters.IdleValvePosition > 255 ? 255 : gParameters.IdleValvePosition < 0 ? 0 : gParameters.IdleValvePosition; //?????????????? ?????????????????? ???????????????????? ?????????????????? ????????
            tx_message.data[tx_message.length++] = gParameters.LongTermCorrection + gParameters.ShortTermCorrection * 256.0f + 128; //?????????????????????? ?????????????????? ?????????????? ??????????????
            tx_message.data[tx_message.length++] = (int8_t)(gParameters.IgnitionAngle * 2.0f); //???????? ???????????????????? ??????????????????
            tx_message.data[tx_message.length++] = gParameters.Speed; //???????????????? ????????????????????
            tx_message.data[tx_message.length++] = ((gParameters.PowerVoltage > 17.5f ? 17.5f : gParameters.PowerVoltage < 5.5f ? 5.5f : gParameters.PowerVoltage) - 5.2f) * 20.0f; //???????????????????? ????????????????
            tx_message.data[tx_message.length++] = gParameters.WishIdleRPM / 10.0f; //???????????????? ?????????????? ?????????????????? ????????
            tx_message.data[tx_message.length++] = (gParameters.AdcLambdaUA > 4.9f ? 4.9f : gParameters.AdcLambdaUA) / 5.0f * 256.0f; //???????????????????? ???? ?????????????? ??????????????????
            tx_message.data[tx_message.length++] = gDiagWorkingMode.Bits.is_use_lambda * 0xC0; //?????????? ?????????????????? ?????????????? ??????????????????
            tx_message.data[tx_message.length++] = (uint32_t)(gParameters.InjectionPulse * 125.0f) & 0xFF; //???????????????????????? ???????????????? ?????????????? (?????????????? ????????)
            tx_message.data[tx_message.length++] = ((uint32_t)(gParameters.InjectionPulse * 125.0f) >> 8) & 0xFF; //???????????????????????? ???????????????? ?????????????? (?????????????? ????????)
            tx_message.data[tx_message.length++] = (uint32_t)(gParameters.MassAirFlow * 10.0f) & 0xFF; //???????????????? ???????????? ?????????????? (?????????????? ????????)
            tx_message.data[tx_message.length++] = ((uint32_t)(gParameters.MassAirFlow * 10.0f) >> 8) & 0xFF; //???????????????? ???????????? ?????????????? (?????????????? ????????)
            tx_message.data[tx_message.length++] = (uint32_t)(gParameters.CyclicAirFlow * 6.0f) & 0xFF; //???????????????? ???????????? ?????????????? (?????????????? ????????)
            tx_message.data[tx_message.length++] = ((uint32_t)(gParameters.CyclicAirFlow * 6.0f) >> 8) & 0xFF; //???????????????? ???????????? ?????????????? (?????????????? ????????)
            tx_message.data[tx_message.length++] = (uint32_t)(gParameters.FuelHourly * 50.0f) & 0xFF; //?????????????? ???????????? ?????????????? (?????????????? ????????)
            tx_message.data[tx_message.length++] = ((uint32_t)(gParameters.FuelHourly * 50.0f) >> 8) & 0xFF; //?????????????? ???????????? ?????????????? (?????????????? ????????)
            tx_message.data[tx_message.length++] = (uint32_t)(gParameters.FuelConsumption * 128.0f) & 0xFF; //?????????????? ???????????? ??????????????(?????????????? ????????)
            tx_message.data[tx_message.length++] = ((uint32_t)(gParameters.FuelConsumption * 128.0f) >> 8) & 0xFF; //?????????????? ???????????? ??????????????(?????????????? ????????)
            tx_message.data[tx_message.length++] = 0; //?????????????????????? ?????????? ?????? (?????????????? ????????)
            tx_message.data[tx_message.length++] = 0; //?????????????????????? ?????????? ?????? (?????????????? ????????)
          } else if(rx_message.data[1] == 0x02) { //endOfLineServiceRecordLocalIdentifier
            tx_message.data[tx_message.length++] = 0x39; //?????????? ???????????????????????? 1
            tx_message.data[tx_message.length++] = 0x18; //?????????? ???????????????????????? 2
            tx_message.data[tx_message.length++] = gDiagWorkingMode.Bytes.byte[0]; //?????????? ???????????? ???????????? 1
            tx_message.data[tx_message.length++] = gDiagWorkingMode.Bytes.byte[1]; //?????????? ???????????? ???????????? 2
            tx_message.data[tx_message.length++] = gParameters.EngineTemp <= -40 ? 0 : gParameters.EngineTemp + 40; //?????????????????????? ?????????????????????? ????????????????
            tx_message.data[tx_message.length++] = (gParameters.FuelRatio < 7.5f ? 7.5f : gParameters.FuelRatio > 21 ? 21 : gParameters.FuelRatio) / 14.7f * 256 - 128; //?????????????????????? ????????????/??????????????
            tx_message.data[tx_message.length++] = gParameters.ThrottlePosition > 100 ? 100 : gParameters.ThrottlePosition < 0 ? 0 : gParameters.ThrottlePosition; //?????????????????? ?????????????????????? ????????????????
            tx_message.data[tx_message.length++] = gParameters.RPM / 40.0f; //???????????????? ???????????????? ??????????????????
            tx_message.data[tx_message.length++] = gParameters.WishIdleRPM / 10.0f; //???????????????? ???????????????? ?????????????????? ???? ???????????????? ????????
            tx_message.data[tx_message.length++] = gParameters.IdleValvePosition > 255 ? 255 : gParameters.IdleValvePosition < 0 ? 0 : gParameters.IdleValvePosition; //?????????????? ?????????????????? ???????????????????? ?????????????????? ????????
            tx_message.data[tx_message.length++] = (int8_t)(gParameters.IgnitionAngle * 2.0f); //???????? ???????????????????? ??????????????????
            tx_message.data[tx_message.length++] = gParameters.Speed; //???????????????? ????????????????????
            tx_message.data[tx_message.length++] = ((gParameters.PowerVoltage > 17.5f ? 17.5f : gParameters.PowerVoltage < 5.5f ? 5.5f : gParameters.PowerVoltage) - 5.2f) * 20.0f; //???????????????????? ????????????????
            tx_message.data[tx_message.length++] = gDiagWorkingMode.Bits.is_use_lambda * 0xC0; //?????????? ?????????????????? ?????????????? ??????????????????
            tx_message.data[tx_message.length++] = (uint32_t)(gParameters.InjectionPulse * 125.0f) & 0xFF; //???????????????????????? ???????????????? ?????????????? (?????????????? ????????)
            tx_message.data[tx_message.length++] = ((uint32_t)(gParameters.InjectionPulse * 125.0f) >> 8) & 0xFF; //???????????????????????? ???????????????? ?????????????? (?????????????? ????????)
            tx_message.data[tx_message.length++] = (uint32_t)(gParameters.MassAirFlow * 10.0f) & 0xFF; //???????????????? ???????????? ?????????????? (?????????????? ????????)
            tx_message.data[tx_message.length++] = ((uint32_t)(gParameters.MassAirFlow * 10.0f) >> 8) & 0xFF; //???????????????? ???????????? ?????????????? (?????????????? ????????)
            tx_message.data[tx_message.length++] = (uint32_t)(gParameters.CyclicAirFlow * 6.0f) & 0xFF; //???????????????? ???????????? ?????????????? (?????????????? ????????)
            tx_message.data[tx_message.length++] = ((uint32_t)(gParameters.CyclicAirFlow * 6.0f) >> 8) & 0xFF; //???????????????? ???????????? ?????????????? (?????????????? ????????)
            tx_message.data[tx_message.length++] = (uint32_t)(gParameters.FuelHourly * 50.0f) & 0xFF; //?????????????? ???????????? ?????????????? (?????????????? ????????)
            tx_message.data[tx_message.length++] = ((uint32_t)(gParameters.FuelHourly * 50.0f) >> 8) & 0xFF; //?????????????? ???????????? ?????????????? (?????????????? ????????)
            tx_message.data[tx_message.length++] = (uint32_t)(gParameters.FuelConsumption * 128.0f) & 0xFF; //?????????????? ???????????? ??????????????(?????????????? ????????)
            tx_message.data[tx_message.length++] = ((uint32_t)(gParameters.FuelConsumption * 128.0f) >> 8) & 0xFF; //?????????????? ???????????? ??????????????(?????????????? ????????)
          } else if(rx_message.data[1] == 0x03) { //  factoryTestRecordLocalIdentifier
            tx_message.data[tx_message.length++] = (gParameters.KnockSensor > 4.95f) ? 255 : (gParameters.KnockSensor / 5.0f * 256);
            tx_message.data[tx_message.length++] = (gParameters.AdcEngineTemp > 4.95f) ? 255 : (gParameters.AdcEngineTemp / 5.0f * 256);
            tx_message.data[tx_message.length++] = (gParameters.AdcManifoldAirPressure > 4.95f) ? 255 : (gParameters.AdcManifoldAirPressure / 5.0f * 256);
            tx_message.data[tx_message.length++] = (gParameters.PowerVoltage > 4.95f) ? 255 : (gParameters.PowerVoltage / 5.0f * 256);
            tx_message.data[tx_message.length++] = (gParameters.AdcLambdaUA > 4.95f) ? 255 : (gParameters.AdcLambdaUA / 5.0f * 256);
            tx_message.data[tx_message.length++] = (gParameters.AdcThrottlePosition > 4.95f) ? 255 : (gParameters.AdcThrottlePosition / 5.0f * 256);
            tx_message.data[tx_message.length++] = 0;
            tx_message.data[tx_message.length++] = 0;
            tx_message.data[tx_message.length++] = 0;
            tx_message.data[tx_message.length++] = 0;
          } else {
            kline_send_error(&tx_message, rx_message.data[0], 0x12);
            break;
          }


        } else {
          kline_send_error(&tx_message, rx_message.data[0], 0x12);
        }

      } while(0);

      if(tx_message.length > 0) {
        kline_send(&tx_message);
        tester_last = now;
      }
    }
  }

  if(session_on && DelayDiff(now, tester_last) > 3000000) {
    if(session_on > 1) {
      kline_setbaud(10400);
      kline_start();
    }
    session_on = 0;
  }
}

static void ecu_oil_pressure_process(void)
{
  static GPIO_PinState prev = GPIO_PIN_RESET;
  static uint32_t pressure_last = 0;
#ifdef SIMULATION
  GPIO_PinState pressure = GPIO_PIN_SET;
#else
  GPIO_PinState pressure = sens_get_oil_pressure(NULL);
#endif
  uint32_t now = Delay_Tick;
  uint32_t hal_now = HAL_GetTick();
  uint8_t is_running = csps_isrunning();

  if(is_running) {
    if(!gStatus.OilPressure.is_running) {
      if(!gStatus.OilPressure.run_time)
        gStatus.OilPressure.run_time = now;

      if(DelayDiff(now, gStatus.OilPressure.run_time) > 1000000) {
        gStatus.OilPressure.is_running = 1;
      }
    } else {
      if(prev != pressure) {
        if(pressure) {
          if(DelayDiff(now, pressure_last) > 500000) {
            prev = pressure;
            pressure_last = now;
          }
        } else {
          prev = pressure;
          pressure_last = now;
        }
      } else {
        pressure_last = now;
      }
      gStatus.OilPressure.is_error = prev != GPIO_PIN_SET;
    }
  } else {
    pressure_last = now;
    gStatus.OilPressure.is_running = 0;
    gStatus.OilPressure.run_time = 0;
    gStatus.OilPressure.is_error = 0;
  }

  if(gStatus.OilPressure.is_error) {
    gStatus.OilPressure.error_time = HAL_DelayDiff(hal_now, gStatus.OilPressure.error_started);
  } else {
    gStatus.OilPressure.error_started = hal_now;
    gStatus.OilPressure.error_time = 0;
  }
}

static void ecu_battery_charge_process(void)
{
  static GPIO_PinState prev = GPIO_PIN_RESET;
  static uint32_t charge_last = 0;
  GPIO_PinState charge = sens_get_charge(NULL);
  uint32_t now = Delay_Tick;
  uint8_t is_running = csps_isrunning();

  if(is_running) {
    if(!gStatus.BatteryCharge.is_running) {
      if(!gStatus.BatteryCharge.run_time)
        gStatus.BatteryCharge.run_time = now;

      if(DelayDiff(now, gStatus.BatteryCharge.run_time) > 500000) {
        gStatus.BatteryCharge.is_running = 1;
      }
    } else {
      if(prev != charge) {
        if(charge) {
          if(DelayDiff(now, charge_last) > 500000) {
            prev = charge;
            charge_last = now;
          }
        } else {
          if(DelayDiff(now, charge_last) > 5000) {
            prev = charge;
            charge_last = now;
          }
        }
      } else {
        charge_last = now;
      }
      gStatus.BatteryCharge.is_error = prev != GPIO_PIN_SET;
    }
  } else {
    charge_last = now;
    gStatus.BatteryCharge.is_running = 0;
    gStatus.BatteryCharge.run_time = 0;
    gStatus.BatteryCharge.is_error = 0;
  }
}

void ecu_init(void)
{
  ecu_config_init();

  ecu_set_table(gEcuParams.startupTableNumber);

  ecu_pid_init();

  ecu_init_post_init();

  ecu_can_init();

  ecu_kline_init();

  memset(&gDiagWorkingMode, 0, sizeof(gDiagWorkingMode));
  memset(&gDiagErrors, 0, sizeof(gDiagErrors));

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
  ecu_config_process();
  ecu_update();
  ecu_idle_valve_process();
  ecu_backup_save_process();

  ecu_mem_process();
  ecu_fuelpump_process();
  ecu_fan_process();
  ecu_oil_pressure_process();
  ecu_battery_charge_process();

  ecu_drag_process();

#ifndef SIMULATION
  ecu_ign_process();
#endif
}

void ecu_loop(void)
{
  ecu_mem_loop();
  ecu_bluetooth_loop();
  ecu_diagnostic_loop();
  ecu_checkengine_loop();
  ecu_corrections_loop();
  ecu_can_loop();
  ecu_kline_loop();
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
      gParametersRequestFillLast = 0;
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

    case PK_IgnitionInjectionTestRequestID :
      PK_Copy(&PK_IgnitionInjectionTestRequest, msgBuf);

      gIITest.IgnitionEnabled = 0;
      gIITest.InjectionEnabled = 0;
      gIITest.StartedTime = 0;

      gIITest.IgnitionEnabled = PK_IgnitionInjectionTestRequest.IgnitionEnabled;
      gIITest.InjectionEnabled = PK_IgnitionInjectionTestRequest.InjectionEnabled;
      gIITest.Count = PK_IgnitionInjectionTestRequest.Count;
      gIITest.Period = PK_IgnitionInjectionTestRequest.Period;
      gIITest.IgnitionPulse = PK_IgnitionInjectionTestRequest.IgnitionPulse;
      gIITest.InjectionPulse = PK_IgnitionInjectionTestRequest.InjectionPulse;

      if(gIITest.IgnitionEnabled || gIITest.InjectionEnabled) {
        gIITest.StartedTime = Delay_Tick;
        if(gIITest.StartedTime == 0)
          gIITest.StartedTime++;
      }
      gIITest.CompletedCount = 0;

      PK_IgnitionInjectionTestResponse.ErrorCode = 0;
      PK_SendCommand(xChaSrc, &PK_IgnitionInjectionTestResponse, sizeof(PK_IgnitionInjectionTestResponse));
      break;

    default:
      break;
  }
}

static int8_t ecu_can_process_message(const sCanMessage *message)
{
  sCanMessage transmit = {0};
  int8_t status = 0;

  if(message->id == 0x100) { //Loopback
    if(message->rtr == CAN_RTR_REMOTE) {
      transmit.id = message->id + 0x10;
      transmit.rtr = CAN_RTR_DATA;
      transmit.length = message->length;
      memcpy(transmit.data.bytes, message->data.bytes, message->length);
    }
  } else if(message->id == 0x102) { //Parameter request
    if(message->rtr == CAN_RTR_REMOTE && message->length == 4) {
      if(message->data.dwords[0] < sizeof(gParameters) / sizeof(uint32_t)) {
        transmit.id = message->id + 0x10;
        transmit.rtr = CAN_RTR_DATA;
        transmit.length = 8;
        transmit.data.dwords[0] = message->data.dwords[0];
        transmit.data.dwords[1] = ((uint32_t *)&gParameters)[message->data.dwords[0]];

        if(OFFSETOF(sParameters, KnockSensor) / 4 == message->data.dwords[0] ||
            OFFSETOF(sParameters, KnockSensorFiltered) / 4 == message->data.dwords[0]) {
          gParametersRequestFillLast = 0;
        }
      }
    }
  } else if(message->id == 0x103) { //Table memory request
    if(message->rtr == CAN_RTR_REMOTE && message->length == 4) {
      if(message->data.dwords[0] < sizeof(gEcuTable) / sizeof(uint32_t)) {
        transmit.id = message->id + 0x10;
        transmit.rtr = CAN_RTR_DATA;
        transmit.length = 8;
        transmit.data.dwords[0] = message->data.dwords[0];
        transmit.data.dwords[1] = ((uint32_t *)&gEcuTable[0])[message->data.dwords[0]];
      }
    }
  } else if(message->id == 0x104) { //Config memory request
    if(message->rtr == CAN_RTR_REMOTE && message->length == 4) {
      if(message->data.dwords[0] < sizeof(gEcuParams) / sizeof(uint32_t)) {
        transmit.id = message->id + 0x10;
        transmit.rtr = CAN_RTR_DATA;
        transmit.length = 8;
        transmit.data.dwords[0] = message->data.dwords[0];
        transmit.data.dwords[1] = ((uint32_t *)&gEcuParams)[message->data.dwords[0]];
      }
    }
  }

  if(transmit.id > 0 || transmit.length > 0) {
    status = can_send(&transmit);
  }

  return status;
}

void ecu_hardfault_handle(void)
{
  gEcuCriticalBackup.CheckBitmapRecorded[CheckHardFaultException >> 3] |= 1 << (CheckHardFaultException & 7);
  config_save_critical_backup(&gEcuCriticalBackup);
}

