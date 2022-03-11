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
#include "config.h"

#include <string.h>
#include "arm_math.h"

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
            HAL_StatusTypeDef Load : 2;
            HAL_StatusTypeDef Save : 2;
            HAL_StatusTypeDef Init : 2;
        }Struct;
        uint8_t Byte;
    }Bkpsram;
}sStatus;

static sEcuTable gEcuTable[TABLE_SETUPS_MAX];
static sEcuParams gEcuParams;
static sEcuCorrections gEcuCorrections;
static sEcuCriticalBackup gEcuCriticalBackup;
static sStatus gStatus = {{{0}}};
static sParameters gParameters = {0};
static sForceParameters gForceParameters = {0};

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

static void ecu_update()
{
  uint32_t now = Delay_Tick;
  uint32_t table_number = gParameters.CurrentTable;
  sEcuTable *table = &gEcuTable[table_number];
  static sMathInterpolateInput ipRpm = {0};
  static sMathInterpolateInput ipMap = {0};
  static sMathInterpolateInput ipTemp = {0};
  static sMathInterpolateInput ipSpeed = {0};
  static sMathInterpolateInput ipThrottle = {0};
  static sMathInterpolateInput ipVoltages = {0};

  static sMathInterpolateInput ipFilling = {0};

  static float rpm;
  static float map;
  static float fuel_ratio;
  static float air_temp;
  static float engine_temp;
  static float throttle;
  static float power_voltage;
  static float reference_voltage;
  static float speed;
  static float knock;
  static float idle_valve_position;
  static float knock_raw;

  static float wish_fuel_ratio;
  static float filling_map;
  static float filling_thr;
  static float filling_volume;
  static float ignition_angle;
  static float ignition_time;
  static float injector_lag;
  static float cycle_air_flow;
  static float mass_air_flow;
  static float injection_time;
  static float injection_phase;
  static float air_destiny;
  static float fuel_flow_per_us;
  static float knock_filtered;
  static float knock_noise_level;
  static float engine_load;

  static float fill_proportion;
  static float fuel_pressure;
  static float fuel_abs_pressure;

  static float idle_wish_rpm;
  static float idle_wish_massair;
  static float idle_wish_ignition;
  static float idle_rpm_shift;
  static float idle_wish_valve_pos;
  static float injection_dutycycle;

  static uint8_t running;
  static uint8_t idle_flag;

  running = csps_isrunning();
  fill_proportion = table->fill_proportion_map_vs_thr;
  fuel_pressure = table->fuel_pressure;

  rpm = csps_getrpm();
  speed = speed_getspeed();
  sens_get_map(&map);
  sens_get_knock(&knock);
  sens_get_knock_raw(&knock_raw);
  sens_get_o2_fuelratio(&fuel_ratio, NULL);
  sens_get_air_temperature(&air_temp);
  sens_get_engine_temperature(&engine_temp);
  sens_get_throttle_position(&throttle);
  sens_get_reference_voltage(&reference_voltage);
  sens_get_power_voltage(&power_voltage);
  idle_valve_position = out_get_idle_valve();

  idle_flag = throttle < 0.2f;

  fuel_abs_pressure = fuel_pressure + (1.0f - map * 0.00001f);
  fuel_flow_per_us = table->injector_performance * 1.66666667e-8f * table->fuel_mass_per_cc; // perf / 60.000.000
  fuel_flow_per_us *= fuel_abs_pressure / fuel_pressure;

  speed *= gEcuParams.speedCorrection;

  ipRpm = math_interpolate_input(rpm, table->rotates, table->rotates_count);
  ipMap = math_interpolate_input(map, table->pressures, table->pressures_count);
  ipTemp = math_interpolate_input(engine_temp, table->engine_temps, table->engine_temp_count);
  ipSpeed = math_interpolate_input(speed, table->idle_rpm_shift_speeds, table->idle_speeds_shift_count);
  ipThrottle = math_interpolate_input(throttle, table->throttles, table->throttles_count);
  ipVoltages = math_interpolate_input(power_voltage, table->voltages, table->voltages_count);

  filling_map = math_interpolate_2d(ipRpm, ipMap, TABLE_ROTATES_MAX, table->fill_by_map);
  filling_thr = math_interpolate_2d(ipRpm, ipThrottle, TABLE_ROTATES_MAX, table->fill_by_thr);

  filling_map *= math_interpolate_2d_int8(ipRpm, ipMap, TABLE_ROTATES_MAX, gEcuCorrections.fill_by_map) * 0.01f + 1.0f;
  filling_thr *= math_interpolate_2d_int8(ipRpm, ipThrottle, TABLE_ROTATES_MAX, gEcuCorrections.fill_by_thr) * 0.01f + 1.0f;

  filling_volume = filling_map * (fill_proportion) + filling_thr * (1.0f - fill_proportion);

  air_destiny = ecu_get_air_destiny(map, air_temp);
  cycle_air_flow = filling_volume * air_destiny;
  mass_air_flow = rpm * 0.03333333f * cycle_air_flow; // rpm / 60 * 2

  ipFilling = math_interpolate_input(cycle_air_flow, table->fillings, table->fillings_count);

  ignition_angle = math_interpolate_2d(ipRpm, ipFilling, TABLE_ROTATES_MAX, table->ignitions);
  ignition_angle += math_interpolate_2d_int8(ipRpm, ipFilling, TABLE_ROTATES_MAX, gEcuCorrections.ignitions) * 0.1f;

  wish_fuel_ratio = math_interpolate_2d(ipRpm, ipFilling, TABLE_ROTATES_MAX, table->fuel_mixtures);
  injection_phase = math_interpolate_2d(ipRpm, ipFilling, TABLE_ROTATES_MAX, table->injection_phase);

  injector_lag = math_interpolate_1d(ipVoltages, table->injector_lag);
  ignition_time = math_interpolate_1d(ipVoltages, table->ignition_time);
  ignition_time *= math_interpolate_1d(ipRpm, table->ignition_time_rpm_mult);

  injection_time = cycle_air_flow / wish_fuel_ratio / fuel_flow_per_us;
  injection_time += injector_lag;

  idle_wish_rpm = math_interpolate_1d(ipTemp, table->idle_wish_rotates);
  idle_wish_massair = math_interpolate_1d(ipTemp, table->idle_wish_massair);
  idle_wish_ignition = math_interpolate_1d(ipRpm, table->idle_wish_ignition);

  idle_rpm_shift = math_interpolate_1d(ipSpeed, table->idle_rpm_shift);
  knock_noise_level = math_interpolate_1d(ipRpm, table->knock_noise_level);

  idle_wish_rpm += idle_rpm_shift;

  //TODO!!!!
  idle_wish_valve_pos = 0.0f;
  injection_dutycycle = 0.0f;

  out_set_idle_valve(idle_wish_valve_pos);

  knock_filtered = knock - knock_noise_level;
  if(knock_filtered < 0.0f)
    knock_filtered = 0.0f;

  if(idle_flag && running) {
    if(out_get_fan() != GPIO_PIN_RESET) {
      ignition_angle += table->idle_ign_fan_corr;
    }
  }

  if(!running) {
    ignition_angle = table->ignition_initial;
    if(throttle >= 75.0f)
      injection_time = 0;
  }

  engine_load = filling_volume / gEcuParams.engineVolume * 4.0f;


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
  gParameters.CyclicFilling = filling_volume;
  gParameters.EngineLoad = engine_load;
  gParameters.WishFuelRatio = wish_fuel_ratio;
  gParameters.IdleValvePosition = idle_valve_position;
  gParameters.WishIdleRPM = idle_wish_rpm;
  gParameters.WishIdleMassAirFlow = idle_wish_massair;
  gParameters.WishIdleValvePosition = idle_wish_valve_pos;
  gParameters.WishIdleIgnitionAngle = idle_wish_ignition;
  gParameters.IgnitionAngle = ignition_angle;
  gParameters.InjectionPhase = injection_phase;
  gParameters.IgnitionTime = injection_time;
  gParameters.IgnitionDutyCycle = injection_dutycycle;
  gParameters.IdleSpeedShift = idle_rpm_shift;

  gParameters.OilSensor = sens_get_oil_pressure();
  gParameters.StarterSensor = sens_get_starter();
  gParameters.HandbrakeSensor = sens_get_handbrake();
  gParameters.ChargeSensor = sens_get_charge();
  gParameters.Rsvd1Sensor = sens_get_rsvd1();
  gParameters.Rsvd2Sensor = sens_get_rsvd2();

  gParameters.FuelPumpRelay = out_get_fuelpump() != GPIO_PIN_RESET;
  gParameters.FanRelay = out_get_fan() != GPIO_PIN_RESET;
  gParameters.CheckEngine = out_get_checkengine() != GPIO_PIN_RESET;
  gParameters.StarterRelay = out_get_starter() != GPIO_PIN_RESET;
  gParameters.Rsvd1Output = out_get_rsvd1() != GPIO_PIN_RESET;
  gParameters.Rsvd2Output = out_get_rsvd2() != GPIO_PIN_RESET;


}

void ecu_init(void)
{
  ecu_config_init();

  ecu_set_start_allowed(HAL_OK);
  ecu_set_table(0);

}

void ecu_irq_fast_loop(void)
{

}

void ecu_irq_slow_loop(void)
{
  ecu_update_current_table();
  ecu_update();

}

void ecu_loop(void)
{

}

void ecu_parse_command(eTransChannels xChaSrc, uint8_t * msgBuf, uint32_t length)
{

}
