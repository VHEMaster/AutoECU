/*
 * sensors.c
 *
 *  Created on: Mar 5, 2022
 *      Author: VHEMaster
 */

#include "sensors.h"
#include "misc.h"
#include "adc.h"
#include "interpolation.h"
#include "defines.h"
#include "delay.h"

typedef struct {
  GPIO_TypeDef *port;
  uint32_t time_switched;
  volatile GPIO_PinState state;
  uint16_t pin;
  uint8_t inverted;
}sSensor;

static sSensor Sensors[SensorCount] = {{0}};

void sensors_init(void)
{

}

HAL_StatusTypeDef sensors_register(eSensor sensor, GPIO_TypeDef *port, uint16_t pin, uint8_t inverted)
{
  if(sensor < SensorCount && port && pin) {
    Sensors[sensor].port = port;
    Sensors[sensor].pin = pin;
    Sensors[sensor].inverted = inverted;

    Sensors[sensor].state = ((port->IDR & pin) != GPIO_PIN_RESET) ? !inverted : inverted;
    return HAL_OK;
  }
  return HAL_ERROR;

}

inline void sensors_loop(void)
{
  GPIO_PinState pin_state;
  for(int i = 0; i < SensorCount; i++) {
    if(Sensors[i].port && Sensors[i].pin) {
      pin_state = ((Sensors[i].port->IDR & Sensors[i].pin) != GPIO_PIN_RESET) ? (!Sensors[i].inverted) : (Sensors[i].inverted);
      if(Sensors[i].state != pin_state) {
        Sensors[i].time_switched = Delay_Tick;
        Sensors[i].state = pin_state;
      }
    }
  }
}

inline GPIO_PinState sens_get_oil_pressure(uint32_t *time)
{
  if(time)
    *time = DelayDiff(Delay_Tick, Sensors[SensorOilPressure].time_switched);
  return Sensors[SensorOilPressure].state;
}

inline GPIO_PinState sens_get_starter(uint32_t *time)
{
  if(time)
    *time = DelayDiff(Delay_Tick, Sensors[SensorStarter].time_switched);
  return Sensors[SensorStarter].state;
}

inline GPIO_PinState sens_get_handbrake(uint32_t *time)
{
  if(time)
    *time = DelayDiff(Delay_Tick, Sensors[SensorHandbrake].time_switched);
  return Sensors[SensorHandbrake].state;
}

inline GPIO_PinState sens_get_charge(uint32_t *time)
{
  if(time)
    *time = DelayDiff(Delay_Tick, Sensors[SensorCharge].time_switched);
  return Sensors[SensorCharge].state;
}

inline GPIO_PinState sens_get_rsvd1(uint32_t *time)
{
  if(time)
    *time = DelayDiff(Delay_Tick, Sensors[SensorRsvd1].time_switched);
  return Sensors[SensorRsvd1].state;
}

inline GPIO_PinState sens_get_rsvd2(uint32_t *time)
{
  if(time)
    *time = DelayDiff(Delay_Tick, Sensors[SensorRsvd2].time_switched);
  return Sensors[SensorRsvd2].state;
}

static float getTemperatureByResistance_airtemp(float resistance)
{
  const static float resistances[19] = {71,89,113,144,187,243,323,436,596,834,1175,1707,2500,3792,5896,9397,15462,26114,45313};
  const static float temperatures[19] = {140,130,120,110,100,90,80,70,60,50,40,30,20,10,0,-10,-20,-30,-40};
  sMathInterpolateInput ipResistance = math_interpolate_input(resistance, resistances, ITEMSOF(resistances));

  return math_interpolate_1d(ipResistance, temperatures);
}

static float getTemperatureByResistance_enginetemp(float resistance)
{
  const static float resistances[19] = {80,177,241,332,467,667,973,1188,1459,1802,2238,2796,3520,4450,5670,7280,9420,15000,30000};
  const static float temperatures[19] = {128,100,90,80,70,60,50,45,40,35,30,25,20,15,10,5,0,-20,-40};
  sMathInterpolateInput ipResistance = math_interpolate_input(resistance, resistances, ITEMSOF(resistances));

  return math_interpolate_1d(ipResistance, temperatures);
}

HAL_StatusTypeDef sens_get_o2_fuelratio(float *output, uint8_t *valid)
{
  HAL_StatusTypeDef status = HAL_OK;
  sO2Status o2status = Misc_O2_GetStatus();
  *output = o2status.FuelRatio;
  if(o2status.Available) {
    if(valid)
      *valid = o2status.Valid && o2status.Working;
  } else {
    if(valid)
      *valid = 0;
    status = HAL_ERROR;
  }
  return status;
}

HAL_StatusTypeDef sens_get_o2_diagnostic(sO2Diagnostic *output)
{
  HAL_StatusTypeDef status = HAL_OK;
  sO2Status o2status = Misc_O2_GetStatus();
  *output = o2status.Diag.Fields;
  status = o2status.Available == 0 ? HAL_ERROR : HAL_OK;

  return status;
}

HAL_StatusTypeDef sens_get_adc_status(void)
{
  return ADC_GetStatus();
}

HAL_StatusTypeDef sens_get_knock_raw(float *output)
{
  return Misc_GetKnockValueRaw(output);
}

HAL_StatusTypeDef sens_get_knock(float *output)
{
  return Misc_GetKnockValueByRPM(output);
}

HAL_StatusTypeDef sens_get_map(float *output)
{
  HAL_StatusTypeDef status = HAL_OK;
  float voltage = ADC_GetVoltage(AdcChManifoldAbsolutePressure);
  float result = voltage * 19000.0f + 10000.0f;
  *output = result;
  return status;
}

HAL_StatusTypeDef sens_get_air_temperature(float *output)
{
  HAL_StatusTypeDef status = HAL_OK;
  float reference_resistance, meter_resistance;
  float power_voltage = ADC_GetVoltage(AdcMcuChReferenceVoltage);
  float temperature = ADC_GetVoltage(AdcChAirTemperature);

  *output = 150.0f;

  if(power_voltage != 0.0f) {
    if(temperature > power_voltage)
      temperature = power_voltage;

    reference_resistance = 1000.0f;
    meter_resistance = (reference_resistance / (1.0f - (temperature/power_voltage))) - reference_resistance;
    temperature = getTemperatureByResistance_airtemp(meter_resistance);
    if(temperature < -40.0f) {
      *output = -40.0f;
      status = HAL_ERROR;
    }
    else if(temperature > 150.0f) {
      *output = 150.0f;
      status = HAL_ERROR;
    }
    else
      *output = temperature;
  }
  else
    status = HAL_ERROR;

  return status;
}

HAL_StatusTypeDef sens_get_engine_temperature(float *output)
{
  HAL_StatusTypeDef status = HAL_OK;
  float reference_resistance, meter_resistance;
  float power_voltage = ADC_GetVoltage(AdcChPowerVoltage);
  float temperature = ADC_GetVoltage(AdcChEngineTemperature);

  *output = 150.0f;

  if(power_voltage != 0.0f) {
    if(temperature > power_voltage)
      temperature = power_voltage;

    reference_resistance = 1000.0f;
    meter_resistance = (reference_resistance / (1.0f - (temperature/power_voltage))) - reference_resistance;
    temperature = getTemperatureByResistance_enginetemp(meter_resistance);
    if(temperature < -40.0f) {
      *output = -40.0f;
      status = HAL_ERROR;
    }
    else if(temperature > 150.0f) {
      *output = 150.0f;
      status = HAL_ERROR;
    }
    else
      *output = temperature;
  }
  else
    status = HAL_ERROR;

  return status;
}

HAL_StatusTypeDef sens_get_throttle_position(float *output)
{
  HAL_StatusTypeDef status = HAL_OK;
  float power_voltage = ADC_GetVoltage(AdcMcuChReferenceVoltage);
  float value = ADC_GetVoltage(AdcChEngineTemperature);
  //TODO: calibrate throttle position sensor
  float voltage_from = power_voltage * 0.14f;
  float voltage_to = power_voltage * 0.8f;

  if(value + 0.2f < voltage_from || value - 0.2f > voltage_to) {
    status = HAL_ERROR;
  }

  if(value < voltage_from)
    value = voltage_from;
  if(value > voltage_to)
    value = voltage_to;

  value -= voltage_from;
  value /= voltage_to - voltage_from;
  value *= 100.0f;

  *output = value;

  return status;
}

HAL_StatusTypeDef sens_get_power_voltage(float *output)
{
  HAL_StatusTypeDef status = HAL_OK;
  float power_voltage = ADC_GetVoltage(AdcChPowerVoltage);

  *output = power_voltage;

  return status;
}

HAL_StatusTypeDef sens_get_reference_voltage(float *output)
{
  HAL_StatusTypeDef status = HAL_OK;
  float reference_voltage = ADC_GetVoltage(AdcMcuChReferenceVoltage);

  *output = reference_voltage;

  return status;
}
