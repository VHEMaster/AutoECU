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

inline GPIO_PinState sens_get_clutch(uint32_t *time)
{
  if(time)
    *time = DelayDiff(Delay_Tick, Sensors[SensorClutch].time_switched);
  return Sensors[SensorClutch].state;
}

inline GPIO_PinState sens_get_ign(uint32_t *time)
{
  if(time)
    *time = DelayDiff(Delay_Tick, Sensors[SensorIgn].time_switched);
  return Sensors[SensorIgn].state;
}

static float getTemperatureByResistance_airtemp(float resistance)
{
  //More here: http://www.vems.hu/wiki/index.php?page=EasyTherm%2FSensorTable
  const static float resistances[31] = {46.88f,57.41f,71.20f,89.30f,112.7f,144.2f,186.6f,243.2f,322.5f,435.7f,595.5f,702.8f,833.9f,987.6f,1175,1412,1707,2057,2500,3069,3792,4712,5896,7415,9397,12002,15462,20003,26114,34281,45313};
  const static float temperatures[31] = {160,150,140,130,120,110,100,90,80,70,60,55,50,45,40,35,30,25,20,15,10,5,0,-5,-10,-15,-20,-25,-30,-35,-40};
  sMathInterpolateInput ipResistance = math_interpolate_input(resistance, resistances, ITEMSOF(resistances));

  return math_interpolate_1d(ipResistance, temperatures);
}

static float getTemperatureByResistance_enginetemp(float resistance)
{
  const static float resistances[24] = {80,102,133,177,241,332,467,667,973,1188,1459,2238,2796,3520,4450,5670,7280,9420,12300,16180,21450,28680,52700,100700};
  const static float temperatures[24] = {128,120,110,100,90,80,70,60,50,45,40,30,25,20,15,10,5,0,-5,-10,-15,-20,-30,-40};
  sMathInterpolateInput ipResistance = math_interpolate_input(resistance, resistances, ITEMSOF(resistances));

  return math_interpolate_1d(ipResistance, temperatures);
}

HAL_StatusTypeDef sens_get_o2_labmda(float *output, uint8_t *valid)
{
  HAL_StatusTypeDef status = HAL_OK;
  sO2Status o2status = Misc_O2_GetStatus();
  *output = o2status.Lambda;
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

HAL_StatusTypeDef sens_get_o2_temperature(float *output)
{
  HAL_StatusTypeDef status = HAL_OK;
  sO2Status o2status = Misc_O2_GetStatus();
  *output = o2status.Temperature;
  if(!o2status.Available) {
    status = HAL_ERROR;
  }
  return status;
}

HAL_StatusTypeDef sens_get_o2_heatervoltage(float *output)
{
  HAL_StatusTypeDef status = HAL_OK;
  sO2Status o2status = Misc_O2_GetStatus();
  *output = o2status.HeaterVoltage;
  if(!o2status.Available) {
    status = HAL_ERROR;
  }
  return status;
}

HAL_StatusTypeDef sens_get_o2_temperaturevoltage(float *output)
{
  HAL_StatusTypeDef status = HAL_OK;
  sO2Status o2status = Misc_O2_GetStatus();
  *output = o2status.TemperatureVoltage;
  if(!o2status.Available) {
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
  return Knock_GetValueRaw(output);
}

HAL_StatusTypeDef sens_get_knock(float *output)
{
  return Knock_GetValueByRPM(output);
}

HAL_StatusTypeDef sens_get_map(float *output)
{
  HAL_StatusTypeDef status = HAL_OK;
  float voltage = ADC_GetVoltage(AdcChManifoldAbsolutePressure);
  float power_voltage = ADC_GetVoltage(AdcMcuChReferenceVoltage);
  if(voltage >= power_voltage * 0.985f)
    status = HAL_ERROR;

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

    reference_resistance = 2700.0f;
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

    reference_resistance = 2700.0f;
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
  float voltage_from = power_voltage * 0.12f;
  float voltage_to = power_voltage * 0.92f;

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
