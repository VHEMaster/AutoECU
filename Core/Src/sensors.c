/*
 * sensors.c
 *
 *  Created on: Mar 5, 2022
 *      Author: VHEMaster
 */

#include "sensors.h"
#include "misc.h"
#include "adc.h"

static float getTemperatureByResistance(float resistance)
{
  const float resistances[22] = {100700,52700,28680,21450,16180,12300,9420,7280,5670,4450,3520,2796,2238,1802,1459,1188,973,667,467,332,241,177};
  const float temperatures[22] = {-40,-30,-20,-15,-10,-5,0,5,10,15,20,25,30,35,40,45,50,60,70,80,90,100};
  float result = 0.0f;
  uint8_t index1, index2;
  float temp1 = 0.0f, temp2 = 0.0f, mult, tempt1, tempt2;

  if(resistance >= resistances[0])
  {
    index1 = 0;
    index2 = 1;
    temp1 = resistances[index1];
    temp2 = resistances[index2];
  }
  else if(resistance <= resistances[(sizeof(resistances) / sizeof(float)) - 1])
  {
    index1 = (sizeof(resistances) / sizeof(float)) - 2;
    index2 = (sizeof(resistances) / sizeof(float)) - 1;
    temp1 = resistances[index1];
    temp2 = resistances[index2];
  }
  else
  {
    for(int i = 1; i < (sizeof(resistances) / sizeof(float)); i++)
    {
      temp1 = resistances[i-1];
      temp2 = resistances[i];
      if(temp1 > resistance && temp2 < resistance)
      {
        index1 = i-1;
        index2 = i;
        break;
      }
      temp1 = 0.0f;
      temp2 = 0.0f;
    }
  }

  if(temp1 != 0.0f || temp2 != 0.0f)
  {
    tempt1 = temperatures[index1];
    tempt2 = temperatures[index2];
    if(temp2 != temp1)
    {
      mult = (resistance - temp1) / (temp2 - temp1);
      result = (tempt2 - tempt1) * mult + tempt1;
    }
    else result = (tempt1 + tempt2) / 2.0f;
  }
  return result;
}

HAL_StatusTypeDef sens_get_o2_fuelratio(float *output, uint8_t *valid)
{
  HAL_StatusTypeDef status = HAL_OK;
  sO2Status o2status = Misc_O2_GetStatus();
  *output = o2status.FuelRatio;
  if(o2status.Working) {
    *valid = o2status.Valid;
  } else {
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

  return status;
}

HAL_StatusTypeDef sens_get_knock(float *output)
{
  HAL_StatusTypeDef status = HAL_OK;
  //TODO: to define the way to output it
  return status;
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

  if(temperature > power_voltage)
    temperature = power_voltage;

  reference_resistance = 1000.0f;
  meter_resistance = (reference_resistance / (1.0f - (temperature/power_voltage))) - reference_resistance;
  temperature = getTemperatureByResistance(meter_resistance);
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

  return status;
}

HAL_StatusTypeDef sens_get_engine_temperature(float *output)
{
  HAL_StatusTypeDef status = HAL_OK;
  float reference_resistance, meter_resistance;
  float power_voltage = ADC_GetVoltage(AdcChPowerVoltage);
  float temperature = ADC_GetVoltage(AdcChEngineTemperature);

  if(temperature > power_voltage)
    temperature = power_voltage;

  reference_resistance = 200.0f;
  meter_resistance = (reference_resistance / (1.0f - (temperature/power_voltage))) - reference_resistance;
  temperature = getTemperatureByResistance(meter_resistance);
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
