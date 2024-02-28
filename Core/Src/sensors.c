/*
 * sensors.c
 *
 *  Created on: Mar 5, 2022
 *      Author: VHEMaster
 */

#include "interpolation.h"
#include "sensors.h"
#include "misc.h"
#include "adc.h"
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

#define SENS_ERROR_DELAY 300000

static volatile float gSensTpsMin = 0.200f;
static volatile float gSensTpsMax = 4.800f;

static volatile float gSensMapGain = 19000.0f;
static volatile float gSensMapOffset = 10000.0f;

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

inline GPIO_PinState sens_get_fan_force_switch(uint32_t *time)
{
  if(time)
    *time = DelayDiff(Delay_Tick, Sensors[SensorFanForceSwitch].time_switched);
  return Sensors[SensorFanForceSwitch].state;
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

void sens_configure_map(float gain, float offset)
{
  gSensMapGain = gain;
  gSensMapOffset = offset;
}

void sens_configure_tps(float min, float max)
{
  gSensTpsMin = min;
  gSensTpsMax = max;
}

STATIC_INLINE HAL_StatusTypeDef getMapPressureByVoltages(float map, float ref, float *pressure)
{
  HAL_StatusTypeDef ret = HAL_OK;

  *pressure = map * gSensMapGain + gSensMapOffset;

  if(map < 0.02f) {
    ret = HAL_ERROR;
  }

  return ret;
}

static float getTemperatureByResistance_airtemp(float resistance)
{
  //More here: http://www.vems.hu/wiki/index.php?page=EasyTherm%2FSensorTable
  const static float resistances[28] = {89.30f,112.7f,144.2f,186.6f,243.2f,322.5f,435.7f,595.5f,702.8f,833.9f,987.6f,1175,1412,1707,2057,2500,3069,3792,4712,5896,7415,9397,12002,15462,20003,26114,34281,45313};
  const static float temperatures[28] = {130,120,110,100,90,80,70,60,55,50,45,40,35,30,25,20,15,10,5,0,-5,-10,-15,-20,-25,-30,-35,-40};
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

INLINE sO2Status sens_get_o2_status(void)
{
  sO2Status o2status = Misc_O2_GetStatus();

  return o2status;
}

INLINE HAL_StatusTypeDef sens_set_o2_lpf(float lpf)
{
  HAL_StatusTypeDef o2status = Misc_O2_SetLpf(lpf);

  return o2status;
}

INLINE HAL_StatusTypeDef sens_get_o2_labmda(const sO2Status *p_status, float *output, uint8_t *valid)
{
  HAL_StatusTypeDef status = HAL_OK;

  if(!p_status)
    return HAL_ERROR;

  *output = p_status->Lambda;

#ifdef DEBUG
    if(*output > 1.09f) {
      HAL_GPIO_WritePin(MCU_RSVD_4_GPIO_Port, MCU_RSVD_4_Pin, GPIO_PIN_SET);
    } else if(*output < 0.9f) {
      HAL_GPIO_WritePin(MCU_RSVD_4_GPIO_Port, MCU_RSVD_4_Pin, GPIO_PIN_RESET);
    }
#endif
    /*
#ifdef DEBUG
    if(*output > 1.09f) {
      HAL_GPIO_WritePin(MCU_RSVD_3_GPIO_Port, MCU_RSVD_3_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(MCU_RSVD_4_GPIO_Port, MCU_RSVD_4_Pin, GPIO_PIN_SET);
    } else if(*output > 1.05f) {
      HAL_GPIO_WritePin(MCU_RSVD_3_GPIO_Port, MCU_RSVD_3_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MCU_RSVD_4_GPIO_Port, MCU_RSVD_4_Pin, GPIO_PIN_SET);
    } else if(*output < 0.9f) {
      HAL_GPIO_WritePin(MCU_RSVD_3_GPIO_Port, MCU_RSVD_3_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MCU_RSVD_4_GPIO_Port, MCU_RSVD_4_Pin, GPIO_PIN_RESET);
    } else if(*output < 0.95f) {
      HAL_GPIO_WritePin(MCU_RSVD_3_GPIO_Port, MCU_RSVD_3_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(MCU_RSVD_4_GPIO_Port, MCU_RSVD_4_Pin, GPIO_PIN_RESET);
    }
#endif
  */

  if(p_status->Available) {
    if(valid)
      *valid = p_status->Valid && p_status->Working;
  } else {
    if(valid)
      *valid = 0;
    status = HAL_ERROR;
  }
  return status;
}

INLINE HAL_StatusTypeDef sens_get_o2_temperature(const sO2Status *p_status, float *output)
{
  HAL_StatusTypeDef status = HAL_OK;

  if(!p_status)
    return HAL_ERROR;

  *output = p_status->Temperature;
  if(!p_status->Available) {
    status = HAL_ERROR;
  }
  return status;
}

INLINE HAL_StatusTypeDef sens_get_o2_heatervoltage(const sO2Status *p_status, float *output)
{
  HAL_StatusTypeDef status = HAL_OK;

  if(!p_status)
    return HAL_ERROR;

  *output = p_status->HeaterVoltage;
  if(!p_status->Available) {
    status = HAL_ERROR;
  }
  return status;
}

INLINE HAL_StatusTypeDef sens_get_o2_temperaturevoltage(const sO2Status *p_status, float *output)
{
  HAL_StatusTypeDef status = HAL_OK;

  if(!p_status)
    return HAL_ERROR;

  *output = p_status->TemperatureVoltage;
  if(!p_status->Available) {
    status = HAL_ERROR;
  }
  return status;
}

INLINE HAL_StatusTypeDef sens_get_o2_diagnostic(const sO2Status *p_status, sO2Diagnostic *output)
{
  HAL_StatusTypeDef status = HAL_OK;

  if(!p_status)
    return HAL_ERROR;

  *output = p_status->Diag.Fields;
  status = p_status->Available == 0 ? HAL_ERROR : HAL_OK;

  return status;
}

INLINE HAL_StatusTypeDef sens_get_adc_status(void)
{
  return adc_get_status();
}

STATIC_INLINE HAL_StatusTypeDef sens_get_map_internal(float *output, float voltage, float power_voltage)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t now = Delay_Tick;
  static uint32_t last_ok = 0;
  static HAL_StatusTypeDef status_old = HAL_OK;
  static float result_old = 103000.0f;
  float result = result_old;

  status = getMapPressureByVoltages(voltage, power_voltage, &result);

  if(status == HAL_OK) {
    status_old = HAL_OK;
    last_ok = now;
    result_old = result;
    *output = result;
#ifdef DEBUG
    if(result > 80000)
      HAL_GPIO_WritePin(MCU_RSVD_1_GPIO_Port, MCU_RSVD_1_Pin, GPIO_PIN_SET);
    else if(result < 60000)
      HAL_GPIO_WritePin(MCU_RSVD_1_GPIO_Port, MCU_RSVD_1_Pin, GPIO_PIN_RESET);
#endif
  } else if(status_old == HAL_OK) {
    *output = result_old;
    status_old = HAL_OK;
    if(DelayDiff(now, last_ok) > SENS_ERROR_DELAY) {
      status_old = HAL_ERROR;
    }
  } else {
    status_old = HAL_ERROR;
    *output = result;
  }


  return status_old;
}

INLINE HAL_StatusTypeDef sens_set_map_lpf(float map_lpf)
{
  return adc_set_lpf(AdcChManifoldAbsolutePressure, map_lpf);
}

INLINE HAL_StatusTypeDef sens_reset_map_lpf(void)
{
  return adc_reset_lpf_state(AdcChManifoldAbsolutePressure);
}

HAL_StatusTypeDef sens_get_map(float *output)
{
  float voltage = adc_get_voltage(AdcChManifoldAbsolutePressure);
  float power_voltage = adc_get_voltage(AdcMcuChReferenceVoltage);

  return sens_get_map_internal(output, voltage, power_voltage);
}

HAL_StatusTypeDef sens_get_map_urgent(float *output)
{
  float voltage = adc_get_voltage_urgent(AdcChManifoldAbsolutePressure);
  float power_voltage = adc_get_voltage(AdcMcuChReferenceVoltage);

  return sens_get_map_internal(output, voltage, power_voltage);
}

HAL_StatusTypeDef sens_get_air_temperature(float *output)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t now = Delay_Tick;
  static uint32_t last_ok = 0;
  static HAL_StatusTypeDef status_old = HAL_OK;
  static float result_old = 150.0f;
  float result = result_old;
  float reference_resistance, meter_resistance;
  float power_voltage = adc_get_voltage(AdcMcuChReferenceVoltage);
  float temperature = adc_get_voltage(AdcChAirTemperature);

  if(power_voltage != 0.0f) {
    if(temperature > power_voltage)
      temperature = power_voltage;

    reference_resistance = 2700.0f;
    meter_resistance = (reference_resistance / (1.0f - (temperature/power_voltage))) - reference_resistance;
    temperature = getTemperatureByResistance_airtemp(meter_resistance);
    if(temperature < -40.0f) {
      result = -40.0f;
      status = HAL_ERROR;
    }
    else if(temperature > 150.0f) {
      result = 150.0f;
      status = HAL_ERROR;
    }
    else
      result = temperature;
  }
  else
    status = HAL_ERROR;

  if(status == HAL_OK) {
    status_old = HAL_OK;
    last_ok = now;
    result_old = result;
    *output = result;
  } else if(status_old == HAL_OK) {
    *output = result_old;
    status_old = HAL_OK;
    if(DelayDiff(now, last_ok) > SENS_ERROR_DELAY) {
      status_old = HAL_ERROR;
    }
  } else {
    status_old = HAL_ERROR;
    *output = result;
  }

  return status_old;
}

HAL_StatusTypeDef sens_get_engine_temperature(float *output)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t now = Delay_Tick;
  static uint32_t last_ok = 0;
  static HAL_StatusTypeDef status_old = HAL_OK;
  static float result_old = 150.0f;
  float result = result_old;
  float reference_resistance, meter_resistance;
  float power_voltage = adc_get_voltage(AdcMcuChReferenceVoltage);
  float temperature = adc_get_voltage(AdcChEngineTemperature);

  if(power_voltage != 0.0f) {
    if(temperature > power_voltage)
      temperature = power_voltage;

    reference_resistance = 2700.0f;
    meter_resistance = (reference_resistance / (1.0f - (temperature/power_voltage))) - reference_resistance;
    temperature = getTemperatureByResistance_enginetemp(meter_resistance);
    if(temperature < -40.0f) {
      result = -40.0f;
      status = HAL_ERROR;
    }
    else if(temperature > 150.0f) {
      result = 150.0f;
      status = HAL_ERROR;
    }
    else
      result = temperature;
  }
  else
    status = HAL_ERROR;

  if(status == HAL_OK) {
    status_old = HAL_OK;
    last_ok = now;
    result_old = result;
    *output = result;
  } else if(status_old == HAL_OK) {
    *output = result_old;
    status_old = HAL_OK;
    if(DelayDiff(now, last_ok) > SENS_ERROR_DELAY) {
      status_old = HAL_ERROR;
    }
  } else {
    status_old = HAL_ERROR;
    *output = result;
  }

  return status_old;
}

HAL_StatusTypeDef sens_get_throttle_position(float *output)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t now = Delay_Tick;
  static uint32_t last_ok = 0;
  static HAL_StatusTypeDef status_old = HAL_OK;
  static float result_old = 0;
  float result = result_old;
  float value = adc_get_voltage(AdcChThrottlePosition);
  float voltage_from = gSensTpsMin;
  float voltage_to = gSensTpsMax;

  if(value + 0.15f < voltage_from || value - 0.15f > voltage_to) {
    status = HAL_ERROR;
  }

  if(value < voltage_from)
    value = voltage_from;
  if(value > voltage_to)
    value = voltage_to;

  value -= voltage_from;
  value /= voltage_to - voltage_from;
  value *= 100.0f;

  result = value;

  if(status == HAL_OK) {
    status_old = HAL_OK;
    last_ok = now;
    result_old = result;
    *output = result;
  } else if(status_old == HAL_OK) {
    *output = result_old;
    status_old = HAL_OK;
    if(DelayDiff(now, last_ok) > SENS_ERROR_DELAY) {
      status_old = HAL_ERROR;
    }
  } else {
    status_old = HAL_ERROR;
    *output = result;
  }

  return status_old;
}

HAL_StatusTypeDef sens_get_power_voltage(float *output)
{
  HAL_StatusTypeDef status = HAL_OK;
  float power_voltage = adc_get_voltage(AdcChPowerVoltage);

  if(power_voltage < 6.0f || power_voltage > 16.5f) {
    status = HAL_ERROR;
  }

  *output = power_voltage;

  return status;
}

HAL_StatusTypeDef sens_get_reference_voltage(float *output)
{
  HAL_StatusTypeDef status = HAL_OK;
  float reference_voltage = adc_get_voltage(AdcMcuChReferenceVoltage);

  if(reference_voltage < 4.5f || reference_voltage > 5.5f) {
    status = HAL_ERROR;
  }

  *output = reference_voltage;

  return status;
}
