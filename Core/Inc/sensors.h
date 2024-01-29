/*
 * sensors.h
 *
 *  Created on: Mar 5, 2022
 *      Author: VHEMaster
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include "main.h"
#include "misc.h"

typedef enum {
  SensorOilPressure = 0,
  SensorFanForceSwitch,
  SensorHandbrake,
  SensorCharge,
  SensorClutch,
  SensorIgn,
  SensorCount
}eSensor;

HAL_StatusTypeDef sens_get_adc_status(void);
HAL_StatusTypeDef sens_get_map(float *output);
HAL_StatusTypeDef sens_get_map_urgent(float *output);
HAL_StatusTypeDef sens_set_map_lpf(float map_lpf);
HAL_StatusTypeDef sens_reset_map_lpf(void);
HAL_StatusTypeDef sens_get_air_temperature(float *output);
HAL_StatusTypeDef sens_get_engine_temperature(float *output);
HAL_StatusTypeDef sens_get_throttle_position(float *output);
HAL_StatusTypeDef sens_get_power_voltage(float *output);
HAL_StatusTypeDef sens_get_reference_voltage(float *output);
sO2Status sens_get_o2_status(void);
HAL_StatusTypeDef sens_set_o2_lpf(float lpf);
HAL_StatusTypeDef sens_get_o2_labmda(const sO2Status *p_status, float *output, uint8_t *valid);
HAL_StatusTypeDef sens_get_o2_temperature(const sO2Status *p_status, float *output);
HAL_StatusTypeDef sens_get_o2_diagnostic(const sO2Status *p_status, sO2Diagnostic *output);
HAL_StatusTypeDef sens_get_o2_heatervoltage(const sO2Status *p_status, float *output);
HAL_StatusTypeDef sens_get_o2_temperaturevoltage(const sO2Status *p_status, float *output);

GPIO_PinState sens_get_charge(uint32_t *time);
GPIO_PinState sens_get_handbrake(uint32_t *time);
GPIO_PinState sens_get_oil_pressure(uint32_t *time);
GPIO_PinState sens_get_fan_force_switch(uint32_t *time);
GPIO_PinState sens_get_clutch(uint32_t *time);
GPIO_PinState sens_get_ign(uint32_t *time);

void sens_configure_map(float gain, float offset);
void sens_configure_tps(float min, float max);

void sensors_init(void);
HAL_StatusTypeDef sensors_register(eSensor sensor, GPIO_TypeDef *port, uint16_t pin, uint8_t inverted);
void sensors_loop(void);

#endif /* INC_SENSORS_H_ */
