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
  SensorStarter,
  SensorHandbrake,
  SensorCharge,
  SensorClutch,
  SensorIgn,
  SensorCount
}eSensor;

HAL_StatusTypeDef sens_get_adc_status(void);
HAL_StatusTypeDef sens_get_map(float *output);
HAL_StatusTypeDef sens_get_air_temperature(float *output);
HAL_StatusTypeDef sens_get_engine_temperature(float *output);
HAL_StatusTypeDef sens_get_throttle_position(float *output);
HAL_StatusTypeDef sens_get_power_voltage(float *output);
HAL_StatusTypeDef sens_get_reference_voltage(float *output);
HAL_StatusTypeDef sens_get_o2_labmda(float *output, uint8_t *valid);
HAL_StatusTypeDef sens_get_o2_temperature(float *output);
HAL_StatusTypeDef sens_get_o2_diagnostic(sO2Diagnostic *output);
HAL_StatusTypeDef sens_get_o2_heatervoltage(float *output);
HAL_StatusTypeDef sens_get_o2_temperaturevoltage(float *output);

GPIO_PinState sens_get_charge(uint32_t *time);
GPIO_PinState sens_get_handbrake(uint32_t *time);
GPIO_PinState sens_get_oil_pressure(uint32_t *time);
GPIO_PinState sens_get_starter(uint32_t *time);
GPIO_PinState sens_get_clutch(uint32_t *time);
GPIO_PinState sens_get_ign(uint32_t *time);

void sensors_init(void);
HAL_StatusTypeDef sensors_register(eSensor sensor, GPIO_TypeDef *port, uint16_t pin, uint8_t inverted);
void sensors_loop(void);

#endif /* INC_SENSORS_H_ */
