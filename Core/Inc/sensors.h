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
  SensorRsvd1,
  SensorRsvd2,
  SensorCount
}eSensor;

HAL_StatusTypeDef sens_get_adc_status(void);
HAL_StatusTypeDef sens_get_knock(float *output);
HAL_StatusTypeDef sens_get_map(float *output);
HAL_StatusTypeDef sens_get_air_temperature(float *output);
HAL_StatusTypeDef sens_get_engine_temperature(float *output);
HAL_StatusTypeDef sens_get_throttle_position(float *output);
HAL_StatusTypeDef sens_get_power_voltage(float *output);
HAL_StatusTypeDef sens_get_reference_voltage(float *output);
HAL_StatusTypeDef sens_get_o2_fuelratio(float *output, uint8_t *valid);
HAL_StatusTypeDef sens_get_o2_diagnostic(sO2Diagnostic *output);

GPIO_PinState sens_get_charge(void);
GPIO_PinState sens_get_handbrake(void);
GPIO_PinState sens_get_oil_pressure(void);
GPIO_PinState sens_get_starter(void);
GPIO_PinState sens_get_rsvd1(void);
GPIO_PinState sens_get_rsvd2(void);

HAL_StatusTypeDef sensors_register(eSensor sensor, GPIO_TypeDef *port, uint16_t pin, uint8_t inverted);
void sensors_loop(void);

#endif /* INC_SENSORS_H_ */
