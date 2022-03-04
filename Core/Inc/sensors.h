/*
 * sensors.h
 *
 *  Created on: Mar 5, 2022
 *      Author: VHEMaster
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include "main.h"

HAL_StatusTypeDef sens_get_air_temperature(float *output);
HAL_StatusTypeDef sens_get_engine_temperature(float *output);
HAL_StatusTypeDef sens_get_throttle_position(float *output);
HAL_StatusTypeDef sens_get_power_voltage(float *output);

#endif /* INC_SENSORS_H_ */
