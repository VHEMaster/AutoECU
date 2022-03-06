/*
 * outputs.h
 *
 *  Created on: Mar 6, 2022
 *      Author: VHEMaster
 */

#ifndef INC_OUTPUTS_H_
#define INC_OUTPUTS_H_

#include "main.h"

typedef enum {
  OutFuelPumpRelay,
  OutCheckEngine,
  OutFanRelay,
  OutStarterRelay,
  OutRsvd1,
  OutRsvd2,
  OutputCount
}eOutput;

void out_set_fuelpump(GPIO_PinState state);
void out_set_checkengine(GPIO_PinState state);
void out_set_fan(GPIO_PinState state);
void out_set_starter(GPIO_PinState state);
void out_set_rsvd1(GPIO_PinState state);
void out_set_rsvd2(GPIO_PinState state);

HAL_StatusTypeDef outputs_register(eOutput output, GPIO_TypeDef *port, uint16_t pin, uint8_t inverted, GPIO_PinState initial);
void outputs_loop(void);

#endif /* INC_OUTPUTS_H_ */
