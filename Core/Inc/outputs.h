/*
 * outputs.h
 *
 *  Created on: Mar 6, 2022
 *      Author: VHEMaster
 */

#ifndef INC_OUTPUTS_H_
#define INC_OUTPUTS_H_

#include "main.h"
#include "structs.h"

typedef enum {
  OutFuelPumpRelay,
  OutCheckEngine,
  OutFanRelay,
  OutStarterRelay,
  OutFanSwitch,
  OutIgn,
  OutputCount
}eOutput;

void out_set_fuelpump(GPIO_PinState state);
void out_set_checkengine(GPIO_PinState state);
void out_set_fan(GPIO_PinState state);
void out_set_starter(GPIO_PinState state);
void out_set_fan_switch(GPIO_PinState state);
void out_set_ign(GPIO_PinState state);

GPIO_PinState out_get_fuelpump(uint32_t *time);
GPIO_PinState out_get_checkengine(uint32_t *time);
GPIO_PinState out_get_fan(uint32_t *time);
GPIO_PinState out_get_starter(uint32_t *time);
GPIO_PinState out_get_fan_switch(uint32_t *time);
GPIO_PinState out_get_ign(uint32_t *time);

void out_set_idle_valve(int32_t position);
uint8_t out_get_idle_valve(void);
uint8_t out_is_idle_valve_moving(void);
int8_t out_reset_idle_valve(void);
void out_enable_idle_valve(uint32_t enablement_position);

HAL_StatusTypeDef outputs_get_diagnostic(sOutputDiagnostic *diagnostic);

void outputs_init(void);
HAL_StatusTypeDef outputs_register(eOutput output, GPIO_TypeDef *port, uint16_t pin, uint8_t inverted, GPIO_PinState initial);
void outputs_loop(void);

#endif /* INC_OUTPUTS_H_ */
