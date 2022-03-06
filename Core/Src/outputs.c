/*
 * outputs.h
 *
 *  Created on: Mar 6, 2022
 *      Author: VHEMaster
 */

#include "outputs.h"

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
  uint8_t inverted;
  volatile GPIO_PinState state;
}sOutput;

static sOutput Outputs[OutputCount] = {{0}};

inline void out_set_fuelpump(GPIO_PinState state)
{
  Outputs[OutFuelPumpRelay].state = state;
}

inline void out_set_checkengine(GPIO_PinState state)
{
  Outputs[OutCheckEngine].state = state;
}

inline void out_set_fan(GPIO_PinState state)
{
  Outputs[OutFanRelay].state = state;
}

inline void out_set_starter(GPIO_PinState state)
{
  Outputs[OutStarterRelay].state = state;
}

inline void out_set_rsvd1(GPIO_PinState state)
{
  Outputs[OutRsvd1].state = state;
}

inline void out_set_rsvd2(GPIO_PinState state)
{
  Outputs[OutRsvd2].state = state;
}


HAL_StatusTypeDef outputs_register(eOutput output, GPIO_TypeDef *port, uint16_t pin, uint8_t inverted, GPIO_PinState initial)
{
  if(output < OutputCount && port && pin) {
    Outputs[output].port = port;
    Outputs[output].pin = pin;
    Outputs[output].inverted = inverted;
    Outputs[output].state = initial;

    port->BSRR = pin << ((initial == inverted) * 16);

    return HAL_OK;
  }
  return HAL_ERROR;

}

inline void outputs_loop(void)
{
  for(int i = 0; i < OutputCount; i++) {
    if(Outputs[i].port && Outputs[i].pin) {
      Outputs[i].port->BSRR = Outputs[i].pin << ((Outputs[i].state == Outputs[i].inverted) * 16);
    }
  }
}

