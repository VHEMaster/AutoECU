/*
 * outputs.h
 *
 *  Created on: Mar 6, 2022
 *      Author: VHEMaster
 */

#include "outputs.h"
#include "misc.h"
#include <string.h>

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
  uint8_t inverted;
  volatile GPIO_PinState state;
}sOutput;

static sOutput Outputs[OutputCount] = {{0}};
static sOutputDiagnostic OutputDiagnostic;
static volatile HAL_StatusTypeDef OutputsDiagStatus = HAL_OK;

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

void outputs_init(void)
{
  memset(&OutputDiagnostic, 0xFF, sizeof(OutputDiagnostic));
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
  HAL_StatusTypeDef diag_status = HAL_OK;
  uint8_t diags[MiscDiagChCount];

  for(int i = 0; i < OutputCount; i++) {
    if(Outputs[i].port && Outputs[i].pin) {
      if(Outputs[i].state == Outputs[i].inverted) {
        if((Outputs[i].port->ODR & Outputs[i].pin) != GPIO_PIN_RESET)
          Outputs[i].port->BSRR = Outputs[i].pin << 16;
      } else {
        if((Outputs[i].port->ODR & Outputs[i].pin) == GPIO_PIN_RESET)
          Outputs[i].port->BSRR = Outputs[i].pin;
      }
    }
  }

  for(int i = 0; i < MiscDiagChCount; i++)
    diag_status |= Misc_Outs_GetDiagnostic(i, &diags[i]);
  OutputDiagnostic.Injectors.Byte = diags[MiscDiagChInjectors];
  OutputDiagnostic.Outs1.Byte = diags[MiscDiagChOutputs1];
  OutputDiagnostic.Outs2.Byte = diags[MiscDiagChOutputs2];
  OutputsDiagStatus = diag_status;

}

HAL_StatusTypeDef outputs_get_diagnostic(sOutputDiagnostic *diagnostic)
{
  *diagnostic = OutputDiagnostic;
  return HAL_OK;
}


