/*
 * outputs.h
 *
 *  Created on: Mar 6, 2022
 *      Author: VHEMaster
 */

#include "outputs.h"
#include "misc.h"
#include <string.h>
#include <limits.h>

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
  uint8_t inverted;
  volatile GPIO_PinState state;
}sOutput;

static sOutput Outputs[OutputCount] = {{0}};
static sOutputDiagnostic OutputDiagnostic;

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

inline GPIO_PinState out_get_fuelpump(void)
{
  return Outputs[OutFuelPumpRelay].state;
}

inline GPIO_PinState out_get_checkengine(void)
{
  return Outputs[OutCheckEngine].state;
}

inline GPIO_PinState out_get_fan(void)
{
  return Outputs[OutFanRelay].state;
}

inline GPIO_PinState out_get_starter(void)
{
  return Outputs[OutStarterRelay].state;
}

inline GPIO_PinState out_get_rsvd1(void)
{
  return Outputs[OutRsvd1].state;
}

inline GPIO_PinState out_get_rsvd2(void)
{
  return Outputs[OutRsvd2].state;
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

  OutputDiagnostic.Injectors.Availability =
      Misc_Outs_GetDiagnostic(MiscDiagChInjectors,
          &OutputDiagnostic.Injectors.Diagnostic.Byte);

  OutputDiagnostic.Outs1.Availability =
      Misc_Outs_GetDiagnostic(MiscDiagChOutputs1,
          &OutputDiagnostic.Outs1.Diagnostic.Byte);

  OutputDiagnostic.Outs2.Availability =
      Misc_Outs_GetDiagnostic(MiscDiagChOutputs2,
          &OutputDiagnostic.Outs2.Diagnostic.Byte);

  OutputDiagnostic.IdleValvePosition.Status =
      Misc_GetIdleValveStatus();
}
inline void out_set_idle_valve(int32_t position)
{
  Misc_SetIdleValvePosition(position > UCHAR_MAX ? UCHAR_MAX : position < 0 ? 0 : position);
}

inline uint8_t out_get_idle_valve(void)
{
  return Misc_GetIdleValvePosition();
}

inline int8_t out_calibrate_idle_valve(void)
{
  return Misc_CalibrateIdleValve();
}

inline void out_enable_idle_valve(uint32_t enablement_position)
{
  Misc_EnableIdleValvePosition(enablement_position > UCHAR_MAX ? UCHAR_MAX : enablement_position < 0 ? 0 : enablement_position);
}

HAL_StatusTypeDef outputs_get_diagnostic(sOutputDiagnostic *diagnostic)
{
  *diagnostic = OutputDiagnostic;
  return HAL_OK;
}

