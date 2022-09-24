/*
 * outputs.h
 *
 *  Created on: Mar 6, 2022
 *      Author: VHEMaster
 */

#include "outputs.h"
#include "misc.h"
#include "delay.h"
#include <string.h>
#include <limits.h>

typedef struct {
  GPIO_TypeDef *port;
  volatile GPIO_PinState state;
  uint32_t time_switched;
  uint16_t pin;
  uint8_t inverted;
}sOutput;

static sOutput Outputs[OutputCount] = {{0}};
static sOutputDiagnostic OutputDiagnostic;

inline void out_set_fuelpump(GPIO_PinState state)
{
  if(Outputs[OutFuelPumpRelay].state != state) {
    Outputs[OutFuelPumpRelay].time_switched = Delay_Tick;
    Outputs[OutFuelPumpRelay].state = state;
  }
}

inline void out_set_checkengine(GPIO_PinState state)
{
  Outputs[OutCheckEngine].state = state;
  if(Outputs[OutCheckEngine].state != state) {
    Outputs[OutCheckEngine].time_switched = Delay_Tick;
    Outputs[OutCheckEngine].state = state;
  }
}

inline void out_set_fan(GPIO_PinState state)
{
  if(Outputs[OutFanRelay].state != state) {
    Outputs[OutFanRelay].time_switched = Delay_Tick;
    Outputs[OutFanRelay].state = state;
  }
}

inline void out_set_starter(GPIO_PinState state)
{
  if(Outputs[OutStarterRelay].state != state) {
    Outputs[OutStarterRelay].time_switched = Delay_Tick;
    Outputs[OutStarterRelay].state = state;
  }
}

inline void out_set_fan_switch(GPIO_PinState state)
{
  if(Outputs[OutFanSwitch].state != state) {
    Outputs[OutFanSwitch].time_switched = Delay_Tick;
    Outputs[OutFanSwitch].state = state;
  }
}

inline void out_set_ign(GPIO_PinState state)
{
  if(Outputs[OutIgn].state != state) {
    Outputs[OutIgn].time_switched = Delay_Tick;
    Outputs[OutIgn].state = state;
  }
}

inline GPIO_PinState out_get_fuelpump(uint32_t *time)
{
  if(time)
    *time = DelayDiff(Delay_Tick, Outputs[OutFuelPumpRelay].time_switched);
  return Outputs[OutFuelPumpRelay].state;
}

inline GPIO_PinState out_get_checkengine(uint32_t *time)
{
  if(time)
    *time = DelayDiff(Delay_Tick, Outputs[OutCheckEngine].time_switched);
  return Outputs[OutCheckEngine].state;
}

inline GPIO_PinState out_get_fan(uint32_t *time)
{
  if(time)
    *time = DelayDiff(Delay_Tick, Outputs[OutFanRelay].time_switched);
  return Outputs[OutFanRelay].state;
}

inline GPIO_PinState out_get_starter(uint32_t *time)
{
  if(time)
    *time = DelayDiff(Delay_Tick, Outputs[OutStarterRelay].time_switched);
  return Outputs[OutStarterRelay].state;
}

inline GPIO_PinState out_get_fan_switch(uint32_t *time)
{
  if(time)
    *time = DelayDiff(Delay_Tick, Outputs[OutFanSwitch].time_switched);
  return Outputs[OutFanSwitch].state;
}

inline GPIO_PinState out_get_ign(uint32_t *time)
{
  if(time)
    *time = DelayDiff(Delay_Tick, Outputs[OutIgn].time_switched);
  return Outputs[OutIgn].state;
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
    Outputs[output].time_switched = Delay_Tick;

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

inline uint8_t out_is_idle_valve_moving(void)
{
  return Misc_IsIdleValveMoving();
}

inline int8_t out_reset_idle_valve(uint32_t reset_position)
{
  return Misc_ResetIdleValve(reset_position);
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


