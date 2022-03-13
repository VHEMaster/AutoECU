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

typedef enum {
  OutputDiagShortToGnd = 0,
  OutputDiagShortToBatOrOvertemp = 1,
  OutputDiagOpenCircuit = 2,
  OutputDiagNoFailure = 3
}eOutputDiagnosticStatus;

typedef struct {
  struct {
    union {
      struct {
        eOutputDiagnosticStatus InjCy4 : 2;
        eOutputDiagnosticStatus InjCy3 : 2;
        eOutputDiagnosticStatus InjCy2 : 2;
        eOutputDiagnosticStatus InjCy1 : 2;
      }Data;
      uint8_t Byte;
    }Diagnostic;
    HAL_StatusTypeDef Availability;
  }Injectors;
  struct {
    union {
      struct {
        eOutputDiagnosticStatus CheckEngine : 2;
        eOutputDiagnosticStatus Speedmeeter : 2;
        eOutputDiagnosticStatus Tachometer : 2;
        eOutputDiagnosticStatus FuelPumpRelay : 2;
      }Data;
      uint8_t Byte;
    }Diagnostic;
    HAL_StatusTypeDef Availability;
  }Outs1;
  struct {
    union {
      struct {
        eOutputDiagnosticStatus OutRsvd2 : 2;
        eOutputDiagnosticStatus OutRsvd1 : 2;
        eOutputDiagnosticStatus StarterRelay : 2;
        eOutputDiagnosticStatus FanRelay : 2;
      }Data;
      uint8_t Byte;
    }Diagnostic;
    HAL_StatusTypeDef Availability;
  }Outs2;
  struct {
    HAL_StatusTypeDef Status;
  }IdleValvePosition;
}sOutputDiagnostic;

void out_set_fuelpump(GPIO_PinState state);
void out_set_checkengine(GPIO_PinState state);
void out_set_fan(GPIO_PinState state);
void out_set_starter(GPIO_PinState state);
void out_set_rsvd1(GPIO_PinState state);
void out_set_rsvd2(GPIO_PinState state);

GPIO_PinState out_get_fuelpump(uint32_t *time);
GPIO_PinState out_get_checkengine(uint32_t *time);
GPIO_PinState out_get_fan(uint32_t *time);
GPIO_PinState out_get_starter(uint32_t *time);
GPIO_PinState out_get_rsvd1(uint32_t *time);
GPIO_PinState out_get_rsvd2(uint32_t *time);

void out_set_idle_valve(int32_t position);
uint8_t out_get_idle_valve(void);
int8_t out_calibrate_idle_valve(void);
void out_enable_idle_valve(uint32_t enablement_position);

HAL_StatusTypeDef outputs_get_diagnostic(sOutputDiagnostic *diagnostic);

void outputs_init(void);
HAL_StatusTypeDef outputs_register(eOutput output, GPIO_TypeDef *port, uint16_t pin, uint8_t inverted, GPIO_PinState initial);
void outputs_loop(void);

#endif /* INC_OUTPUTS_H_ */
