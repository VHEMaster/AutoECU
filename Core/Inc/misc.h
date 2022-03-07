/*
 * mis.h
 *
 *  Created on: Mar 4, 2022
 *      Author: VHEMaster
 */

#ifndef INC_MISC_H_
#define INC_MISC_H_

#include "main.h"

typedef enum {
  O2DiagShortToGnd = 0,
  O2DiagNoPower,
  O2DiagShortToBat,
  O2DiagOK
}eO2DiagCode;

typedef struct {
    eO2DiagCode VM : 2;
    eO2DiagCode UN : 2;
    eO2DiagCode IAIP : 2;
    eO2DiagCode DIAHGD : 2;
}sO2Diagnostic;

typedef struct {
    float FuelRatio;
    uint8_t Available;
    uint8_t Working;
    uint8_t Valid;
    union {
        uint8_t Byte;
        sO2Diagnostic Fields;
    }Diag;
}sO2Status;

typedef enum {
  MiscDiagChInjectors = 0,
  MiscDiagChOutputs1,
  MiscDiagChOutputs2,
  MiscDiagChCount
}eMiscDiagChannels;

void Misc_ErrorCallback(SPI_HandleTypeDef * _hspi);
void Misc_TxCpltCallback(SPI_HandleTypeDef * _hspi);
void Misc_RxCpltCallback(SPI_HandleTypeDef * _hspi);
void Misc_TxRxCpltCallback(SPI_HandleTypeDef * _hspi);
HAL_StatusTypeDef Misc_Init(SPI_HandleTypeDef * _hspi);
HAL_StatusTypeDef Misc_O2_Init(uint32_t pwm_period, volatile uint32_t *pwm_duty);
HAL_StatusTypeDef Mics_Knock_Init(void);
HAL_StatusTypeDef Misc_GetKnockValueByRPM(float *value);
HAL_StatusTypeDef Misc_GetKnockValueRaw(float *value);
HAL_StatusTypeDef Misc_Outs_GetDiagnostic(eMiscDiagChannels channel, uint8_t *byte);
sO2Status Misc_O2_GetStatus(void);
void Misc_Loop(void);

#endif /* INC_MISC_H_ */
