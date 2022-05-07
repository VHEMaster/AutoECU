/*
 * mis.h
 *
 *  Created on: Mar 4, 2022
 *      Author: VHEMaster
 */

#ifndef INC_MISC_H_
#define INC_MISC_H_

#include "main.h"
#include "structs.h"

typedef enum {
  O2AmplificationFactor8 = 0,
  O2AmplificationFactor17 = 1,
  O2AmplificationFactorCount,
}eO2AmplificationFactor;

typedef struct {
    volatile float ReferenceVoltage;
    float HeaterVoltage;
    float TemperatureVoltage;
    float Lambda;
    float Temperature;
    volatile uint8_t Valid;
    eO2AmplificationFactor AmplificationFactor;
    uint8_t Available;
    uint8_t Working;
    union {
        uint8_t Byte;
        sO2Diagnostic Fields;
    }Diag;
}sO2Status;

typedef struct {
    uint8_t bandpass_filter_frequency;
    uint8_t gain_value;
    uint8_t integrator_time_constant;
    uint8_t oscillator_freq_select;
    uint8_t channel_select;
    uint8_t diagnostic_mode;
    uint8_t so_output_mode;
}sKnockConfig;

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
HAL_StatusTypeDef Misc_Outs_GetDiagnostic(eMiscDiagChannels channel, uint8_t *byte);
sO2Status Misc_O2_GetStatus(void);

uint8_t Misc_GetIdleValvePosition(void);
void Misc_SetIdleValvePosition(uint8_t position);
HAL_StatusTypeDef Misc_GetIdleValveStatus(void);
void Misc_EnableIdleValvePosition(uint8_t enablement_position);
int8_t Misc_CalibrateIdleValve(void);

void Misc_Fast_Loop(void);
void Misc_Loop(void);

HAL_StatusTypeDef Knock_GetValueByRPM(float *value);
HAL_StatusTypeDef Knock_GetValueRaw(float *value);

void Knock_SetBandpassFilterFrequency(uint8_t value);
void Knock_SetGainValue(uint8_t value);
void Knock_SetIntegratorTimeConstant(uint8_t value);
void Knock_SetOscillatorFrequency(uint8_t value);
void Knock_SetChannel(uint8_t value);
void Knock_SetDiagnosticMode(uint8_t value);
void Knock_SetSoOutputMode(uint8_t value);
uint8_t Knock_GetBandpassFilterFrequency(void);
uint8_t Knock_GetGainValue(void);
uint8_t Knock_GetIntegratorTimeConstant(void);
uint8_t Knock_GetOscillatorFrequency(void);
uint8_t Knock_GetChannel(void);
uint8_t Knock_GetDiagnosticMode(void);
uint8_t Knock_GetSoOutputMode(void);

void O2_SetAmplificationFactor(eO2AmplificationFactor factor);
eO2AmplificationFactor O2_GetAmplificationFactor(void);

#endif /* INC_MISC_H_ */
