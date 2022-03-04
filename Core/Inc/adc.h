/*
 * adc.h
 *
 *  Created on: Mar 4, 2022
 *      Author: VHEMaster
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "main.h"

typedef enum {
  AdcChKnock = 0,
  AdcChAirTemperature,
  AdcChEngineTemperature,
  AdcChManifoldAbsolutePressure,
  AdcChThrottlePosition,
  AdcChPowerVoltage,
  AdcChO2UR,
  AdcChO2UA
}eAdcChannel;

void ADC_ErrorCallback(SPI_HandleTypeDef * _hspi);
void ADC_TxCpltCallback(SPI_HandleTypeDef * _hspi);
void ADC_RxCpltCallback(SPI_HandleTypeDef * _hspi);
void ADC_TxRxCpltCallback(SPI_HandleTypeDef * _hspi);
HAL_StatusTypeDef ADC_Init(SPI_HandleTypeDef * _hspi);
HAL_StatusTypeDef ADC_Fast_Loop(void);
HAL_StatusTypeDef ADC_Slow_Loop(void);
float ADC_GetVoltage(eAdcChannel channel);

#endif /* INC_ADC_H_ */
