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
  AdcChO2UA,
  AdcMcuChReferenceVoltage
}eAdcChannel;

#define ADC_RANGE_PM2500 0x00
#define ADC_RANGE_PM1250 0x01
#define ADC_RANGE_PM0625 0x02
#define ADC_RANGE_0P2500 0x05
#define ADC_RANGE_0P1250 0x06
#define MCU_RANGE_DIRECT 0x10

#define ADC_FILTER_DISABLE 0
#define ADC_FILTER_ENABLE  1

typedef int8_t (*AdcChannelEvent)(eAdcChannel channel);

void ADC_ErrorCallback(SPI_HandleTypeDef * _hspi);
void ADC_TxCpltCallback(SPI_HandleTypeDef * _hspi);
void ADC_RxCpltCallback(SPI_HandleTypeDef * _hspi);
void ADC_TxRxCpltCallback(SPI_HandleTypeDef * _hspi);
void ADC_MCU_ConvCpltCallback(ADC_HandleTypeDef * _hadc);
HAL_StatusTypeDef adc_init(SPI_HandleTypeDef * _hspi, ADC_HandleTypeDef * _hadc);
HAL_StatusTypeDef adc_register(eAdcChannel channel, uint8_t range, float divider, uint8_t filter);
HAL_StatusTypeDef adc_set_lpf(eAdcChannel channel, float lpf);
HAL_StatusTypeDef adc_get_lpf(eAdcChannel channel, float *p_lpf);
HAL_StatusTypeDef adc_fast_loop(void);
HAL_StatusTypeDef adc_slow_loop(void);
float adc_get_voltage(eAdcChannel channel);
float adc_get_voltage_unfiltered(eAdcChannel channel);
float adc_get_voltage_urgent(eAdcChannel channel);
HAL_StatusTypeDef adc_get_status(void);
void adc_set_events(AdcChannelEvent startEvent, AdcChannelEvent doneEvent);

#endif /* INC_ADC_H_ */
