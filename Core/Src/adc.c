/*
 * adc.c
 *
 *  Created on: Mar 4, 2022
 *      Author: VHEMaster
 */

#include "adc.h"
#include "delay.h"
#include <string.h>

#define SPI_NSS_ON() HAL_GPIO_WritePin(SPI1_NSS_ADC_GPIO_Port, SPI1_NSS_ADC_Pin, GPIO_PIN_RESET)
#define SPI_NSS_OFF() HAL_GPIO_WritePin(SPI1_NSS_ADC_GPIO_Port, SPI1_NSS_ADC_Pin, GPIO_PIN_SET)

#define ADC_VREF 4.096f
#define MCU_VREF 3.30f

#define ADC_CHANNELS 8
#define MCU_CHANNELS 1
#define ADC_BUFFER_SIZE 8

static uint16_t AdcBuffer[ADC_CHANNELS + MCU_CHANNELS][ADC_BUFFER_SIZE] = {{0}};
static float AdcVoltages[ADC_CHANNELS + MCU_CHANNELS];
static volatile HAL_StatusTypeDef adcInitStatus = HAL_OK;
static volatile HAL_StatusTypeDef adcStatus = HAL_OK;

static uint16_t ChData[ADC_CHANNELS + MCU_CHANNELS] = {0};
static uint8_t ChRange[ADC_CHANNELS + MCU_CHANNELS] = {0};
static uint8_t ChFilter[ADC_CHANNELS + MCU_CHANNELS] = {0};
static uint8_t ChIgnoreNext[ADC_CHANNELS + MCU_CHANNELS] = {0};
static float ChDivider[ADC_CHANNELS + MCU_CHANNELS] = {0};

static SPI_HandleTypeDef *hspi = NULL;
static ADC_HandleTypeDef *hadc = NULL;

static uint8_t tx[32] __attribute__((aligned(32)));
static uint8_t rx[32] __attribute__((aligned(32)));

static volatile uint8_t semTx = 0;
static volatile uint8_t semRx = 0;
static uint32_t AdcWritePos = 0;
static uint8_t AdcChannel = 0;
static uint8_t AdcFirstTime = 1;
static uint8_t McuFirstTime = 1;
static uint32_t McuWritePos = 0;

static AdcChannelEvent AdcStartSamplingEvent = NULL;
static AdcChannelEvent AdcSamplingDoneEvent = NULL;

void adc_set_events(AdcChannelEvent startEvent, AdcChannelEvent doneEvent)
{
  AdcStartSamplingEvent = startEvent;
  AdcSamplingDoneEvent = doneEvent;
}

void ADC_ErrorCallback(SPI_HandleTypeDef * _hspi)
{
  if(_hspi == hspi) {

  }
}

void ADC_TxCpltCallback(SPI_HandleTypeDef * _hspi)
{
  if(_hspi == hspi) {
    SPI_NSS_OFF();
    semTx = 1;
  }
}

void ADC_RxCpltCallback(SPI_HandleTypeDef * _hspi)
{
  if(_hspi == hspi) {
    SPI_NSS_OFF();
    semRx = 1;
  }
}

void ADC_TxRxCpltCallback(SPI_HandleTypeDef * _hspi)
{
  if(_hspi == hspi) {
    SPI_NSS_OFF();
    semTx = 1;
    semRx = 1;
  }
}

void ADC_MCU_ConvCpltCallback(ADC_HandleTypeDef * _hadc)
{
  static uint8_t McuChannel = 0;
  uint16_t data;
  if(_hadc == hadc) {
    data = HAL_ADC_GetValue(hadc);

    ChIgnoreNext[ADC_CHANNELS + McuChannel] = 1;
    if(!ChIgnoreNext[ADC_CHANNELS + McuChannel]) {
      if(ChFilter[ADC_CHANNELS + McuChannel]) {
        if(McuFirstTime) {
          McuFirstTime = 0;
          McuWritePos = 0;
          for(int j = 0; j < ADC_BUFFER_SIZE; j++) {
            AdcBuffer[ADC_CHANNELS + McuChannel][j] = data;
          }
        }
        else {
          AdcBuffer[ADC_CHANNELS + McuChannel][McuWritePos] = data;
          if(++McuWritePos >= ADC_BUFFER_SIZE)
            McuWritePos = 0;
        }
      } else {
        AdcBuffer[ADC_CHANNELS + McuChannel][0] = data;
      }
    } else {
      ChIgnoreNext[ADC_CHANNELS + McuChannel] = 0;
    }

    if(AdcSamplingDoneEvent)
      if(AdcSamplingDoneEvent(ADC_CHANNELS + McuChannel) < 0)
        ChIgnoreNext[ADC_CHANNELS + McuChannel] = 1;

    if(++McuChannel >= MCU_CHANNELS)
      McuChannel = 0;

    if(AdcStartSamplingEvent)
      if(AdcStartSamplingEvent(ADC_CHANNELS + McuChannel) < 0)
        ChIgnoreNext[ADC_CHANNELS + McuChannel] = 1;
  }

}

static uint8_t waitTxRxCplt(void)
{
  if(semRx && semTx) {
    semRx = 0;
    semTx = 0;
    return 1;
  }
  return 0;
}

static HAL_StatusTypeDef SPI_SendCommand(uint16_t cmd)
{

  tx[0] = cmd >> 8;
  tx[1] = cmd & 0xFF;
  //SCB_CleanDCache_by_Addr((uint32_t*)tx, 2);
  SPI_NSS_ON();
  HAL_SPI_TransmitReceive_IT(hspi, tx, rx, 1);
  while(!waitTxRxCplt()) {}
  //SPI_NSS_OFF();
  //SCB_InvalidateDCache_by_Addr((uint32_t*)rx, 2);

  if(rx[0] != 0x00 || rx[1] != 0x00)
    return HAL_ERROR;

  return HAL_OK;
}

static HAL_StatusTypeDef SPI_WriteRegister(uint8_t reg, uint8_t data)
{
  tx[0] = reg << 1 | 1;
  tx[1] = data;
  tx[2] = 0;
  //SCB_CleanDCache_by_Addr((uint32_t*)tx, 3);
  SPI_NSS_ON();
  HAL_SPI_TransmitReceive_IT(hspi, tx, rx, 3);
  while(!waitTxRxCplt()) {}
  //SPI_NSS_OFF();
  //SCB_InvalidateDCache_by_Addr((uint32_t*)rx, 3);

  if(rx[0] != 0x00 || rx[1] != 0x00)
    return HAL_ERROR;
  if(rx[2] != data)
    return HAL_ERROR;

  return HAL_OK;
}

static HAL_StatusTypeDef SPI_ReadRegister(uint8_t reg, uint8_t *data)
{
  *data = 0;

  tx[0] = reg << 1;
  tx[1] = 0;
  tx[2] = 0;
  //SCB_CleanDCache_by_Addr((uint32_t*)tx, 3);
  SPI_NSS_ON();
  HAL_SPI_TransmitReceive_IT(hspi, tx, rx, 3);
  while(!waitTxRxCplt()) {}
  //SPI_NSS_OFF();
  //SCB_InvalidateDCache_by_Addr((uint32_t*)rx, 3);

  if(rx[0] != 0x00 || rx[1] != 0x00)
    return HAL_ERROR;

  *data = rx[2];

  return HAL_OK;
}

static inline float ADC_Convert(uint8_t channel)
{
  float data = ChData[channel];
  switch(ChRange[channel]) {
    case ADC_RANGE_0P2500 :
      return data / 65536.0f * ADC_VREF * 2.500f;
    case ADC_RANGE_0P1250 :
      return data / 65536.0f * ADC_VREF * 1.250f;
    case MCU_RANGE_DIRECT :
      return data / 65536.0f * MCU_VREF;
    case ADC_RANGE_PM2500 :
      return (data - 32768.0f) / 32768.0f * ADC_VREF * 2.500f;
    case ADC_RANGE_PM1250 :
      return (data - 32768.0f) / 32768.0f * ADC_VREF * 1.250f;
    case ADC_RANGE_PM0625 :
      return (data - 32768.0f) / 32768.0f * ADC_VREF * 0.625f;
    default:
      return 0;
  }
}

HAL_StatusTypeDef adc_init(SPI_HandleTypeDef * _hspi, ADC_HandleTypeDef * _hadc)
{
  HAL_StatusTypeDef result = HAL_OK;
  uint8_t data;

  SCB_CleanDCache_by_Addr((uint32_t*)tx, sizeof(tx));
  SCB_CleanDCache_by_Addr((uint32_t*)rx, sizeof(rx));

  HAL_GPIO_WritePin(SPI1_NRST_GPIO_Port, SPI1_NRST_Pin, GPIO_PIN_SET);

  DelayMs(1);

  hspi = _hspi;
  hadc = _hadc;

  //RST command
  result = SPI_SendCommand(0x8500);
  if(result != HAL_OK)
    goto ret;

  //Enable all channels
  result = SPI_WriteRegister(0x02, 0x00);
  if(result != HAL_OK)
    goto ret;

  //Check all channels enabled
  result = SPI_ReadRegister(0x02, &data);
  if(result != HAL_OK)
    goto ret;

  if(data != 0x00) {
    goto ret;
    result = HAL_ERROR;
  }

  //All channels in sequence mode
  result = SPI_WriteRegister(0x01, 0xFF);
  if(result != HAL_OK)
    goto ret;

  //Check all channels in sequence mode
  result = SPI_ReadRegister(0x01, &data);
  if(result != HAL_OK)
    goto ret;

  if(data != 0xFF) {
    result = HAL_ERROR;
    goto ret;
  }

  //SDO format: 011, dev address: 0
  result = SPI_WriteRegister(0x03, 0x03);
  if(result != HAL_OK)
    goto ret;

  //Check SDO format: 011, dev address: 0
  result = SPI_ReadRegister(0x03, &data);
  if(result != HAL_OK)
    goto ret;

  if(data != 0x03) {
    result = HAL_ERROR;
    goto ret;
  }

  for(int i = 0; i < ADC_CHANNELS; i++) {
    result = SPI_WriteRegister(0x05 + i, ChRange[i]);
    if(result != HAL_OK)
      goto ret;

    //Check channel setup
    result = SPI_ReadRegister(0x05 + i, &data);
    if(result != HAL_OK)
      goto ret;

    if(data != ChRange[i]) {
      result = HAL_ERROR;
      goto ret;
    }
  }

  for(int i = 0; i < ADC_CHANNELS + MCU_CHANNELS; i++) {
    for(int j = 0; j < ADC_BUFFER_SIZE; j++) {
      AdcBuffer[i][j] = 0x8000;
    }
  }

  result = SPI_SendCommand(0xA000); //AUTO_RST command
  if(result != HAL_OK)
    goto ret;

ret:
  adcInitStatus = result;
  adcStatus = result;

  if(AdcStartSamplingEvent)
    if(AdcStartSamplingEvent(ADC_CHANNELS) < 0)
      ChIgnoreNext[AdcChannel] = 1;

  return result;
}

HAL_StatusTypeDef adc_register(eAdcChannel channel, uint8_t range, float divider, uint8_t filter)
{
  HAL_StatusTypeDef result = HAL_OK;

  if(channel >= ADC_CHANNELS + MCU_CHANNELS)
    return HAL_ERROR;

  ChRange[channel] = range;
  ChDivider[channel] = divider;
  ChFilter[channel] = filter;

  return result;
}

HAL_StatusTypeDef adc_fast_loop(void)
{
  static HAL_StatusTypeDef result = HAL_OK;
  static uint8_t state = 0;
  uint16_t data;

  if(adcInitStatus != HAL_OK)
    return adcInitStatus;

  switch(state) {
    case 0:
      if(AdcStartSamplingEvent)
        if(AdcStartSamplingEvent(AdcChannel) < 0)
          ChIgnoreNext[AdcChannel] = 1;
      memset(tx, 0, 4);
      SCB_CleanDCache_by_Addr((uint32_t*)tx, 4);
      SPI_NSS_ON();
      HAL_SPI_TransmitReceive_DMA(hspi, tx, rx, 4);
      state++;
      break;
    case 1:
      if(waitTxRxCplt())
      {
        //SPI_NSS_OFF();
        SCB_InvalidateDCache_by_Addr((uint32_t*)rx, 4);
        data = rx[2] << 8 | rx[3];

        if(!ChIgnoreNext[AdcChannel]) {
          if(ChFilter[AdcChannel]) {
            if(!AdcFirstTime)
              AdcBuffer[AdcChannel][AdcWritePos] = data;
            else {
              AdcWritePos = 0;
              for(int i = 0; i < ADC_BUFFER_SIZE; i++) {
                AdcBuffer[AdcChannel][i] = data;
              }
            }
          } else {
            AdcBuffer[AdcChannel][0] = data;
          }
        } else {
          ChIgnoreNext[AdcChannel] = 0;
        }

        if(AdcSamplingDoneEvent)
          if(AdcSamplingDoneEvent(AdcChannel) < 0)
            ChIgnoreNext[AdcChannel] = 1;

        if(++AdcChannel >= ADC_CHANNELS) {
          if(++AdcWritePos >= ADC_BUFFER_SIZE) {
            AdcWritePos = 0;
          }
          AdcChannel = 0;
          AdcFirstTime = 0;
        }

        memset(tx, 0, 4);
        SCB_CleanDCache_by_Addr((uint32_t*)tx, 4);
        SPI_NSS_ON();
        HAL_SPI_TransmitReceive_DMA(hspi, tx, rx, 4);

      }
      break;

    default:
      state = 0;
      break;
  }

  return result;
}

HAL_StatusTypeDef adc_slow_loop(void)
{
  static HAL_StatusTypeDef result = HAL_OK;
  uint32_t data;
  uint32_t failed_channels = 0;

  if(adcInitStatus != HAL_OK)
    return adcInitStatus;

  for(int i = 0; i < ADC_CHANNELS + MCU_CHANNELS; i++) {
    if(ChFilter[i]) {
      data = 0;
      for(int j = 0; j < ADC_BUFFER_SIZE; j++)
        data += AdcBuffer[i][j];
      ChData[i] = data / ADC_BUFFER_SIZE;
    } else {
      ChData[i] = AdcBuffer[i][0];
    }
    AdcVoltages[i] = ADC_Convert(i) * ChDivider[i];
  }

  for(int i = 0; i < ADC_CHANNELS; i++) {
    if(ChData[i] == 0x0000 || ChData[i] == 0xFFFF)
      failed_channels++;
    else break;
  }

  if(failed_channels == ADC_CHANNELS)
    adcStatus = HAL_ERROR;

  return result;
}

inline float adc_get_voltage(eAdcChannel channel)
{
  if(channel < ADC_CHANNELS + MCU_CHANNELS)
    return AdcVoltages[channel];
  return 0.f;
}

HAL_StatusTypeDef adc_get_status(void)
{
  return adcStatus;
}

