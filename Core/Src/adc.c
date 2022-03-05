/*
 * adc.c
 *
 *  Created on: Mar 4, 2022
 *      Author: VHEMaster
 */

#include "adc.h"

#define SPI_NSS_ON() HAL_GPIO_WritePin(SPI1_NSS_ADC_GPIO_Port, SPI1_NSS_ADC_Pin, GPIO_PIN_RESET)
#define SPI_NSS_OFF() HAL_GPIO_WritePin(SPI1_NSS_ADC_GPIO_Port, SPI1_NSS_ADC_Pin, GPIO_PIN_SET)

#define ADC_VREF 4.096f
#define MCU_VREF 3.30f

#define ADC_RANGE_PM2500 0x00
#define ADC_RANGE_PM1250 0x01
#define ADC_RANGE_PM0625 0x02
#define ADC_RANGE_0P2500 0x03
#define ADC_RANGE_0P1250 0x04
#define MCU_RANGE_DIRECT 0x10

#define ADC_CHANNELS 8
#define MCU_CHANNELS 1
#define ADC_BUFFER_SIZE 32

static uint16_t AdcBuffer[ADC_CHANNELS + MCU_CHANNELS][ADC_BUFFER_SIZE] = {{0}};
static float AdcVoltages[ADC_CHANNELS + MCU_CHANNELS];

static uint16_t ChData[ADC_CHANNELS + MCU_CHANNELS] = {0};
static const uint8_t ChRange[ADC_CHANNELS + MCU_CHANNELS] = {
    ADC_RANGE_0P1250,
    ADC_RANGE_0P1250,
    ADC_RANGE_0P2500,
    ADC_RANGE_0P1250,
    ADC_RANGE_0P1250,
    ADC_RANGE_0P2500,
    ADC_RANGE_0P1250,
    ADC_RANGE_0P1250,
    MCU_RANGE_DIRECT,
};
static const float ChDivider[ADC_CHANNELS + MCU_CHANNELS] = {
    1.0f,
    1.0f,
    2.0f,
    1.0f,
    1.0f,
    2.0f,
    1.0f,
    1.0f,
    2.0f,
};

static SPI_HandleTypeDef * hspi;
static ADC_HandleTypeDef * hadc;

static uint8_t tx[32] __attribute__((aligned(32)));
static uint8_t rx[32] __attribute__((aligned(32)));

static volatile uint8_t semTx = 0;
static volatile uint8_t semRx = 0;
static volatile uint8_t semAdcCplt = 0;
static uint32_t AdcWritePos = 0;
static uint8_t AdcChannel = 0;
static uint8_t AdcFirstTime = 1;
static uint8_t McuFirstTime = 1;
static uint32_t McuWritePos = 0;


void ADC_ErrorCallback(SPI_HandleTypeDef * _hspi)
{
  if(_hspi == hspi) {

  }
}

void ADC_TxCpltCallback(SPI_HandleTypeDef * _hspi)
{
  if(_hspi == hspi) {
    semTx = 1;
  }
}

void ADC_RxCpltCallback(SPI_HandleTypeDef * _hspi)
{
  if(_hspi == hspi) {
    semRx = 1;
  }
}

void ADC_TxRxCpltCallback(SPI_HandleTypeDef * _hspi)
{
  if(_hspi == hspi) {
    semTx = 1;
    semRx = 1;
  }
}

void ADC_MCU_ConvCpltCallback(ADC_HandleTypeDef * _hadc)
{
  uint16_t data;
  if(_hadc == hadc) {
    data = HAL_ADC_GetValue(hadc);

    if(McuFirstTime) {
      McuFirstTime = 0;
      for(int i = 0; i < ADC_BUFFER_SIZE; i++) {
        AdcBuffer[ADC_CHANNELS + 0][i] = data;
      }
    }
    else {
      AdcBuffer[ADC_CHANNELS + 0][McuWritePos] = data;
      if(++McuWritePos >= ADC_BUFFER_SIZE)
        McuWritePos = 0;
    }

  }
}

static uint8_t waitTxCplt(void)
{
  if(semTx) {
    semTx = 0;
    return 1;
  }
  return 0;
}

static uint8_t waitRxCplt(void)
{
  if(semRx) {
    semRx = 0;
    return 1;
  }
  return 0;
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

static uint8_t waitAdcCplt(void)
{
  if(semAdcCplt) {
    semAdcCplt = 0;
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
  SPI_NSS_OFF();
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
  SPI_NSS_OFF();
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
  SPI_NSS_OFF();
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

HAL_StatusTypeDef ADC_Init(SPI_HandleTypeDef * _hspi, ADC_HandleTypeDef * _hadc)
{
  HAL_StatusTypeDef result = HAL_OK;
  SCB_CleanDCache_by_Addr((uint32_t*)tx, sizeof(tx));
  SCB_CleanDCache_by_Addr((uint32_t*)rx, sizeof(rx));

  hspi = _hspi;
  hadc = _hadc;

  result = SPI_SendCommand(0x8500); //RST command
  if(result != HAL_OK)
    return result;

  //Enable all channels
  result = SPI_WriteRegister(0x02, 0x00);
  if(result != HAL_OK)
    return result;

  //All channels in sequence mode
  result = SPI_WriteRegister(0x01, 0xFF);
  if(result != HAL_OK)
    return result;

  //SDO format: 011, dev address: 0
  result = SPI_WriteRegister(0x03, 0x03);
  if(result != HAL_OK)
    return result;

  for(int i = 0; i < ADC_CHANNELS; i++) {
    result = SPI_WriteRegister(0x05 + i, ChRange[i]);
    if(result != HAL_OK)
      return result;
  }

  result = SPI_SendCommand(0xA000); //AUTO_RST command
  if(result != HAL_OK)
    return result;

  return result;
}

HAL_StatusTypeDef ADC_Fast_Loop(void)
{
  static HAL_StatusTypeDef result = HAL_OK;
  static uint8_t state = 0;
  uint16_t data;

  switch(state) {
    case 0:
      *((uint32_t *)tx) = 0;
      SCB_CleanDCache_by_Addr((uint32_t*)tx, 4);
      SPI_NSS_ON();
      HAL_SPI_TransmitReceive_DMA(hspi, tx, rx, 4);
      state++;
      break;
    case 1:
      if(waitTxRxCplt())
      {
        SPI_NSS_OFF();
        SCB_InvalidateDCache_by_Addr((uint32_t*)rx, 4);
        data = rx[3] << 8 | rx[4];

        if(!AdcFirstTime)
          AdcBuffer[AdcChannel][AdcWritePos] = data;
        else {
          for(int i = 0; i < ADC_BUFFER_SIZE; i++) {
            AdcBuffer[AdcChannel][i] = data;
          }
        }

        if(++AdcChannel >= ADC_CHANNELS) {
          if(++AdcWritePos >= ADC_BUFFER_SIZE)
            AdcWritePos = 0;
          AdcChannel = 0;
          AdcFirstTime = 0;
        }

        state = 0;
      }
      break;

    default:
      state = 0;
      break;
  }

  return result;
}

HAL_StatusTypeDef ADC_Slow_Loop(void)
{
  static HAL_StatusTypeDef result = HAL_OK;
  uint32_t data;

  for(int i = 0; i < ADC_CHANNELS + MCU_CHANNELS; i++) {
    data = 0;
    for(int j = 0; j < ADC_BUFFER_SIZE; j++)
      data += AdcBuffer[i][j];
    ChData[i] = data / ADC_BUFFER_SIZE;
    AdcVoltages[i] = ADC_Convert(i) * ChDivider[i];
  }

  return result;
}

inline float ADC_GetVoltage(eAdcChannel channel)
{
  if(channel < ADC_CHANNELS + MCU_CHANNELS)
    return AdcVoltages[channel];
  return 0.f;
}

