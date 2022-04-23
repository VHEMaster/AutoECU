#include "crc.h"
#include "defines.h"

#define POLY 0x8005 // CRC-16-MAXIM (IBM) (or 0xA001)

static uint16_t CRC16_Generate_SW(uint8_t * input, uint32_t size)
{
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < size; pos++)
  {
      crc ^= input[pos];
      for (int i = 8; i != 0; i--)
      {
          if ((crc & 0x0001) != 0)
          {
              crc >>= 1;
              crc ^= __RBIT(POLY) >> 16;
          }
          else crc >>= 1;
      }
  }
  return crc;
}

static CRC_HandleTypeDef *handle_crc;
static volatile uint8_t CRC_IsBusy = 0;

void CRC16_Init(CRC_HandleTypeDef *hcrc)
{
  handle_crc = hcrc;

  hcrc->Instance = CRC;
  hcrc->Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
  hcrc->Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
  hcrc->Init.GeneratingPolynomial = POLY;
  hcrc->Init.CRCLength = CRC_POLYLENGTH_16B;
  hcrc->Init.InitValue = 0xFFFF;
  hcrc->Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;
  hcrc->Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;
  hcrc->InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(hcrc) != HAL_OK) {
    Error_Handler();
  }
}

INLINE uint8_t CRC16_IsBusy(void)
{
  return CRC_IsBusy;
}

INLINE uint16_t CRC16_Generate(uint8_t *input, uint32_t size)
{
  uint16_t result = 0;
  if (handle_crc != NULL && !CRC_IsBusy) {
    CRC_IsBusy = 1;
    result = HAL_CRC_Calculate(handle_crc, (uint32_t *)input, size);
    CRC_IsBusy = 0;
  } else {
    result = CRC16_Generate_SW(input, size);
  }
  return result;
}

INLINE uint8_t CRC8_Generate(uint8_t *input, uint32_t size)
{
  uint16_t result = CRC16_Generate(input, size);
  return (result & 0xFF) ^ (result >> 8);
}

