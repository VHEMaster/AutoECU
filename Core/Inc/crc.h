#include "main.h"

#ifndef INC_CRC_H_
#define INC_CRC_H_

#define CRC_HW
//#define CRC_SW

uint8_t CRC8_Generate(uint8_t * input, uint32_t size);
uint16_t CRC16_Generate(uint8_t * input, uint32_t size);

#ifdef CRC_HW
void CRC16_Init(CRC_HandleTypeDef * hcrc);
#elif !defined(CRC_SW)
#error Must be defined CRC_HW or CRC_SW
#endif

uint16_t XOR_Generate(const uint8_t *input, uint32_t size);

#endif /* INC_CRC_H_ */
