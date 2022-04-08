#include "main.h"

#ifndef INC_CRC_H_
#define INC_CRC_H_


uint8_t CRC8_Generate(uint8_t * input, uint32_t size);
uint16_t CRC16_Generate(uint8_t * input, uint32_t size);
uint8_t CRC16_IsBusy(void);

void CRC16_Init(CRC_HandleTypeDef * hcrc);

#endif /* INC_CRC_H_ */
