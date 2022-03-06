/*
 * config.h
 *
 *  Created on: 2 февр. 2021 г.
 *      Author: VHEMaster
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#include "main.h"
#include "structs.h"

int8_t flash_page_load(void *buffer, uint32_t size, uint8_t page);
int8_t flash_page_save(const void *buffer, uint32_t size, uint8_t page);
void flash_fast_loop(void);

#endif /* INC_FLASH_H_ */
