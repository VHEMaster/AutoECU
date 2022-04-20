/*
 * ecu.h
 *
 *  Created on: Mar 4, 2022
 *      Author: VHEMaster
 */

#ifndef INC_ECU_H_
#define INC_ECU_H_

#include "main.h"
#include "xCommand.h"

void ecu_init(void);
void ecu_loop(void);
void ecu_irq_slow_loop(void);
void ecu_irq_fast_loop(void);
void ecu_parse_command(eTransChannels xChaSrc, uint8_t * msgBuf, uint32_t length);
void ecu_hardfault_handle(void);

#endif /* INC_ECU_H_ */
