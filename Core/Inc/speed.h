/*
 * speed.h
 *
 *  Created on: Mar 5, 2022
 *      Author: VHEMaster
 */

#ifndef INC_SPEED_H_
#define INC_SPEED_H_

#include "main.h"

void speed_init(volatile uint32_t *timebase);
void speed_loop(void);
void speed_exti(uint32_t timestamp);

#endif /* INC_SPEED_H_ */
