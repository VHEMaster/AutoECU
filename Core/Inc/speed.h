/*
 * speed.h
 *
 *  Created on: Mar 5, 2022
 *      Author: VHEMaster
 */

#ifndef INC_SPEED_H_
#define INC_SPEED_H_

#include "main.h"

void speed_init(__IO uint32_t *timebase, TIM_HandleTypeDef *_htim, uint32_t channel);
void speed_loop(void);
void speed_exti(uint32_t timestamp);

uint8_t speed_isrotates(void);
float speed_getspeed(void);
void speed_setinputcorrective(float corrective);
void speed_setoutputcorrective(float corrective);

#endif /* INC_SPEED_H_ */
