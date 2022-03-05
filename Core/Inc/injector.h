/*
 * injector.h
 *
 *  Created on: Mar 5, 2022
 *      Author: VHEMaster
 */

#ifndef INC_INJECTOR_H_
#define INC_INJECTOR_H_

#include "main.h"

typedef enum {
  InjectorCy1 = 0,
  InjectorCy2,
  InjectorCy3,
  InjectorCy4,
  InjectorCount
}eInjector;

void injector_irq(eInjector injector);
void injector_register(eInjector injector, TIM_HandleTypeDef *htim, uint32_t channel);
HAL_StatusTypeDef injector_enable(eInjector injector, uint32_t usec);

#endif /* INC_INJECTOR_H_ */
