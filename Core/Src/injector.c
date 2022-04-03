/*
 * injector.c
 *
 *  Created on: Mar 5, 2022
 *      Author: VHEMaster
 */

#include "injector.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    GPIO_TypeDef *port;
    uint16_t pin;
    volatile uint8_t enabled;
    uint32_t prescaler;
}sInjector;

static sInjector injectors[InjectorCount] = {{0}};

inline void injector_irq(eInjector injector)
{
  if(injector < InjectorCount) {
    injectors[injector].port->BSRR = injectors[injector].pin;
    injectors[injector].enabled = 0;
  }
}

HAL_StatusTypeDef injector_register(eInjector injector, TIM_HandleTypeDef *htim, GPIO_TypeDef *port, uint16_t pin)
{
  if(injector < InjectorCount) {
    injectors[injector].htim = htim;
    injectors[injector].port = port;
    injectors[injector].pin = pin;
    injectors[injector].enabled = 0;
    injectors[injector].port->BSRR = injectors[injector].pin;
    injectors[injector].prescaler = htim->Instance->PSC + 1;
  } else return HAL_ERROR;
  return HAL_OK;
}

HAL_StatusTypeDef injector_isenabled(eInjector injector, uint8_t *enabled)
{
  if(injector < InjectorCount && enabled) {
    *enabled = injectors[injector].enabled;
  } else return HAL_ERROR;
  return HAL_OK;
}

inline HAL_StatusTypeDef injector_enable(eInjector injector, uint32_t usec)
{
  uint32_t psc;
  if(injector < InjectorCount && injectors[injector].htim && injectors[injector].port) {
    __HAL_TIM_DISABLE_IT(injectors[injector].htim, TIM_IT_UPDATE);
    __HAL_TIM_DISABLE(injectors[injector].htim);
    psc = injectors[injector].prescaler;
    while(usec > 0x10000) {
      usec >>= 1;
      psc <<= 1;
    }
    injectors[injector].htim->Instance->PSC = psc - 1;
    injectors[injector].htim->Instance->ARR = usec;
    injectors[injector].htim->Instance->EGR |= 1;
    injectors[injector].enabled = 1;
    __HAL_TIM_CLEAR_IT(injectors[injector].htim, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_FLAG(injectors[injector].htim, TIM_FLAG_UPDATE);
    injectors[injector].port->BSRR = injectors[injector].pin << 16;
    __HAL_TIM_ENABLE_IT(injectors[injector].htim, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(injectors[injector].htim);
  }
  return HAL_OK;
}

