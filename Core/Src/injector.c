/*
 * injector.c
 *
 *  Created on: Mar 5, 2022
 *      Author: VHEMaster
 */

#include "injector.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t tim_channel;
    volatile uint8_t enabled;
}sInjector;

static sInjector injectors[InjectorCount] = {{0}};

inline void injector_irq(eInjector injector)
{
  if(injector < InjectorCount) {
    injectors[injector].htim->Init.Period = 0;
    injectors[injector].htim->Instance->ARR = 0;
    injectors[injector].enabled = 0;
  }
}

HAL_StatusTypeDef injector_register(eInjector injector, TIM_HandleTypeDef *htim, uint32_t channel)
{
  if(injector < InjectorCount) {
    injectors[injector].htim = htim;
    injectors[injector].tim_channel = channel;
    injectors[injector].enabled = 0;
  } else return HAL_ERROR;
  return HAL_OK;
}

inline HAL_StatusTypeDef injector_isenabled(eInjector injector, uint8_t *enabled)
{
  if(injector < InjectorCount && enabled) {
    *enabled = injectors[injector].enabled;
  } else return HAL_ERROR;
  return HAL_OK;
}

inline HAL_StatusTypeDef injector_enable(eInjector injector, uint32_t usec)
{
  if(injector < InjectorCount) {
    if(injectors[injector].enabled) {
      if(TIM_CHANNEL_STATE_GET(injectors[injector].htim, injectors[injector].tim_channel) != HAL_TIM_CHANNEL_STATE_READY) {
        HAL_TIM_PWM_Stop_IT(injectors[injector].htim, injectors[injector].tim_channel);
      }
    }
    injectors[injector].htim->Init.Period = usec;
    injectors[injector].htim->Instance->ARR = usec;
    if(HAL_TIM_PWM_Start_IT(injectors[injector].htim, injectors[injector].tim_channel) == HAL_OK) {
      injectors[injector].enabled = 1;
    } else return HAL_ERROR;
  } else return HAL_ERROR;
  return HAL_OK;
}

