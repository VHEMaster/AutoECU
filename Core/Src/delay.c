#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"
#include "delay.h"
#include "defines.h"

static TIM_HandleTypeDef *htim;

void DelayInit(TIM_HandleTypeDef *_htim)
{
  htim = _htim;
  HAL_TIM_Base_Start(htim);
}

INLINE void DelayUs(uint32_t val)
{
  uint32_t tickstart = Delay_Tick;
  while(DelayDiff(Delay_Tick, tickstart) < val) {}
}

INLINE void DelayMs(uint32_t val)
{
  DelayUs(val * 1000);
}

INLINE uint32_t DelayDiff(uint32_t a, uint32_t b)
{
  if(a >= b)
    return (a - b);
  return ((DelayMask - b) + a);
}

INLINE uint32_t HAL_DelayDiff(uint32_t a, uint32_t b)
{
  if(a >= b)
    return (a - b);
  return ((0xFFFFFFFF - b) + a);
}


