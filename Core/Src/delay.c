#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"
#include "delay.h"
#include "arm_math.h"

void DelayInit(void)
{

}

CMSIS_INLINE __INLINE void DelayUs(uint32_t val)
{
  uint32_t tickstart = Delay_Tick;
  while(DelayDiff(Delay_Tick, tickstart) < val) {}
}

CMSIS_INLINE __INLINE void DelayMs(uint32_t val)
{
  DelayUs(val * 1000);
}

CMSIS_INLINE __INLINE uint32_t DelayDiff(uint32_t a, uint32_t b)
{
  if(a >= b)
    return (a - b);
  return ((DelayMask - b) + a);
}

CMSIS_INLINE __INLINE uint32_t HAL_DelayDiff(uint32_t a, uint32_t b)
{
  if(a >= b)
    return (a - b);
  return ((0xFFFFFFFF - b) + a);
}


