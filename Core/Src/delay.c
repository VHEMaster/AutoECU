#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"
#include "delay.h"

void DelayInit(void)
{

}

inline void DelayUs(uint32_t val)
{
  uint32_t tickstart = Delay_Tick;
  while(DelayDiff(Delay_Tick, tickstart) < val) {}
}

inline void DelayMs(uint32_t val)
{
  DelayUs(val * 1000);
}

inline uint32_t DelayDiff(uint32_t a, uint32_t b)
{
  if(a >= b)
    return (a - b);
  return ((DelayMask - b) + a);
}

inline uint32_t HAL_DelayDiff(uint32_t a, uint32_t b)
{
  if(a >= b)
    return (a - b);
  return ((0xFFFFFFFF - b) + a);
}

