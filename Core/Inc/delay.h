#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"

#define DelayTimer TIM5

#define Delay_Tick (DelayTimer->CNT)
#define DelayMask 0x03FFFFFF

void DelayInit(TIM_HandleTypeDef *_htim);
void DelayUs(uint32_t micros);
void DelayMs(uint32_t millis);
uint32_t DelayDiff(uint32_t a, uint32_t b);
uint32_t HAL_DelayDiff(uint32_t a, uint32_t b);
