/*
 * speed.c
 *
 *  Created on: Mar 5, 2022
 *      Author: VHEMaster
 */

#include "speed.h"
#include "delay.h"
#include <string.h>

#define IRQ_SIZE 27

typedef struct {
    float input_corrective;
    float output_corrective;
    uint32_t irq_data[IRQ_SIZE];
    volatile uint32_t pulse_last;
    uint8_t rotates;
    float speed;
    float speed_prev;
    __IO uint32_t *timebase;
    float acceleration;
    uint32_t acceleration_time;
}sSpeedCtx;


static sSpeedCtx gSpeedCtx;
static TIM_HandleTypeDef *htim;
static uint32_t tim_channel;

void speed_init(__IO uint32_t *timebase, TIM_HandleTypeDef *_htim, uint32_t channel)
{
  htim = _htim;
  tim_channel = channel;

  memset(&gSpeedCtx, 0, sizeof(gSpeedCtx));
  gSpeedCtx.input_corrective = 1.0f;
  gSpeedCtx.output_corrective = 1.0f;

  gSpeedCtx.timebase = timebase;
  for(int i = 0; i < IRQ_SIZE; i++) {
    gSpeedCtx.irq_data[i] = 0;
  }
}

#ifdef SIMULATION
void speed_emulate(uint32_t timestamp, float speed)
{
  static uint32_t time_last = 0;
  static float time_prev = 0;
  uint32_t diff = DelayDiff(timestamp, time_last);

  float time_needed = 1000000.0f / (speed / 3.6f * 6.0f * 2.0f);

  if(speed == 0)
    time_prev = timestamp;

  if(DelayDiff(timestamp, time_prev) >= time_needed) {
    time_prev += time_needed;
    if(time_prev > (float)(DelayMask))
      time_prev -= (float)(DelayMask);
    speed_exti(timestamp);
  }
}
#endif

void speed_exti(uint32_t timestamp)
{
  int i;
  const float accel_koff = 0.01f;
  float acceleration;
  float average = 0;

  gSpeedCtx.pulse_last = timestamp;

  // Set that we are speeding already
  if(!gSpeedCtx.rotates) {
    gSpeedCtx.speed = 1.0f;
    gSpeedCtx.rotates = 1;
  }

  for(i = 1; i < IRQ_SIZE; i++)
    gSpeedCtx.irq_data[i - 1] = gSpeedCtx.irq_data[i];
  gSpeedCtx.irq_data[IRQ_SIZE - 1] = timestamp;
  if(gSpeedCtx.irq_data[0] == 0) {
    return;
  }

  for(i = 1; i < IRQ_SIZE; i++)
    average += DelayDiff(gSpeedCtx.irq_data[i], gSpeedCtx.irq_data[i - 1]);
  average /= (float)(IRQ_SIZE - 1);

  // Since both edges are used
  average *= 2.0f;

  gSpeedCtx.speed = 600000.0f / average; // 1000000.0f / (average / 3.6f * 6.0f)
  gSpeedCtx.speed *= gSpeedCtx.input_corrective;

  htim->Instance->PSC = average * 0.1f / gSpeedCtx.output_corrective;
  if(TIM_CHANNEL_STATE_GET(htim, tim_channel) != HAL_TIM_CHANNEL_STATE_BUSY)
    HAL_TIM_PWM_Start(htim, tim_channel);

  acceleration = ((gSpeedCtx.speed - gSpeedCtx.speed_prev) * 0.27777778f) / (DelayDiff(timestamp, gSpeedCtx.acceleration_time) * 0.000001f);
  gSpeedCtx.acceleration = acceleration * accel_koff + gSpeedCtx.acceleration * (1.0f - accel_koff);
  gSpeedCtx.acceleration_time = timestamp;
  gSpeedCtx.speed_prev = gSpeedCtx.speed;
}

void speed_loop(void)
{
  uint32_t pulse_last = gSpeedCtx.pulse_last;
  uint32_t now = *gSpeedCtx.timebase;
  if(gSpeedCtx.rotates && DelayDiff(now, pulse_last) > 200000) {
    for(int i = 0; i < IRQ_SIZE; i++) {
      gSpeedCtx.irq_data[i] = 0;
    }
    gSpeedCtx.pulse_last = now;
    gSpeedCtx.acceleration_time = now - 500000;
    gSpeedCtx.rotates = 0;
    gSpeedCtx.speed = 0;
    gSpeedCtx.speed_prev = 0;
    htim->Instance->PSC = 0xFFFF;
    if(TIM_CHANNEL_STATE_GET(htim, tim_channel) != HAL_TIM_CHANNEL_STATE_READY)
      HAL_TIM_PWM_Stop(htim, tim_channel);
  }
}

uint8_t speed_isrotates(void)
{
  return gSpeedCtx.rotates;
}

float speed_getspeed(void)
{
  return gSpeedCtx.speed;
}

float speed_getacceleration(void)
{
  return gSpeedCtx.acceleration;
}

void speed_setinputcorrective(float corrective)
{
  gSpeedCtx.input_corrective = corrective;
}

void speed_setoutputcorrective(float corrective)
{
  gSpeedCtx.output_corrective = corrective;
}
