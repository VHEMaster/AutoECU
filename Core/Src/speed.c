/*
 * speed.c
 *
 *  Created on: Mar 5, 2022
 *      Author: VHEMaster
 */

#include "speed.h"
#include "delay.h"

#define IRQ_SIZE 4
static uint32_t speed_irq_data[IRQ_SIZE];
static volatile uint32_t speed_pulse_last = 0;
static volatile uint8_t speed_rotates = 0;
static volatile float speed_speed = 0;
static volatile float speed_prev = 0;
static volatile uint32_t *speed_timebase;
static volatile float speed_input_corrective = 1.0f;
static volatile float speed_output_corrective = 1.0f;
static volatile float speed_acceleration = 0;
static volatile uint32_t speed_acceleration_time = 0;

static TIM_HandleTypeDef *htim;
static uint32_t tim_channel;

void speed_init(volatile uint32_t *timebase, TIM_HandleTypeDef *_htim, uint32_t channel)
{
  htim = _htim;
  tim_channel = channel;
  speed_timebase = timebase;
  for(int i = 0; i < IRQ_SIZE; i++) {
    speed_irq_data[i] = 0;
  }
}

#ifdef SIMULATION
void speed_emulate(uint32_t timestamp, float speed)
{
  static uint32_t time_last = 0;
  static float time_prev = 0;
  uint32_t diff = DelayDiff(timestamp, time_last);

  float time_needed = 1000000.0f / (speed / 3.6f * 6.0f);

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
  const float accel_koff = 0.1f;
  float acceleration;
  float average = 0;
  //float pwm_speed;

  speed_pulse_last = timestamp;
  for(i = 1; i < IRQ_SIZE; i++)
    speed_irq_data[i - 1] = speed_irq_data[i];
  speed_irq_data[IRQ_SIZE - 1] = timestamp;
  if(speed_irq_data[0] == 0) {
    return;
  }
  speed_rotates = 1;


  for(i = 1; i < IRQ_SIZE; i++)
    average += DelayDiff(speed_irq_data[i], speed_irq_data[i - 1]);
  average /= (float)(IRQ_SIZE - 1);

  speed_speed = 1000000.0f / (average / 3.6f * 6.0f);
  speed_speed *= speed_input_corrective;

  htim->Instance->PSC = average * 0.1f / speed_output_corrective;
  if(TIM_CHANNEL_STATE_GET(htim, tim_channel) != HAL_TIM_CHANNEL_STATE_BUSY)
    HAL_TIM_PWM_Start(htim, tim_channel);


  acceleration = ((speed_speed - speed_prev) * 0.27777778f) / (DelayDiff(timestamp, speed_acceleration_time) * 0.000001f);
  speed_acceleration = acceleration * accel_koff + speed_acceleration * (1.0f - accel_koff);
  speed_acceleration_time = timestamp;
  speed_prev = speed_speed;
}

void speed_loop(void)
{
  uint32_t pulse_last = speed_pulse_last;
  uint32_t now = *speed_timebase;
  if(speed_rotates && DelayDiff(now, pulse_last) > 1000000) {
    for(int i = 0; i < IRQ_SIZE; i++) {
      speed_irq_data[i] = 0;
    }
    speed_pulse_last = now;
    speed_acceleration_time = now - 500000;
    speed_rotates = 0;
    speed_speed = 0;
    speed_prev = 0;
    htim->Instance->PSC = 0xFFFF;
    if(TIM_CHANNEL_STATE_GET(htim, tim_channel) != HAL_TIM_CHANNEL_STATE_READY)
      HAL_TIM_PWM_Stop(htim, tim_channel);
  }
}

uint8_t speed_isrotates(void)
{
  return speed_rotates;
}

float speed_getspeed(void)
{
  return speed_speed;
}

float speed_getacceleration(void)
{
  return speed_acceleration;
}

void speed_setinputcorrective(float corrective)
{
  speed_input_corrective = corrective;
}

void speed_setoutputcorrective(float corrective)
{
  speed_output_corrective = corrective;
}
