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
static volatile uint32_t speed_rotates = 0;
static volatile uint32_t speed_speed = 0;
static volatile uint32_t *speed_timebase;

void speed_init(volatile uint32_t *timebase)
{
  speed_timebase = timebase;
  for(int i = 0; i < IRQ_SIZE; i++) {
    speed_irq_data[i] = 0;
  }
}

void speed_exti(uint32_t timestamp)
{
  int i;
  float average = 0;

  speed_pulse_last = timestamp;
  for(i = 1; i < IRQ_SIZE; i++)
    speed_irq_data[i - 1] = speed_irq_data[i];
  speed_irq_data[IRQ_SIZE - 1] = timestamp;
  if(speed_irq_data[0] == 0) {
    return;
  }
  speed_rotates = 1;


  for(i = 1; i < IRQ_SIZE; i++)
  {
    average += DelayDiff(speed_irq_data[i], speed_irq_data[i - 1]);
  }
  average /= (float)(IRQ_SIZE - 1);

  speed_speed = 1000000.0f / (average / 3.6f * 6.0f);
}

void speed_loop(void)
{
  uint32_t now = *speed_timebase;
  if(speed_rotates && DelayDiff(now, speed_pulse_last) > 1000000) {
    for(int i = 0; i < IRQ_SIZE; i++) {
      speed_irq_data[i] = 0;
      speed_pulse_last = now;
    }
    speed_rotates = 0;
    speed_speed = 0;
  }
}

float speed_getspeed(void)
{
  return speed_speed;
}
