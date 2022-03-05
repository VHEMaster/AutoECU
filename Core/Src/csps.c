#include "csps.h"
#include "delay.h"
#include <math.h>
#include <string.h>

#define ANGLE_INITIAL (-114.0f)
#define IRQ_SIZE 16
#define DATA_SIZE 16

typedef struct
{
  uint32_t DelayCur;
  uint32_t DelayPrev;
  float AngleCur14;
  float AnglePrev14;
  float AngleCur23;
  float AnglePrev23;
  float RPM;
  float uSPA;
}sCspsData;

static const float csps_cors[116] = {
    2.68668088f, 0.719637128f, 1.132915204f, 0.93333743f, 1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f,
    1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f,
    1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f,
    1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f,
    1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f,
    1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f,
    1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f,
    1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f,
    1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f,
    1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f,
    1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f,
    1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f,
    1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f,
    1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f, 1.01200111f, 0.987573024f,
    1.014849528f, 0.9962869f, 0.872279303f, 3.667010358f,
};

static float cpsp_dynamic_corr[116] = {0.0f};


static volatile uint32_t csps_pulse_last = 0;
static uint32_t cspc_irq_data[IRQ_SIZE] = {0};
static volatile uint8_t csps_found = 0;
static volatile uint8_t csps_rotates = 0;
static volatile uint32_t csps_last = 0;
static volatile float csps_errors = 0;
static volatile float csps_rpm = 0;
static volatile float csps_uspa = 0;
static volatile float csps_period = 0;
static volatile uint32_t *csps_timebase = NULL;

static sCspsData CspsData[DATA_SIZE];
static sCspsData * volatile CspsDataPtr = &CspsData[0];

static float csps_cors_avg = 1.0f;
static float csps_cors_sum = 1.0f;
static volatile uint32_t ticks = 0;

void csps_init(volatile uint32_t *timebase)
{
  float tval = 0;
  csps_timebase = timebase;
  for(int i = 0; i < 116; i++)
    tval += csps_cors[i];
  csps_cors_sum = (120.0f / tval) * 3.0f;
  csps_cors_avg = tval / 116.0f;

  for(int i = 0; i < 116; i++)
  {
    cpsp_dynamic_corr[i] = csps_cors[i];
  }
}


inline void csps_exti(uint32_t timestamp)
{
  static float csps_angle14 = 0, csps_angle23 = 0;
  static float cs14_p = 0, cs23_p = 0;
  static float average_prev = 0;
  static uint8_t dataindex = 0;
  static uint32_t t1 = 0;
  static uint32_t t2 = 0;
  //static GPIO_PinState pin_prev = GPIO_PIN_RESET;
  //GPIO_PinState pin_state;
  static uint8_t found = 0;

  float rpm_koff = 1.0f / 10.0f;

  uint32_t i, cur, prev;
  sCspsData data;
  float cs14, cs23;
  float average = 0;
  float diff = 0;

  cur = timestamp;

  /* TODO: is this code really needed?
  pin_state = HAL_GPIO_ReadPin(TIM5_CH1_SENS_CSPS_GPIO_Port, TIM5_CH1_SENS_CSPS_Pin);
  if(pin_prev == pin_state) {
    return;
  }

  pin_prev = pin_state;
  */

  csps_pulse_last = timestamp;
  for(i = 1; i < IRQ_SIZE; i++)
    cspc_irq_data[i - 1] = cspc_irq_data[i];
  cspc_irq_data[IRQ_SIZE - 1] = cur;
  if(cspc_irq_data[0] == 0) {
    return;
  }
  prev = cspc_irq_data[IRQ_SIZE - 2];
  csps_rotates = 1;

  if(found) {
    t1++;
  }

  for(i = 1; i < IRQ_SIZE; i++)
  {
    average += DelayDiff(cspc_irq_data[i], cspc_irq_data[i - 1]);
  }
  average /= (float)(IRQ_SIZE - 1);

  if(average / average_prev > 1.0f + 1.0f / IRQ_SIZE)
  {
    if(++t2 == 2)
    {
      ticks = t1;
      if(found && t1 != 116) {
        csps_errors += 1.0f;
      }

      t1 = 0;
      csps_last = cur;
      found = 1;

    }
  }
  else
  {
    t2 = 0;
    if(t1 >= 116)
    {
      t1 = 1;
      csps_errors += 1.0f;
    }
  }

  average_prev = average;

  if(found)
  {
    float adder = csps_cors[t1] * csps_cors_sum;// * 3.0f;
    static float adder_prev = 3.0f;

    //adder = 3.0f;

    switch(t2)
    {
      case 2:
        csps_angle14 = ANGLE_INITIAL;
        if (ANGLE_INITIAL > 0)
          csps_angle23 = ANGLE_INITIAL - 180.0f;
        else
          csps_angle23 = ANGLE_INITIAL + 180.0f;
        cs14_p = csps_angle14 - csps_cors[115] * csps_cors_sum;
        cs23_p = csps_angle23 - csps_cors[115] * csps_cors_sum;
        //adder = 9.0f;
        break;
      case 1:
        //adder = 9.0f;
        /* no break */
      default:
        cs14 = csps_angle14 + adder;
        if(cs14 > 180.0f) csps_angle14 = cs14 - 360.0f;
        else csps_angle14 = cs14;

        cs23 = csps_angle23 + adder;
        if(cs23 > 180.0f) csps_angle23 = cs23 - 360.0f;
        else csps_angle23 = cs23;

        cs14_p += adder_prev;
        if(cs14_p > 180.0f)
          cs14_p -= 360.0f;

        cs23_p += adder_prev;
        if(cs23_p > 180.0f)
          cs23_p -= 360.0f;

        if((cs14_p - csps_angle14 > 0.0f || cs14_p - csps_angle14 < -90.0f) && cs14_p - csps_angle14 < 90.0f)
        {
          csps_angle14 = cs14_p;
        }

        if((cs23_p - csps_angle23 > 0.0f || cs23_p - csps_angle23 < -90.0f) && cs23_p - csps_angle23 < 90.0f)
        {
          csps_angle23 = cs23_p;
        }
        break;
    }

    adder_prev = adder;

    if(csps_rpm < 200.0f)
      rpm_koff = 1.0f / 3.0f;

    diff = (float)DelayDiff(cur, prev) / csps_cors[t1] * csps_cors_avg;

    if(csps_period > 1000000.0f)
      csps_period = 1000000.0f;

    csps_period = csps_period * (1.0f - rpm_koff) + (diff * 120.0f) * rpm_koff;
    csps_rpm = 1000000.0f / csps_period * 60.0f;

    csps_uspa = diff / 3.0f;

    data.AngleCur14 = csps_angle14;
    data.AngleCur23 = csps_angle23;
    data.AnglePrev14 = cs14_p;
    data.AnglePrev23 = cs23_p;
    data.DelayPrev = prev;
    data.DelayCur = cur;
    data.RPM = csps_rpm;
    data.uSPA = csps_uspa;

    diff = DelayDiff(cur, prev);
    //cpsp_dynamic_corr[t1] = cpsp_dynamic_corr[t1] * 0.95f + (120.0f / (csps_period / diff)) * 0.05f;

  }
  else
  {
    data.AngleCur14 = 0;
    data.AngleCur23 = 0;
    data.AnglePrev14 = 0;
    data.AnglePrev23 = 0;
    data.DelayPrev = 0;
    data.DelayCur = 0;
    data.RPM = 0;
    data.uSPA = 3000.0f;
    csps_period = 1000000.0f;
    csps_rpm = 0;
  }

  CspsData[dataindex] = data;
  CspsDataPtr = &CspsData[dataindex];
  if(++dataindex >= DATA_SIZE)
    dataindex = 0;

  csps_found = found;
  if(t2 >= 2) t2 = 0;

  const float tach_duty_cycle = 0.25f;
  float angle_tach = csps_angle14;

  if(angle_tach >= 0.0f && angle_tach < 180.0f * tach_duty_cycle)
    HAL_GPIO_WritePin(TACHOMETER_GPIO_Port, TACHOMETER_Pin, GPIO_PIN_SET);
  else if(angle_tach >= 180.0f * tach_duty_cycle)
    HAL_GPIO_WritePin(TACHOMETER_GPIO_Port, TACHOMETER_Pin, GPIO_PIN_RESET);
  else if(angle_tach < -180.0f + 180.0f * tach_duty_cycle)
    HAL_GPIO_WritePin(TACHOMETER_GPIO_Port, TACHOMETER_Pin, GPIO_PIN_SET);
  else if(angle_tach >= -180.0f + 180.0f * tach_duty_cycle && angle_tach < 0.0f)
    HAL_GPIO_WritePin(TACHOMETER_GPIO_Port, TACHOMETER_Pin, GPIO_PIN_RESET);
}

inline float csps_getangle14(void)
{
  static float angle_prev = 0;
  float angle, acur, aprev, mult, cur;
  sCspsData data = *CspsDataPtr;
  float now = *csps_timebase;

  if(!csps_rotates || !csps_found)
    return 0.0f;

  cur = DelayDiff(data.DelayCur, data.DelayPrev);
  now = DelayDiff(now, data.DelayPrev);

  acur = data.AngleCur14;
  aprev = data.AnglePrev14;

  if(acur < aprev)
    acur += 360.0f;

  angle = acur - aprev;
  mult = angle / cur;
  angle = mult * now + aprev;

  while(angle > 180.0f)
    angle -= 360.0f;

  if((angle - angle_prev < 0.0f && angle - angle_prev > -90.0f) || angle - angle_prev > 90.0f)
  {
    angle = angle_prev;
  }

  //Check for NaNs
  if(angle != angle)
    angle = 0.0f;

  angle_prev = angle;

  return angle;
}

inline float csps_getangle23from14(float angle)
{
  if(!csps_rotates)
    return 0.0f;

  if(angle > 0.0f) angle -= 180.0f;
  else angle += 180.0f;
  return angle;
}

inline float csps_getrpm(void)
{
  return csps_rpm;
}

inline float csps_getuspa(void)
{
  return csps_uspa;
}

inline float csps_getperiod(void)
{
  return csps_period;
}

inline uint8_t csps_isrotates(void)
{
  return csps_rotates;
}

inline uint8_t csps_isfound(void)
{
  return csps_found;
}

uint8_t csps_iserror(void)
{
  return csps_errors > 3.0f;
}

inline void csps_loop(void)
{
  static uint32_t last_error_null = 0;

  uint32_t pulse_value = csps_pulse_last;
  uint32_t now = *csps_timebase;

  if(DelayDiff(now, pulse_value) > 50000)
  {
    for(int i = 0; i < IRQ_SIZE; i++)
      cspc_irq_data[i] = 0;
    csps_found = 0;
    csps_rpm = 0;
    csps_rotates = 0;
    csps_period = 1.0f / csps_rpm;
    HAL_GPIO_WritePin(TACHOMETER_GPIO_Port, TACHOMETER_Pin, GPIO_PIN_RESET);
  }

  if(DelayDiff(now, last_error_null) > 50000)
  {
    csps_errors *= 0.95f;
    last_error_null = now;
  }

}
