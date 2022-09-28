#include "csps.h"
#include "delay.h"
#include "defines.h"
#include <math.h>
#include <string.h>

#define ANGLE_INITIAL (-114.0f)
#define IRQ_SIZE 5
#define DATA_SIZE 16

#ifdef DEBUG
static volatile float CspsAngleInitial = ANGLE_INITIAL;
#else
static const float CspsAngleInitial = ANGLE_INITIAL;
#endif

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

#ifdef CSPS_DYNAMIC_CORR
static float cpsp_dynamic_corr[116] = {0.0f};
#endif

static sCspsData csps_data_empty = {0};
static volatile uint32_t csps_pulse_last = 0;
static uint32_t cspc_irq_data[IRQ_SIZE] = {0};
static volatile uint8_t csps_found = 0;
static volatile uint8_t csps_rotates = 0;
static volatile uint8_t csps_running = 0;
static volatile uint8_t csps_phased = 0;
static volatile uint8_t csps_phase_found = 0;
static volatile uint32_t csps_last = 0;
static volatile float csps_errors = 0;
static volatile float csps_rpm = 0;
static volatile float csps_uspa = 0;
static volatile float csps_period = 0;
static volatile float csps_tsps_rel_pos = 0;

static volatile uint32_t csps_turns = 0;
static volatile uint32_t csps_turns_phase = 0;
static volatile uint32_t csps_halfturns = 0;

static TIM_HandleTypeDef *htim;
static uint32_t tim_channel;
static __IO uint32_t *csps_timebase = NULL;

static sCspsData CspsData[DATA_SIZE] = {{0}};
static sCspsData * volatile CspsDataPtr = &CspsData[0];

static float csps_cors_avg = 1.0f;
static float csps_cors_sum = 1.0f;
static volatile uint32_t ticks = 0;

static void csps_handle(uint32_t timestamp);

#ifdef DEBUG
volatile static uint32_t csps_irq_start = 0;
volatile static uint32_t csps_irq_end = 0;
volatile static uint16_t csps_irq_times = 0;
volatile static uint32_t csps_irq_time = 0;
volatile static float csps_irq_avg = 0;
volatile static float csps_irq_max = 0;
#endif

INLINE void csps_exti(uint32_t timestamp)
{
#ifdef DEBUG
    csps_irq_start = Delay_Tick;
#endif
      csps_handle(timestamp);
#ifdef DEBUG
    //For time measurement taken by the csps irq handler. No need to optimize anything here
    csps_irq_end = Delay_Tick;
    csps_irq_time = DelayDiff(csps_irq_end, csps_irq_start);

    if(csps_irq_avg == 0)
    csps_irq_avg = csps_irq_time;
    else if(csps_irq_times < 1000) {
    csps_irq_avg = csps_irq_avg * 0.99f + csps_irq_time * 0.01f;
    if(csps_irq_time > csps_irq_max)
      csps_irq_max = csps_irq_time;
    else csps_irq_max = csps_irq_max * 0.99f + csps_irq_time * 0.01f;
    csps_irq_times++;
    } else {
    csps_irq_avg = csps_irq_avg * 0.99999f + csps_irq_time * 0.00001f;
    if(csps_irq_time > csps_irq_max)
      csps_irq_max = csps_irq_time;
    else csps_irq_max = csps_irq_max * 0.99999f + csps_irq_time * 0.00001f;
    }
#endif
}

#ifdef SIMULATION
void csps_emulate(uint32_t timestamp, float rpm, uint8_t phased)
{
  static uint8_t phase = 0;
  static uint32_t step = 60;
  static float time_prev = 0;
  float period = 60000000 / rpm;
  float time_needed = period * csps_cors[step] / 120.0f;

  if(rpm == 0)
    time_prev = timestamp;

  if(DelayDiff(timestamp, time_prev) >= time_needed) {
    time_prev += time_needed;
    if(time_prev > (float)(DelayMask))
      time_prev -= (float)(DelayMask);
    csps_exti(timestamp);
    HAL_GPIO_TogglePin(TIM5_CH1_SENS_CSPS_GPIO_Port, TIM5_CH1_SENS_CSPS_Pin);
    if(++step >= 116)
      step = 0;
    if(step == 107) {
      phase ^= 1;
      if(phase) {
        HAL_GPIO_WritePin(TIM8_CH3_SENS_TSPS_GPIO_Port, TIM8_CH3_SENS_TSPS_Pin, GPIO_PIN_RESET);
        if(phased)
          csps_tsps_exti(timestamp);
      }
    }
    if(step == 12) {
      HAL_GPIO_WritePin(TIM8_CH3_SENS_TSPS_GPIO_Port, TIM8_CH3_SENS_TSPS_Pin, GPIO_PIN_SET);
    }

  }
}
#endif

void csps_init(__IO uint32_t *timebase, TIM_HandleTypeDef *_htim, uint32_t channel)
{
  float tval = 0;
  csps_timebase = timebase;
  tim_channel = channel;
  htim = _htim;
  for(int i = 0; i < 116; i++)
    tval += csps_cors[i];
  csps_cors_sum = (120.0f / tval) * 3.0f;
  csps_cors_avg = tval / 120.0f;

  csps_data_empty.AngleCur14 = 0;
  csps_data_empty.AngleCur23 = 0;
  csps_data_empty.AnglePrev14 = 0;
  csps_data_empty.AnglePrev23 = 0;
  csps_data_empty.DelayPrev = 0;
  csps_data_empty.DelayCur = 0;
  csps_data_empty.PhasedActive = 0;
  csps_data_empty.PhasedAngleCur = 0;
  csps_data_empty.PhasedAnglePrev = 0;
  csps_data_empty.RPM = 0;
  csps_data_empty.Period = 1000000.0;
  csps_data_empty.uSPA = 3000.0f;

#ifdef CSPS_DYNAMIC_CORR
  for(int i = 0; i < 116; i++) {
    cpsp_dynamic_corr[i] = csps_cors[i];
  }
#endif
}

INLINE void csps_tsps_exti(uint32_t timestamp)
{
  if(csps_found && csps_rotates) {
    csps_tsps_rel_pos = csps_getangle14(csps_data());
    csps_phase_found = 1;
  }
}

ITCM_FUNC void csps_handle(uint32_t timestamp)
{
  static float csps_angle14 = ANGLE_INITIAL, csps_angle23 = ANGLE_INITIAL + 180, csps_angle_phased = 0;
  static float cs14_p = 0, cs23_p = 0, cs_phased_p = 0;
  static float adder_prev = 3.0f;
  static float average_prev = 0;
  static uint8_t dataindex = 0;
  static uint32_t t1 = 0;
  static uint32_t t2 = 0;
  //static GPIO_PinState pin_prev = GPIO_PIN_RESET;
  //GPIO_PinState pin_state;
  static uint8_t found = 0;

  float rpm_koff = 1.0f / 10.0f;
  float angle_initial = CspsAngleInitial;

  uint32_t i, cur, prev;
  sCspsData data;
  float cs14, cs23, csph;
  float average = 0;
  float diff = 0;
  float adder = 0;

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

  if(!csps_rotates) {
    found = 0;
    t1 = 0;
    t2 = 0;
  }

  csps_rotates = 1;

  //if(found) {
    t1++;
  //}

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

      if(t1 == 116) {
        found = 1;
      }
      t1 = 0;
      csps_last = cur;

    }
  }
  else
  {
    t2 = 0;
  }

  if(t1 >= 116)
  {
    t1 = 1;
    csps_errors += 1.0f;
  }

  average_prev = average;

  if(found)
  {
    adder = csps_cors[t1] * csps_cors_sum;// * 3.0f;

    //adder = 3.0f;

    switch(t2)
    {
      case 2:
        csps_turns++;
        adder = angle_initial - csps_angle14;

        csps_angle14 = angle_initial;
        if (angle_initial > 0)
          csps_angle23 = angle_initial - 180.0f;
        else
          csps_angle23 = angle_initial + 180.0f;

        cs14_p = csps_angle14 - adder;
        cs23_p = csps_angle23 - adder;

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
          csps_angle14 = cs14_p;

        if((cs23_p - csps_angle23 > 0.0f || cs23_p - csps_angle23 < -90.0f) && cs23_p - csps_angle23 < 90.0f)
          csps_angle23 = cs23_p;
        break;
    }


    csph = csps_angle_phased + adder;
    if(csph > 360.0f) csps_angle_phased = csph - 720.0f;
    else csps_angle_phased = csph;

    cs_phased_p += adder_prev;
    if(cs_phased_p > 360.0f)
      cs_phased_p -= 720.0f;

    if((cs_phased_p - csps_angle_phased > 0.0f || cs_phased_p - csps_angle_phased < -180.0f) && cs_phased_p - csps_angle_phased < 180.0f)
      csps_angle_phased = cs_phased_p;

    adder_prev = adder;

    if(csps_rpm < 200.0f)
      rpm_koff = 1.0f / 3.0f;

    diff = (float)DelayDiff(cur, prev) / csps_cors[t1] * csps_cors_avg;

    if(csps_period > 1000000.0f)
      csps_period = 1000000.0f;

    csps_period = csps_period * (1.0f - rpm_koff) + (diff * 120.0f) * rpm_koff;
    csps_rpm = 60000000.0f / csps_period;

    csps_uspa = diff * 0.33333333f;

    if((csps_angle14 >= 0.0f && cs14_p < 0.0f) && (cs14_p > -90.0f && csps_angle14 < 90.0f)) {
      csps_halfturns++;
    }
    else if(csps_angle14 < -90.0f && cs14_p > 90.0f) {
      csps_halfturns++;
    }

    data.AngleCur14 = csps_angle14;
    data.AngleCur23 = csps_angle23;
    data.AnglePrev14 = cs14_p;
    data.AnglePrev23 = cs23_p;
    data.DelayPrev = prev;
    data.DelayCur = cur;
    data.RPM = csps_rpm;
    data.uSPA = csps_uspa;

    if(csps_phase_found) {
      csps_phase_found = 0;
      csps_phased = 1;
      csps_turns_phase = csps_turns;
      csps_angle_phased = csps_angle14;
      cs_phased_p = cs14_p;
    } else {
      if(csps_turns - csps_turns_phase > 5) {
        csps_phased = 0;
      }
    }

    data.PhasedActive = csps_phased;
    data.PhasedAngleCur = csps_angle_phased;
    data.PhasedAnglePrev = cs_phased_p;

    diff = DelayDiff(cur, prev);
#ifdef CSPS_DYNAMIC_CORR
    cpsp_dynamic_corr[t1] = cpsp_dynamic_corr[t1] * 0.95f + (120.0f / (csps_period / diff)) * 0.05f;
#endif

  }
  else
  {
    data = csps_data_empty;
    csps_period = 1000000.0f;
    csps_rpm = 0;
    csps_phase_found = 0;
  }

  data.Period = csps_period;
  csps_turns;
  csps_halfturns;

  CspsData[dataindex] = data;
  CspsDataPtr = &CspsData[dataindex];
  if(++dataindex >= DATA_SIZE)
    dataindex = 0;

  csps_found = found;
  if(t2 >= 2) t2 = 0;

  /*
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
  */

  uint32_t value = (uint32_t)((float)(60 * 100000) / csps_rpm * 0.5f);
  htim->Instance->PSC = value > 0xFFFF ? 0xFFFF : value;
  if(TIM_CHANNEL_STATE_GET(htim, tim_channel) != HAL_TIM_CHANNEL_STATE_BUSY)
    HAL_TIM_PWM_Start(htim, tim_channel);
}

ITCM_FUNC INLINE float csps_getangle14(sCspsData data)
{
  static float angle_prev = 0;
  float angle, acur, aprev, mult, cur;
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

ITCM_FUNC INLINE float csps_getangle23from14(float angle)
{
  if(!csps_rotates)
    return 0.0f;

  if(angle > 0.0f) angle -= 180.0f;
  else angle += 180.0f;
  return angle;
}

ITCM_FUNC INLINE float csps_getphasedangle_cy(sCspsData data, uint8_t cylinder, float angle)
{
  uint8_t phased = csps_isphased(data);
  switch(cylinder) {
    case 0 :
      angle += 0.0f;
      break;
    case 2 :
      angle -= 180.0f;
      break;
    case 3 :
      angle -= 360.0f;
      break;
    case 1 :
      angle -= 540.0f;
      break;
    default:
      angle = 0;
      break;
  }

  if(phased) {
    while(angle <= -360.0f)
      angle += 720.0f;
  } else {
    while(angle <= -180.0f)
      angle += 360.0f;
  }

  return angle;
}

ITCM_FUNC float csps_getphasedangle(sCspsData data)
{
  static float angle_prev = 0;
  float angle, acur, aprev, mult, cur;
  float now = *csps_timebase;

  if(!csps_rotates || !csps_found)
    return 0.0f;

  cur = DelayDiff(data.DelayCur, data.DelayPrev);
  now = DelayDiff(now, data.DelayPrev);

  acur = data.PhasedAngleCur;
  aprev = data.PhasedAnglePrev;

  if(acur < aprev)
    acur += 720.0f;

  angle = acur - aprev;
  mult = angle / cur;
  angle = mult * now + aprev;

  while(angle > 360.0f)
    angle -= 720.0f;

  if((angle - angle_prev < 0.0f && angle - angle_prev > -180.0f) || angle - angle_prev > 180.0f)
  {
    angle = angle_prev;
  }

  //Check for NaNs
  if(angle != angle)
    angle = 0.0f;

  angle_prev = angle;

  return angle;
}

ITCM_FUNC INLINE float csps_getrpm(sCspsData data)
{
  return data.RPM;
}

ITCM_FUNC INLINE float csps_getuspa(sCspsData data)
{
  return data.uSPA;
}

ITCM_FUNC INLINE float csps_getperiod(sCspsData data)
{
  return data.Period;
}

ITCM_FUNC INLINE uint8_t csps_isrunning(void)
{
  return csps_running;
}

ITCM_FUNC INLINE uint8_t csps_isrotates(void)
{
  return csps_rotates;
}

ITCM_FUNC INLINE uint8_t csps_isphased(sCspsData data)
{
  return data.PhasedActive;
}

ITCM_FUNC INLINE uint8_t csps_isfound(void)
{
  return csps_found;
}

ITCM_FUNC INLINE uint8_t csps_iserror(void)
{
  return csps_errors > 5.0f;
}

ITCM_FUNC INLINE uint32_t csps_gethalfturns(void)
{
  return csps_halfturns;
}

ITCM_FUNC INLINE uint32_t csps_getturns(void)
{
  return csps_turns;
}

ITCM_FUNC INLINE sCspsData csps_data(void)
{
  if(csps_found)
	  return *CspsDataPtr;
  return csps_data_empty;
}

ITCM_FUNC INLINE float csps_gettspsrelpos(void)
{
  if(csps_phased)
    return csps_tsps_rel_pos;
  return 0.0f;
}

void csps_loop(void)
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
    csps_phase_found = 0;
    csps_phased = 0;
    csps_running = 0;
    csps_period = 1000000.0f;
    htim->Instance->PSC = 0xFFFF;
    if(TIM_CHANNEL_STATE_GET(htim, tim_channel) != HAL_TIM_CHANNEL_STATE_READY)
      HAL_TIM_PWM_Stop(htim, tim_channel);
  }

  if(!csps_running) {
    if(csps_rpm > 550.0f) {
      csps_running = 1;
    }
  } else {
    if(csps_rpm < 400.0f) {
      csps_running = 0;
    }
  }

  if(DelayDiff(now, last_error_null) > 50000)
  {
    csps_errors *= 0.95f;
    last_error_null = now;
  }

}
