/*
 * pid.c
 *
 *  Created on: 10 мар. 2022 г.
 *      Author: VHEMaster
 */

#include "arm_math.h"
#include "pid.h"
#include "delay.h"
#include <string.h>
#include <float.h>

CMSIS_INLINE __INLINE void math_pid_reset(sMathPid *pid)
{
  pid->Current = 0;
  pid->Target = 0;
  pid->Error = 0;
  pid->P = 0;
  pid->I = 0;
  pid->D = 0;
}

CMSIS_INLINE __INLINE void math_pid_set_target(sMathPid *pid, float target)
{
  pid->Target = target;
}

CMSIS_INLINE __INLINE void math_pid_set_koffs(sMathPid *pid, float Kp, float Ki, float Kd)
{
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
}

CMSIS_INLINE __INLINE void math_pid_set_clamp(sMathPid *pid, float from, float to)
{
  pid->ClampFrom = from;
  pid->ClampTo = to;
}

CMSIS_INLINE __INLINE void math_pid_init(sMathPid *pid)
{
  memset(pid, 0, sizeof(sMathPid));
  pid->Kp = 1.0f;
  pid->ClampFrom = FLT_MIN;
  pid->ClampTo = FLT_MAX;
}

CMSIS_INLINE __INLINE float math_pid_update(sMathPid *pid, float input, unsigned int time)
{
  float error = input - pid->Target;
  float output;
  float P,I,D;
  uint32_t dt = DelayDiff(time, pid->LastTime);
  float dtf = dt * 0.000001f; //  / 1000000.0f
  pid->LastTime = time;

  pid->Error = error;
  P = pid->P = error * pid->Kp;
  I = pid->I += error * dtf * pid->Ki;
  D = pid->D = -(input - pid->Current) / dtf * pid->Kd;

  pid->Current = input;

  output = P + I + D;
  pid->Output = output;

  if(output > pid->ClampTo)
    output = pid->ClampTo;
  else if(output < pid->ClampFrom)
    output = pid->ClampFrom;

  return output;
}
