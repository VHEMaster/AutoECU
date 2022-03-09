/*
 * pid.h
 *
 *  Created on: 9 мар. 2022 г.
 *      Author: VHEMaster
 */

#ifndef PID_H_
#define PID_H_

#include "arm_math.h"
#include "delay.h"
#include <string.h>
#include <float.h>

typedef struct {
    float ClampFrom;
    float ClampTo;
    float Kp;
    float Ki;
    float Kd;
    float P;
    float I;
    float D;
    float Current;
    float Target;
    float Output;
    float Error;
    uint32_t LastTime;
}sEcuPid;

CMSIS_INLINE __STATIC_INLINE void ecu_pid_reset(sEcuPid *pid)
{
  pid->Current = 0;
  pid->Target = 0;
  pid->Error = 0;
}

CMSIS_INLINE __STATIC_INLINE void ecu_pid_set_target(sEcuPid *pid, float target)
{
  pid->Target = target;
}

CMSIS_INLINE __STATIC_INLINE void ecu_pid_set_koffs(sEcuPid *pid, float Kp, float Ki, float Kd)
{
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
}

CMSIS_INLINE __STATIC_INLINE void ecu_pid_set_clamp(sEcuPid *pid, float from, float to)
{
  pid->ClampFrom = from;
  pid->ClampTo = to;
}

CMSIS_INLINE __STATIC_INLINE void ecu_pid_init(sEcuPid *pid)
{
  memset(pid, 0, sizeof(sEcuPid));
  pid->Kp = 1.0f;
  pid->ClampFrom = FLT_MIN;
  pid->ClampTo = FLT_MAX;
}

CMSIS_INLINE __STATIC_INLINE float ecu_pid_update(sEcuPid *pid, float input, uint32_t time)
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

#endif /* PID_H_ */
