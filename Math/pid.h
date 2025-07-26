/*
 * pid.h
 *
 *  Created on: 9 мар. 2022 г.
 *      Author: VHEMaster
 */

#ifndef PID_H_
#define PID_H_

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
    unsigned int LastTime;
}math_pid_ctx_t;

void math_pid_reset(math_pid_ctx_t *pid, unsigned int time);
void math_pid_set_target(math_pid_ctx_t *pid, float target);
void math_pid_set_koffs(math_pid_ctx_t *pid, float Kp, float Ki, float Kd);
void math_pid_set_clamp(math_pid_ctx_t *pid, float from, float to);
void math_pid_init(math_pid_ctx_t *pid);

float math_pid_update(math_pid_ctx_t *pid, float input, unsigned int time);

#endif /* PID_H_ */
