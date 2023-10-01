/*
 * interpolation.h
 *
 *  Created on: 10 мар. 2022 г.
 *      Author: VHEMaster
 */

#ifndef INTERPOLATION_H_
#define INTERPOLATION_H_

typedef struct {
    float X0;
    float P0;
    float F;
    float Q;
    float H;
    float R;
    float State;
    float Covariance;
    float K;
}sMathKalmanCtx;

sMathKalmanCtx math_kalman_init(float q, float r, float f, float h);
void math_kalman_set_state(sMathKalmanCtx *ctx, float state, float covariance);
float math_kalman_correct(sMathKalmanCtx *ctx, float value);

#endif /* INTERPOLATION_H_ */
