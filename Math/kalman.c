/*
 * interpolation.c
 *
 *  Created on: 10 мар. 2022 г.
 *      Author: VHEMaster
 */

#include "defines.h"
#include "kalman.h"


sMathKalmanCtx math_kalman_init(float q, float r, float f, float h)
{
  sMathKalmanCtx ctx = {0};

  ctx.Q = q;
  ctx.R = r;
  ctx.F = f;
  ctx.H = h;

  return ctx;
}

INLINE void math_kalman_set_state(sMathKalmanCtx *ctx, float state, float covariance)
{
  if(!ctx) {
    return;
  }

  ctx->State = state;
  ctx->Covariance = covariance;

}

INLINE float math_kalman_correct(sMathKalmanCtx *ctx, float value)
{
  if(!ctx) {
    return value;
  }

  //time update - prediction
  ctx->X0 = ctx->F * ctx->State;
  ctx->P0 = ctx->F * ctx->Covariance * ctx->F + ctx->Q;

  //measurement update - correction
  ctx->K = ctx->H * ctx->P0 / (ctx->H * ctx->P0 * ctx->H + ctx->R);
  ctx->State = ctx->X0 + ctx->K * (value - ctx->H * ctx->X0);
  ctx->Covariance = (1.0f - ctx->K * ctx->H) * ctx->P0;

  return ctx->State;

}
