/*
 * interpolation.h
 *
 *  Created on: 10 мар. 2022 г.
 *      Author: VHEMaster
 */

#ifndef INTERPOLATION_H_
#define INTERPOLATION_H_

#include "stdint.h"

typedef struct {
    float values[2];
    uint32_t indexes[2];
    uint32_t size;
    float input;
    float mult;
}sMathInterpolateInput;

void math_interpolate_test(void);

sMathInterpolateInput math_interpolate_input(float value, const float *table, uint32_t size);
float math_interpolate_1d(sMathInterpolateInput input, const float *table);
float math_interpolate_2d(sMathInterpolateInput input_x, sMathInterpolateInput input_y, uint32_t y_size, const float (*table)[y_size]);

float math_interpolate_1d_int16(sMathInterpolateInput input, const int16_t *table);
float math_interpolate_2d_int16(sMathInterpolateInput input_x, sMathInterpolateInput input_y, uint32_t y_size, const int16_t (*table)[y_size]);

float math_interpolate_1d_set_int16(sMathInterpolateInput input, int16_t *table, float new_value);
float math_interpolate_2d_set_int16(sMathInterpolateInput input_x, sMathInterpolateInput input_y,
    uint32_t y_size, int16_t (*table)[y_size], float new_value);

#endif /* INTERPOLATION_H_ */
