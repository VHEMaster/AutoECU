/*
 * interpolation.c
 *
 *  Created on: 10 мар. 2022 г.
 *      Author: VHEMaster
 */

#include "defines.h"
#include "interpolation.h"

STATIC_INLINE int math_binary_search(const float *array, int start_index, int end_index, float element)
{
  int iterations = 0;
  while(start_index <= end_index && ++iterations < 256) {
    int middle = start_index + ((end_index- start_index ) >> 1);
    if (array[middle] <= element && array[middle + 1] > element)
      return middle;
    if (array[middle + 1] <= element)
      start_index = middle + 1;
    else if(array[middle] >= element)
      end_index = middle - 1;
  }
  return -1;
}

INLINE sMathInterpolateInput math_interpolate_input(float value, const float *table, uint32_t size)
{
  sMathInterpolateInput result = {0};
  int find_index = -1;

  if(value != value)
    return result;

  result.input = value;
  result.size = size;

  if(size == 1)
  {
    result.indexes[0] = 0;
    result.indexes[1] = 0;
    result.values[0] = table[0];
    result.values[1] = table[0];
  }
  else if(value <= table[1])
  {
    result.indexes[0] = 0;
    result.indexes[1] = 1;
    result.values[0] = table[result.indexes[0]];
    result.values[1] = table[result.indexes[1]];
  }
  else if(value >= table[size - 2])
  {
    result.indexes[0] = size - 2;
    result.indexes[1] = size - 1;
    result.values[0] = table[result.indexes[0]];
    result.values[1] = table[result.indexes[1]];
  }
  else
  {
    find_index = math_binary_search(table, 0, size - 1, value);
    if(find_index >= 0) {
      result.indexes[0] = find_index;
      result.indexes[1] = find_index + 1;
      result.values[0] = table[find_index];
      result.values[1] = table[find_index+1];
    }
  }

  if(result.values[1] != result.values[0])
    result.mult = (value - result.values[0]) / (result.values[1] - result.values[0]);
  else result.mult = 1.0f;

  return result;
}

INLINE float math_interpolate_1d(sMathInterpolateInput input, const float *table)
{
  float result;
  float output[2];

  output[0] = table[input.indexes[0]];
  output[1] = table[input.indexes[1]];

  result = (output[1] - output[0]) * input.mult + output[0];

  return result;
}

INLINE float math_interpolate_2d(sMathInterpolateInput input_x, sMathInterpolateInput input_y,
    uint32_t y_size, const float (*table)[y_size])
{
  float result = 0.0f;
  float output_1d[2];
  float input_2d[2][2];

  input_2d[0][0] = table[input_y.indexes[0]][input_x.indexes[0]];
  input_2d[0][1] = table[input_y.indexes[0]][input_x.indexes[1]];
  input_2d[1][0] = table[input_y.indexes[1]][input_x.indexes[0]];
  input_2d[1][1] = table[input_y.indexes[1]][input_x.indexes[1]];

  output_1d[0] = (input_2d[0][1] - input_2d[0][0]) * input_x.mult + input_2d[0][0];
  output_1d[1] = (input_2d[1][1] - input_2d[1][0]) * input_x.mult + input_2d[1][0];
  result = (output_1d[1] - output_1d[0]) * input_y.mult + output_1d[0];

  return result;
}

INLINE float math_interpolate_1d_int16(sMathInterpolateInput input, const int16_t *table)
{
  float result;
  float output[2];

  output[0] = table[input.indexes[0]];
  output[1] = table[input.indexes[1]];

  result = (output[1] - output[0]) * input.mult + output[0];

  return result;
}

INLINE float math_interpolate_2d_int16(sMathInterpolateInput input_x, sMathInterpolateInput input_y,
    uint32_t y_size, const int16_t (*table)[y_size])
{
  float result = 0.0f;
  float output_1d[2];
  float input_2d[2][2];

  input_2d[0][0] = table[input_y.indexes[0]][input_x.indexes[0]];
  input_2d[0][1] = table[input_y.indexes[0]][input_x.indexes[1]];
  input_2d[1][0] = table[input_y.indexes[1]][input_x.indexes[0]];
  input_2d[1][1] = table[input_y.indexes[1]][input_x.indexes[1]];

  output_1d[0] = (input_2d[0][1] - input_2d[0][0]) * input_x.mult + input_2d[0][0];
  output_1d[1] = (input_2d[1][1] - input_2d[1][0]) * input_x.mult + input_2d[1][0];
  result = (output_1d[1] - output_1d[0]) * input_y.mult + output_1d[0];

  return result;
}

INLINE float math_interpolate_1d_set_int16(sMathInterpolateInput input, int16_t *table, float new_value)
{
  float previous;
  float diff;
  float output[2];

  output[0] = table[input.indexes[0]];
  output[1] = table[input.indexes[1]];

  previous = (output[1] - output[0]) * input.mult + output[0];

  diff = new_value - previous;

  table[input.indexes[0]] = output[0] + diff * (1.0f - input.mult);
  table[input.indexes[1]] = output[1] + diff * input.mult;

  return diff;
}

INLINE float math_interpolate_2d_set_int16(sMathInterpolateInput input_x, sMathInterpolateInput input_y,
    uint32_t y_size, int16_t (*table)[y_size], float new_value)
{
  float previous;
  float diff;
  float output_1d[2];
  float input_2d[2][2];

  input_2d[0][0] = table[input_y.indexes[0]][input_x.indexes[0]];
  input_2d[0][1] = table[input_y.indexes[0]][input_x.indexes[1]];
  input_2d[1][0] = table[input_y.indexes[1]][input_x.indexes[0]];
  input_2d[1][1] = table[input_y.indexes[1]][input_x.indexes[1]];

  output_1d[0] = (input_2d[0][1] - input_2d[0][0]) * input_x.mult + input_2d[0][0];
  output_1d[1] = (input_2d[1][1] - input_2d[1][0]) * input_x.mult + input_2d[1][0];
  previous = (output_1d[1] - output_1d[0]) * input_y.mult + output_1d[0];

  diff = new_value - previous;

  table[input_y.indexes[0]][input_x.indexes[0]] = input_2d[0][0] + diff * (1.0f - input_x.mult) * (1.0f - input_y.mult);
  table[input_y.indexes[0]][input_x.indexes[1]] = input_2d[0][1] + diff * input_x.mult * (1.0f - input_y.mult);
  table[input_y.indexes[1]][input_x.indexes[0]] = input_2d[1][0] + diff * (1.0f - input_x.mult) * input_y.mult;
  table[input_y.indexes[1]][input_x.indexes[1]] = input_2d[1][1] + diff * input_x.mult * input_y.mult;

  return diff;
}
