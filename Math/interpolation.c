/*
 * interpolation.c
 *
 *  Created on: 10 мар. 2022 г.
 *      Author: VHEMaster
 */

#include "arm_math.h"
#include "interpolation.h"

CMSIS_INLINE __STATIC_INLINE int math_binary_search(float array[], int start_index, int end_index, float element){
   while (start_index <= end_index){
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

CMSIS_INLINE __INLINE sMathInterpolateInput math_interpolate_input(float value, float *table, uint32_t size)
{
  sMathInterpolateInput result = {0};
  int find_index = -1;

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

CMSIS_INLINE __INLINE float math_interpolate_1d(sMathInterpolateInput input, float *table)
{
  float result;
  float output[2];

  output[0] = table[input.indexes[0]];
  output[1] = table[input.indexes[1]];

  result = (output[1] - output[0]) * input.mult + output[0];

  return result;
}

#define TYPE_PTR_TO_2D(pointer, x, y, y_size, type) ((type *)(pointer))[(y)*(y_size)+(x)]

CMSIS_INLINE __INLINE float math_interpolate_2d(sMathInterpolateInput input_x, sMathInterpolateInput input_y,
    uint32_t y_size, float (*table)[y_size])
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
