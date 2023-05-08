/*
 * math_misc.c
 *
 *  Created on: 12 мар. 2022 г.
 *      Author: VHEMaster
 */

#include "defines.h"
#include "math_misc.h"
#include <float.h>

INLINE void math_minmax(const float *array, unsigned int size, float *pmin, float *pmax)
{
  float min = FLT_MAX;
  float max = FLT_MIN;
  float value;

  while(size--) {
    value = *array++;
    if(value < min)
      min = value;
    if(value > max)
      max = value;
  }

  if(pmax)
    *pmax = max;
  if(pmin)
    *pmin = min;

}

INLINE float math_median(const float *array, unsigned int size)
{
  float value = 0;
  float sorted[size];
  unsigned char changed = 0;

  for(int i = 0; i < size; i++) {
    sorted[i] = array[i];
  }

  for (int i = 0; i < size - 1; i++) {
    changed = 0;
    for (int j = 0; j < size - i - 1; j++) {
      if (sorted[j] > sorted[j + 1]) {
        value = sorted[j];
        sorted[j] = sorted[j + 1];
        sorted[j + 1] = value;
        changed = 1;
      }
    }
    if(!changed)
      break;
  }

  value = sorted[size >> 1];

  return value;
}
