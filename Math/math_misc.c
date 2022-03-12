/*
 * math_misc.c
 *
 *  Created on: 12 мар. 2022 г.
 *      Author: VHEMaster
 */

#include "math_misc.h"
#include <float.h>

void math_minmax(const float *array, unsigned int size, float *pmin, float *pmax)
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
