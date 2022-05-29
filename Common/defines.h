/*
 * defines.h
 *
 *  Created on: 3 мар. 2022 г.
 *      Author: VHEMaster
 */

#ifndef DEFINES_H_
#define DEFINES_H_

#include <stdint.h>
#include "stm32f7xx.h"

#define ECU_CYLINDERS_COUNT 4

#define TABLE_SETUPS_MAX 3
#define TABLE_PRESSURES_MAX 16
#define TABLE_THROTTLES_MAX 16
#define TABLE_FILLING_MAX 16
#define TABLE_ROTATES_MAX 16
#define TABLE_SPEEDS_MAX 16
#define TABLE_TEMPERATURES_MAX 16
#define TABLE_VOLTAGES_MAX 16
#define TABLE_STRING_MAX 16

#define CHECK_ITEMS_MAX 128
#define CHECK_BITMAP_SIZE (CHECK_ITEMS_MAX / 8)

#define PACKET_TABLE_MAX_SIZE 512
#define PACKET_CONFIG_MAX_SIZE PACKET_TABLE_MAX_SIZE
#define PACKET_CRITICAL_MAX_SIZE PACKET_TABLE_MAX_SIZE
#define PACKET_CORRECTION_MAX_SIZE PACKET_TABLE_MAX_SIZE

#define DRAG_MAX_POINTS 3072
#define DRAG_POINTS_DISTANCE 20000

#define OFFSETOF(TYPE, ELEMENT) ((size_t)&(((TYPE *)0)->ELEMENT))
#define ITEMSOF(ARRAY) (sizeof((ARRAY)) / sizeof((ARRAY)[0]))

#define STATIC_INLINE __attribute__((always_inline)) static inline
#define INLINE __attribute__((always_inline)) inline

STATIC_INLINE void CacheInvalidate(void * buffer, uint32_t size)
{
  uint32_t aligned = (uint32_t)buffer % 32;
  if(aligned == 0)
    SCB_InvalidateDCache_by_Addr((uint32_t*)buffer, size);
  else SCB_InvalidateDCache_by_Addr((uint32_t*)((uint32_t)buffer - aligned), size + aligned);
}

STATIC_INLINE void CacheClean(void * buffer, uint32_t size)
{
  uint32_t aligned = (uint32_t)buffer % 32;
  if(aligned == 0)
    SCB_CleanDCache_by_Addr((uint32_t*)buffer, size);
  else SCB_CleanDCache_by_Addr((uint32_t*)((uint32_t)buffer - aligned), size + aligned);
}

#endif /* DEFINES_H_ */
