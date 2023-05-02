/*
 * defines.h
 *
 *  Created on: 3 мар. 2022 г.
 *      Author: VHEMaster
 */

#ifndef DEFINES_H_
#define DEFINES_H_

#include "main.h"

#define ECU_CYLINDERS_COUNT 4
#define ECU_CYLINDERS_COUNT_HALF (ECU_CYLINDERS_COUNT >> 1) //Division by 2

#define TABLE_SETUPS_MAX 2
#define TABLE_PRESSURES_MAX 16
#define TABLE_THROTTLES_MAX 16
#define TABLE_FILLING_MAX 16
#define TABLE_ROTATES_MAX 16
#define TABLE_SPEEDS_MAX 16
#define TABLE_TEMPERATURES_MAX 16
#define TABLE_VOLTAGES_MAX 16
#define TABLE_STRING_MAX 16
#define TABLE_ENRICHMENT_PERCENTS_MAX 8

#define CHECK_ITEMS_MAX 128
#define CHECK_BITMAP_SIZE (CHECK_ITEMS_MAX >> 3) //Division by 8

#define IDLE_VALVE_POS_MAX (160)

#define PACKET_TABLE_MAX_SIZE 512
#define PACKET_CONFIG_MAX_SIZE PACKET_TABLE_MAX_SIZE
#define PACKET_CRITICAL_MAX_SIZE PACKET_TABLE_MAX_SIZE
#define PACKET_CORRECTION_MAX_SIZE PACKET_TABLE_MAX_SIZE

#define DRAG_MAX_POINTS 3072
#define DRAG_POINTS_DISTANCE 20000

#define OFFSETOF(TYPE, ELEMENT) ((size_t)&(((TYPE *)0)->ELEMENT))
#define ITEMSOF(ARRAY) (sizeof((ARRAY)) / sizeof((ARRAY)[0]))
#define CLAMP(val,min,max) ((val) < (min) ? (min) : (val) > (max) ? (max) : (val))

#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

#define STATIC_INLINE __attribute__((always_inline)) static inline
#define INLINE __attribute__((always_inline)) inline
#define ALIGNED(x) __attribute__((aligned(x)))
#define BUFFER_DMA __attribute__((section(".dma_bss")))
#define ITCM_FUNC __attribute__((section(".itcm_func")))
#define IS_DEBUGGER_ATTACHED() ((DBGMCU->CR & 0x07) > 0)
#define BREAKPOINT(x) __BKPT((x))

#if __CORTEX_M == (7)
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
#endif

#endif /* DEFINES_H_ */
