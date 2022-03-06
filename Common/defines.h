/*
 * defines.h
 *
 *  Created on: 3 мар. 2022 г.
 *      Author: VHEMaster
 */

#ifndef DEFINES_H_
#define DEFINES_H_

#define TABLE_SETUPS_MAX 4
#define TABLE_PRESSURES_MAX 12
#define TABLE_THROTTLES_MAX 12
#define TABLE_FILLING_MAX 12
#define TABLE_ROTATES_MAX 12
#define TABLE_SPEEDS_MAX 12
#define TABLE_TEMPERATURES_MAX 12
#define TABLE_STRING_MAX 16

#define PACKET_TABLE_MAX_SIZE 224
#define PACKET_CONFIG_MAX_SIZE PACKET_TABLE_MAX_SIZE

#define DRAG_MAX_POINTS 3072
#define DRAG_POINTS_DISTANCE 20000

#define OFFSETOF(TYPE, ELEMENT) ((size_t)&(((TYPE *)0)->ELEMENT))
#define ITEMSOF(ARRAY) (sizeof((ARRAY)) / sizeof((ARRAY)[0]))

#endif /* DEFINES_H_ */
