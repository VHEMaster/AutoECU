/*
 * xProFIFO.h
 *
 *  Created on: Dec 2, 2020
 *      Author: denys.prokhorov
 */

#ifndef XPROFIFO_H_
#define XPROFIFO_H_

#include "main.h"

typedef struct {
    uint32_t capacity;
    uint32_t read;
    uint32_t write;
    uint8_t elemsize;
    uint8_t overflow;
} sProInfo;

typedef struct {
	void* buffer;
	sProInfo info;
} sProFIFO;

extern void protInit(sProFIFO* fifo, void* xBuffer, uint8_t xElemSize, int xCapacity);
extern void protClear(sProFIFO* fifo);
extern uint32_t protPush(sProFIFO* fifo, void* xData);
extern uint32_t protPushSequence(sProFIFO* fifo, void* xData, uint32_t xCount);
extern uint32_t protPull(sProFIFO* fifo, void* xDest);
extern void protLook(sProFIFO* fifo, uint32_t xOffset, void* xDest);
extern uint32_t protGetSize(sProFIFO* fifo);
extern uint32_t protGetAvail(sProFIFO* fifo);
extern void protMoveWrite(sProFIFO* fifo, uint32_t amove);
extern void protMoveRead(sProFIFO* fifo, uint32_t amove);
extern uint8_t protIsSome(sProFIFO* fifo);

#endif /* XPROFIFO_H_ */
