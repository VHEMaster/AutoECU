/*
 * xCommand.h
 *
 *  Created on: Dec 6, 2020
 *      Author: denys.prokhorov
 */


#ifndef XCOMMAND_H_
#define XCOMMAND_H_

#include "main.h"

#define TASK_SLEEP  { osDelay(1); } // Task must give it's time to another process or just skip some time
#define MAX_PACK_LEN (768)

typedef enum {
    etrNone,
    etrPC,
    etrECU,
    etrIMMO,
    etrCTRL,
    etrBT,
    etrCount
} eTransChannels;

void xFifosInit(void);
void xGetterInit(void);
void xGetterLoop(void);
int8_t xSender(eTransChannels xChaDest, const uint8_t* xMsgPtr, uint32_t xMsgLen);
void xDmaTxIrqHandler(UART_HandleTypeDef *huart);
void xDmaRxIrqHandler(UART_HandleTypeDef *huart);
void xDmaErIrqHandler(UART_HandleTypeDef *huart);

void xSenderRaw(eTransChannels xChaDest, const uint8_t* xMsgPtr, uint32_t xMsgLen);


#endif /* XCOMMAND_H_ */
