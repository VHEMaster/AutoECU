/*
 * can.h
 *
 *  Created on: May 8, 2022
 *      Author: VHEMaster
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "main.h"

typedef struct {
    uint32_t rtr;
    uint16_t id;
    uint8_t length;
    uint8_t data[8];
}sCanMessage;

HAL_StatusTypeDef can_init(CAN_HandleTypeDef *_hcan);
HAL_StatusTypeDef can_start(uint16_t filter_id, uint16_t filter_mask);
void can_loop(void);
int8_t can_test(void);

int8_t can_send(const sCanMessage *message);
int8_t can_transmit(uint32_t id, uint32_t rtr, uint32_t length, const uint8_t *data, uint32_t *p_tx_mailbox);
int8_t can_receive(sCanMessage *message);

void can_rxfifo0pendingcallback(CAN_HandleTypeDef *_hcan);


#endif /* INC_CAN_H_ */
