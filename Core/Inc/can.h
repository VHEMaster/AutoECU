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
    union {
        uint8_t bytes[8];
        uint16_t words[4];
        uint32_t dwords[2];
        uint64_t qword;
    } data;
    uint32_t rtr;
    uint16_t id;
    uint8_t length;
    uint8_t pad;
}sCanRawMessage __attribute__((aligned(8)));

HAL_StatusTypeDef can_init(CAN_HandleTypeDef *_hcan);
HAL_StatusTypeDef can_start(uint16_t filter_id, uint16_t filter_mask);
void can_loop(void);
int8_t can_test(void);

int8_t can_send(const sCanRawMessage *message);
int8_t can_transmit(uint32_t id, uint32_t rtr, uint32_t length, const uint8_t *data, uint32_t *p_tx_mailbox);
int8_t can_receive(sCanRawMessage *message);

void can_rxfifo_pending_callback(CAN_HandleTypeDef *_hcan, uint32_t fifo);
void can_txfifo_aborted_callback(CAN_HandleTypeDef *_hcan, uint32_t fifo);
void can_txfifo_completed_callback(CAN_HandleTypeDef *_hcan, uint32_t fifo);


#endif /* INC_CAN_H_ */
