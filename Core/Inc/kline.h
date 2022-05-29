/*
 * kline.h
 *
 *  Created on: 29 мая 2022 г.
 *      Author: VHEMaster
 */

#ifndef INC_KLINE_H_
#define INC_KLINE_H_

#include "main.h"

#define KLINE_MSG_LEN_MAX 260

typedef struct {
    uint8_t error_loopback;
    uint8_t error_protocol;
    uint32_t last_loopback;
    uint32_t last_protocol;
} sKlineStatus;

typedef struct {
    uint32_t length;
    uint8_t data[KLINE_MSG_LEN_MAX];
    uint8_t src;
    uint8_t dst;
} sKlineMessage;

HAL_StatusTypeDef kline_init(UART_HandleTypeDef *_huart);
HAL_StatusTypeDef kline_start(void);
int8_t kline_send(sKlineMessage *message);
int8_t kline_receive(sKlineMessage *message);
HAL_StatusTypeDef kline_setbaud(uint32_t baudrate);
void kline_loop(void);
sKlineStatus kline_getstatus(void);

void kline_uart_tx_callback(UART_HandleTypeDef *_huart);
void kline_uart_error_callback(UART_HandleTypeDef *_huart);

#endif /* INC_KLINE_H_ */
