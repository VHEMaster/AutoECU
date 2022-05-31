/*
 * kline.c
 *
 *  Created on: 29 мая 2022 г.
 *      Author: VHEMaster
 */

#include "kline.h"
#include "delay.h"
#include "defines.h"
#include "xProFIFO.h"
#include <string.h>

#define KLINE_TX_TIMEOUT 8000
#define KLINE_RX_TIMEOUT 10000

#define KLINE_RX_BUFFER_SIZE 256

#define KLINE_RX_BUFFER_COUNT 16
#define KLINE_TX_BUFFER_COUNT 16

static UART_HandleTypeDef *huart;

static sProFIFO klinetxfifo;
static sProFIFO klinerxfifo;

static sKlineMessage kline_tx_fifo_buffer[KLINE_TX_BUFFER_COUNT];
static sKlineMessage kline_rx_fifo_buffer[KLINE_RX_BUFFER_COUNT];

static uint8_t kline_rx_buffer[KLINE_RX_BUFFER_SIZE] __attribute__((aligned(32)));

static volatile uint8_t kline_tx_busy = 0;
static volatile uint8_t kline_er_flag = 0;
static volatile uint32_t kline_rxpointer = 0xFFFFFFFF;

static sKlineStatus kline_status = {0};

void kline_uart_tx_callback(UART_HandleTypeDef *_huart)
{
  if(_huart == huart) {
    kline_tx_busy = 0;
  }
}

void kline_uart_error_callback(UART_HandleTypeDef *_huart)
{
  if(_huart == huart) {
    kline_er_flag = 1;
  }
}

static int8_t kline_parse(sKlineMessage *message, uint8_t is_lbk, const sKlineMessage *lbk_msg) {
  int8_t status = 0;
  uint8_t fmt;
  uint8_t dst;
  uint8_t src;
  uint8_t crc;
  uint8_t len;
  uint8_t len_expected;
  uint8_t crc_actual;
  uint8_t *data;
  uint8_t addr_mode;

  if(is_lbk) {
    if(message->length == lbk_msg->length) {
      if(memcmp(message->data, lbk_msg->data, message->length) == 0) {
        status = 0;
      } else {
        status = -1;
      }
    } else if(message->length > lbk_msg->length) {
      status = -1;
    }
  } else {
    if(message->length >= 4) {
      data = message->data;
      fmt = *data++;
      dst = *data++;
      src = *data++;
      addr_mode = fmt & 0xC0;
      if((fmt & 0x3F) == 0) {
        len = *data++;
        len_expected = 5 + len;
      } else {
        len = fmt & 0x3F;
        len_expected = 4 + len;
      }
      if(message->length == len_expected) {
        crc = data[len];
        crc_actual = 0;
        for(int i = 0; i < message->length -1; i++) {
          crc_actual += message->data[i];
        }
        if(crc == crc_actual) {
          //TODO: is it really need to check it?
          if(addr_mode == 0x80) {
            message->src = src;
            message->dst = dst;
            for(int i = 0; i < len; i++) {
              message->data[i] = *data++;
            }
            message->length = len;
            status = 1;
          } else {
            status = -3;
          }
        } else {
          status = -2;
        }
      } else if(message->length > len_expected) {
        status = -1;
      }
    }
  }

  if(status != 0) {
    message->length = 0;
  }

  return status;
}

HAL_StatusTypeDef kline_init(UART_HandleTypeDef *_huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  HAL_StatusTypeDef status = HAL_OK;
  huart = _huart;

  if(!huart)
    return HAL_ERROR;

  protInit(&klinetxfifo, kline_tx_fifo_buffer, sizeof(kline_tx_fifo_buffer[0]), ITEMSOF(kline_tx_fifo_buffer));
  protInit(&klinerxfifo, kline_rx_fifo_buffer, sizeof(kline_rx_fifo_buffer[0]), ITEMSOF(kline_rx_fifo_buffer));
  kline_rxpointer = 0xFFFFFFFF;

  // Short K-Line to GND while initialization is ongoing
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  return status;
}

HAL_StatusTypeDef kline_setbaud(uint32_t baudrate)
{
  HAL_UART_Abort(huart);
  huart->Init.BaudRate = baudrate;
  kline_rxpointer = 0xFFFFFFFF;
  return HAL_UART_Init(huart);
}

HAL_StatusTypeDef kline_start(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  HAL_StatusTypeDef status = HAL_OK;

  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  kline_rxpointer = 0xFFFFFFFF;
  kline_er_flag = 0;
  kline_tx_busy = 0;
  status = HAL_UART_Receive_DMA(huart, kline_rx_buffer, sizeof(kline_rx_buffer));

  return status;
}

void kline_loop(void)
{
  static uint32_t rx_last = 0;
  static uint32_t tx_last = 0;
  static sKlineMessage rx_message = {0};
  static sKlineMessage tx_message = {0};
  static uint8_t receiving = 0;
  static uint8_t transmitting = 0;
  uint32_t now = Delay_Tick;
  uint32_t dmacnt;
  uint32_t length;
  uint32_t dmasize;
  int8_t status;
  do
  {

    dmacnt = huart->hdmarx->Instance->NDTR;
    dmasize = huart->RxXferSize;
    if(kline_rxpointer == 0xFFFFFFFF) kline_rxpointer = dmacnt;
    if(dmacnt > kline_rxpointer)
      length = (dmasize-dmacnt)+kline_rxpointer;
    else length = kline_rxpointer-dmacnt;

    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) != GPIO_PIN_SET) {
      rx_last = now;
      receiving = 1;
    }

    if(length + rx_message.length > KLINE_MSG_LEN_MAX) length = KLINE_MSG_LEN_MAX - rx_message.length;
    if(length > 0)
    {
      CacheInvalidate(kline_rx_buffer, sizeof(kline_rx_buffer));
      for(int i=0;i<length;i++)
      {
        rx_message.data[rx_message.length++] = kline_rx_buffer[dmasize-kline_rxpointer];
        if(kline_rxpointer == 1) kline_rxpointer = dmasize;
        else kline_rxpointer--;
      }

      rx_last = now;
      receiving = 1;
    } else {
      if(receiving) {
        if((!transmitting && DelayDiff(now, rx_last) > KLINE_RX_TIMEOUT) ||
            (transmitting && DelayDiff(now, rx_last) > KLINE_TX_TIMEOUT)) {
          status = kline_parse(&rx_message, transmitting, &tx_message);

          if(status) {
            if(status > 0 && rx_message.length > 0) {
              protPush(&klinerxfifo, &rx_message);
            } else {
              if(transmitting) {
                kline_status.error_protocol++;
                kline_status.last_protocol = now;
              } else {
                kline_status.error_loopback++;
                kline_status.last_loopback = now;
              }
            }
          }

          transmitting = 0;
          receiving = 0;
          rx_message.length = 0;
          rx_last = now;
        }
      } else if(transmitting && DelayDiff(now, rx_last) > KLINE_TX_TIMEOUT) {
        transmitting = 0;
        kline_status.error_loopback++;
        kline_status.last_loopback = now;
      }
    }
  } while(length > 0);

  if(!kline_tx_busy) {
    if(!receiving && !transmitting) {
      if(protGetSize(&klinetxfifo) > 0) {
        if(protPull(&klinetxfifo, &tx_message)) {
          kline_tx_busy = 1;
          if(HAL_UART_Transmit_IT(huart, tx_message.data, tx_message.length) == HAL_OK) {
            tx_last = now;
            rx_last = now;
            transmitting = 1;
          } else {
            kline_tx_busy = 0;
          }
        }
      }
    }
  } else if(DelayDiff(now, tx_last) > 1000000) {
    HAL_UART_AbortTransmit(huart);
    kline_tx_busy = 0;
    transmitting = 0;
  }

  if(kline_status.error_loopback && DelayDiff(now, kline_status.last_loopback) > 2000000) {
    kline_status.error_loopback = 0;
    kline_status.last_loopback = 0;
  }

  if(kline_status.error_protocol && DelayDiff(now, kline_status.last_protocol) > 2000000) {
    kline_status.error_protocol = 0;
    kline_status.last_protocol = 0;
  }
}

int8_t kline_send(sKlineMessage *message)
{
  int8_t status = 0;
  uint8_t len = 0;
  uint8_t inc = 3;

  if(!message || !message->length || message->length > 255)
    return -1;

  if(protGetAvail(&klinetxfifo) > 0) {
    if(message->length >= 64)
      inc = 4;

    for(int i = message->length - 1; i >= 0; i--)
      message->data[i + inc] = message->data[i];

    message->data[len++] = 0x80 + (message->length < 64 ? message->length : 0);
    message->data[len++] = message->dst;
    message->data[len++] = message->src;
    if(message->length >= 64)
      message->data[len++] = message->length;
    message->length += len;
    message->data[message->length] = 0;
    for(int i = 0; i < message->length; i++)
      message->data[message->length] += message->data[i];
    message->length++;

    if(protPush(&klinetxfifo, message))
      status = 1;
  }

  return status;
}

int8_t kline_receive(sKlineMessage *message)
{
  int8_t status = 0;

  if(!message)
    return -1;

  if(protGetSize(&klinerxfifo) > 0)
    if(protPull(&klinerxfifo, message))
      status = 1;

  return status;
}

sKlineStatus kline_getstatus(void)
{
  return kline_status;
}

