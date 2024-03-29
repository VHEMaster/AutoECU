/*
 * can.c
 *
 *  Created on: May 8, 2022
 *      Author: VHEMaster
 */

#include "can.h"
#include "delay.h"
#include "defines.h"
#include "xProFIFO.h"
#include "xCommand.h"
#include <string.h>

static CAN_HandleTypeDef *hcan;
static CAN_FilterTypeDef can_filter;

#define CAN_TX_BUFFER_COUNT (64)
#define CAN_RX_BUFFER_COUNT (64)

static sProFIFO cantxfifo;
static sProFIFO canrxfifo;
static sCanRawMessage cantxbuffer[CAN_TX_BUFFER_COUNT];
static sCanRawMessage canrxbuffer[CAN_RX_BUFFER_COUNT];

#define CAN_LOOPBACK() HAL_GPIO_WritePin(CAN1_LBK_GPIO_Port, CAN1_LBK_Pin, GPIO_PIN_SET)
#define CAN_NORMAL() HAL_GPIO_WritePin(CAN1_LBK_GPIO_Port, CAN1_LBK_Pin, GPIO_PIN_RESET)

void can_rxfifo_pending_callback(CAN_HandleTypeDef *_hcan, uint32_t fifo)
{
  CAN_RxHeaderTypeDef header;
  sCanRawMessage message = {0};
  HAL_StatusTypeDef status;
  if(_hcan == hcan) {
    status = HAL_CAN_GetRxMessage(hcan, fifo, &header, message.data.bytes);
    if(status == HAL_OK) {
      message.id = header.StdId;
      message.length = header.DLC;
      message.rtr = header.RTR;
      protPush(&canrxfifo, &message);
    }

  }
}

void can_txfifo_aborted_callback(CAN_HandleTypeDef *_hcan, uint32_t fifo)
{

}

void can_txfifo_completed_callback(CAN_HandleTypeDef *_hcan, uint32_t fifo)
{

}

HAL_StatusTypeDef can_init(CAN_HandleTypeDef *_hcan)
{
  HAL_StatusTypeDef status = HAL_OK;
  hcan = _hcan;

  CAN_LOOPBACK();
  protInit(&canrxfifo, canrxbuffer, sizeof(canrxbuffer[0]), ITEMSOF(canrxbuffer));
  protInit(&cantxfifo, cantxbuffer, sizeof(cantxbuffer[0]), ITEMSOF(cantxbuffer));

  return status;
}

HAL_StatusTypeDef can_start(const uint16_t *filter_ids, const uint16_t *filter_masks, uint8_t len)
{
  HAL_StatusTypeDef status = HAL_OK;

  for(int i = 0; i < len; i++) {
    can_filter.FilterActivation = CAN_FILTER_ENABLE;
    can_filter.FilterBank = i;
    can_filter.FilterFIFOAssignment = ((i & 1) == 0) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
    can_filter.FilterIdHigh = filter_ids[i] << 5;
    can_filter.FilterIdLow = 0;
    can_filter.FilterMaskIdHigh = filter_masks[i] << 5;
    can_filter.FilterMaskIdLow = 0;
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter.SlaveStartFilterBank = 13;

    status = HAL_CAN_ConfigFilter(hcan, &can_filter);
    if(status != HAL_OK)
      return status;
  }

  status = HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY);
  if(status != HAL_OK)
    return status;

  status = HAL_CAN_ActivateNotification(hcan, CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR);
  if(status != HAL_OK)
    return status;

  status = HAL_CAN_Start(hcan);
  if(status != HAL_OK)
    return status;

  CAN_NORMAL();

  return status;
}

int8_t can_test(void)
{
  static uint32_t last = 0;
  static uint8_t txdata[8];
  static uint32_t txlength;
  static uint32_t txid;
  static uint32_t txrtr;
  sCanRawMessage message;
  HAL_StatusTypeDef hal_status;
  int8_t status;
  uint32_t now = Delay_Tick;

  hal_status = HAL_CAN_Stop(hcan);
  if(hal_status != HAL_OK)
    return -10;
  hcan->Init.Mode = CAN_MODE_LOOPBACK;
  hal_status = HAL_CAN_Init(hcan);
  if(hal_status != HAL_OK)
    return -11;
  CAN_LOOPBACK();
  hal_status = HAL_CAN_Start(hcan);
  if(hal_status != HAL_OK)
    return -12;


  for(int i = 0; i < 10; i++) {
    txid = can_filter.FilterIdHigh >> 5;
    txrtr = CAN_RTR_DATA;
    txlength = 4;
    for(int j = 0; j < txlength; j++)
      txdata[j] = i * 10 + j;

    last = now;

    while((status = can_transmit(txid, txrtr, txlength, txdata, NULL)) == 0) {
      now = Delay_Tick;
      if(DelayDiff(now, last) > 100000) {
        status = -20;
        break;
      }
    }

    if(status <= 0)
      break;

    if(status > 0) {
      last = now;

      while((status = can_receive(&message)) == 0) {
        now = Delay_Tick;
        if(DelayDiff(now, last) > 50000) {
          status = -21;
          break;
        }
      }

      if(status <= 0)
        break;

      if(status > 0) {
        if(txid != message.id || txrtr != message.rtr || txlength != message.length || memcmp(txdata, message.data.bytes, txlength) != 0)
          status = -100;
      }
    }
  }


  hal_status = HAL_CAN_Stop(hcan);
  if(hal_status != HAL_OK)
    return -13;
  hcan->Init.Mode = CAN_MODE_NORMAL;
  hal_status = HAL_CAN_Init(hcan);
  if(hal_status != HAL_OK)
    return -14;
  CAN_NORMAL();
  hal_status = HAL_CAN_Start(hcan);
  if(hal_status != HAL_OK)
    return -15;

  return status;
}

int8_t can_send(const sCanRawMessage *message)
{
  int8_t status = 0;
  if(protGetAvail(&cantxfifo)) {
    if(protPush(&cantxfifo, message)) {
      status = 1;
    }
  }

  return status;
}

void can_loop(void)
{
  static sCanRawMessage message = {0};
  static uint8_t state = 0;
  int8_t status;

  while(1) {
    switch(state) {
      case 0:
        if(protPull(&cantxfifo, &message)) {
          state++;
          continue;
        }
        break;
      case 1:
        status = can_transmit(message.id, message.rtr, message.length, message.data.bytes, NULL);
        if(status != 0) {
          state = 0;
          continue;
        }
        break;
      default:
        state = 0;
        continue;
        break;
    }
    break;
  }
}

int8_t can_transmit(uint32_t id, uint32_t rtr, uint32_t length, const uint8_t *data, uint32_t *p_tx_mailbox)
{
  static uint8_t state = 0;
  static uint32_t time_last = 0;
  static uint32_t mailbox;
  static CAN_TxHeaderTypeDef header = {0};
  uint32_t now = Delay_Tick;
  HAL_StatusTypeDef hal_status;
  int8_t status = 0;
  uint32_t level;

  while(1) {
    switch(state) {
      case 0:
        header.IDE = CAN_ID_STD;
        header.StdId = id;
        header.RTR = rtr;
        header.DLC = length;
        time_last = now;
        state++;
        continue;
      case 1:
        level = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
        if(level > 0) {
          time_last = now;
          state++;
          continue;
        }
        else if(DelayDiff(now, time_last) > 50000) {
          state = 0;
          status = -1;
        }
        break;
      case 2:
        hal_status = HAL_CAN_AddTxMessage(hcan, &header, (uint8_t *)data, &mailbox);
        if(hal_status == HAL_OK) {
          state = 0;
          status = 1;
          time_last = now;
          if(p_tx_mailbox)
            *p_tx_mailbox = mailbox;
        }
        else if(DelayDiff(now, time_last) > 50000) {
          state = 0;
          status = -2;
        }
        break;
      default:
        state = 0;
        continue;
    }
    break;
  }

  return status;
}

int8_t can_receive(sCanRawMessage *p_message)
{
  sCanRawMessage message;
  if(protPull(&canrxfifo, &message)) {
    *p_message = message;
    return 1;
  }
  return 0;
}
