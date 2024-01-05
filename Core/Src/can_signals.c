/*
 * CAN_SIGNALs.c
 *
 *  Created on: 6 нояб. 2023 г.
 *      Author: VHEMaster
 */

#include <string.h>
#include <math.h>
#include "main.h"
#include "defines.h"
#include "structs.h"
#include "can_signals.h"
#include "can.h"
#include "delay.h"

static sCanMessage * gInitialMessage = NULL;

HAL_StatusTypeDef can_signal_message_clear(sCanMessage *message)
{
  HAL_StatusTypeDef ret = HAL_ERROR;

  if(message != NULL) {
    memset(message->MessageBuffer, 0, sizeof(message->MessageBuffer));
  }

  return ret;
}

HAL_StatusTypeDef can_signal_append_raw(sCanMessage *message, const sCanSignal *signal, uint32_t raw_value)
{
  HAL_StatusTypeDef ret = HAL_ERROR;
  uint64_t frame;
  uint32_t mask;
  uint32_t max = (1 << signal->LengthBit) - 1;

  raw_value = MIN(raw_value, max);

  memcpy(&frame, message->MessageBuffer, sizeof(uint64_t));

  mask = (1 << signal->LengthBit) - 1;
  raw_value &= mask;

  //TODO: optimize it later to avoid U64 type usage
  frame &= ~((uint64_t)mask << signal->StartBit);
  frame |= (uint64_t)raw_value << signal->StartBit;

  memcpy(message->MessageBuffer, &frame, sizeof(uint64_t));

  ret = HAL_OK;

  return ret;
}

HAL_StatusTypeDef can_signal_append_float(sCanMessage *message, sCanSignal *signal, float value)
{
  HAL_StatusTypeDef ret = HAL_ERROR;
  uint32_t raw_value;

  if(!signal->MinMaxDefined) {
    signal->MinValue = signal->Offset;
    signal->MaxValue = (powf(2, signal->LengthBit) - 1.0f) * signal->Gain + signal->Offset;
    signal->MinMaxDefined = 1;
  }

  value = CLAMP(value, signal->MinValue, signal->MaxValue);

  value -= signal->Offset;
  if (signal->Gain != 1.0f) {
    value /= signal->Gain;
  }

  raw_value = (uint32_t)value;

  ret = can_signal_append_raw(message, signal, raw_value);

  return ret;
}

INLINE HAL_StatusTypeDef can_signal_append_uint(sCanMessage *message, sCanSignal *signal, uint32_t value)
{
  HAL_StatusTypeDef ret = HAL_ERROR;

  ret = can_signal_append_raw(message, signal, value);

  return ret;
}

HAL_StatusTypeDef can_signal_get_raw(const sCanMessage *message, const sCanSignal *signal, uint32_t *p_raw_value)
{
  HAL_StatusTypeDef ret = HAL_ERROR;
  uint32_t value = 0;
  uint64_t frame;
  uint32_t mask;

  memcpy(&frame, message->MessageBuffer, sizeof(uint64_t));

  mask = (1 << signal->LengthBit) - 1;

  //TODO: optimize it later to avoid U64 type usage
  value = frame >> signal->StartBit;
  value &= mask;

  *p_raw_value = value;

  ret = HAL_OK;

  return ret;
}

HAL_StatusTypeDef can_signal_get_float(const sCanMessage *message, const sCanSignal *signal, float *p_value)
{
  HAL_StatusTypeDef ret = HAL_ERROR;
  uint32_t raw_value;
  float value;

  ret = can_signal_get_raw(message, signal, &raw_value);

  value = raw_value;
  if (signal->Gain != 1.0f) {
    value *= signal->Gain;
  }
  value += signal->Offset;

  if (p_value) {
    *p_value = value;
  }

  return ret;
}

HAL_StatusTypeDef can_message_send(const sCanMessage *message)
{
  HAL_StatusTypeDef ret = HAL_ERROR;
  int8_t status = 0;
  sCanRawMessage raw_msg;

  raw_msg.id = message->Id;
  raw_msg.length = message->Length;
  raw_msg.rtr = CAN_RTR_DATA;
  memcpy(raw_msg.data.bytes, message->MessageBuffer, raw_msg.length);

  status = can_send(&raw_msg);
  if(status == 1) {
    ret = HAL_OK;
  }

  return ret;
}

HAL_StatusTypeDef can_message_register_msg(sCanMessage *message)
{
  HAL_StatusTypeDef ret = HAL_OK;
  sCanMessage * msg = gInitialMessage;

  do {
    if(message == NULL) {
      ret = HAL_ERROR;
      break;
    }

    if(msg == NULL) {
      gInitialMessage = message;
      message->Next = NULL;
      break;
    } else {
      while (msg) {
        if(msg == message || msg->Id == message->Id) {
          ret = HAL_ERROR;
          break;
        }
        if(msg->Next) {
          msg = msg->Next;
        } else {
          msg->Next = message;
          message->Next = NULL;
          ret = HAL_OK;
          break;
        }
      }
    }
  } while(0);

  return ret;
}

sCanMessage * can_message_get_msg(const sCanRawMessage *message)
{
  sCanMessage * retmsg = gInitialMessage;

  while(retmsg) {
    if(retmsg->Id == message->id) {
      if(message->length >= retmsg->Length) {
        memcpy(retmsg->MessageBuffer, message->data.bytes, message->length);
      } else {
        retmsg = NULL;
      }
      break;
    }
    retmsg = retmsg->Next;
  }

  return retmsg;
}
