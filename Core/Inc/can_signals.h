/*
 * can_signals.h
 *
 *  Created on: 7 нояб. 2023 г.
 *      Author: VHEMaster
 */

#ifndef INC_CAN_SIGNALS_H_
#define INC_CAN_SIGNALS_H_

#include "structs.h"

#define CAN_SIGNAL_BYTES_MAX  (8U)
#define CAN_BITS_IN_BYTE      (8U)
#define CAN_SIGNAL_BITS_MAX   (CAN_SIGNAL_BYTES_MAX * CAN_BITS_IN_BYTE)

typedef enum {
  CAN_SIGNAL_TYPE_UNSIGNED,
  CAN_SIGNAL_TYPE_FLOAT,
}eCanSignalType;

typedef struct {
    eCanSignalType SignalType;
    float Gain;
    float Offset;
    uint8_t StartBit;
    uint8_t LengthBit;
    uint8_t MinMaxDefined;
    float MinValue;
    float MaxValue;
}sCanSignal;

typedef struct {
    uint32_t Id;
    uint8_t Length;
    uint8_t MessageBuffer[CAN_SIGNAL_BYTES_MAX];
}sCanMessage;

HAL_StatusTypeDef can_signal_append_uint(sCanMessage *message, sCanSignal *signal, uint32_t value);
HAL_StatusTypeDef can_signal_append_float(sCanMessage *message, sCanSignal *signal, float value);
HAL_StatusTypeDef can_signal_append_raw(sCanMessage *message, const sCanSignal *signal, uint32_t raw_value);
HAL_StatusTypeDef can_signal_message_clear(sCanMessage *message);
HAL_StatusTypeDef can_message_send(const sCanMessage *message);

#endif /* INC_CAN_SIGNALS_H_ */
