/*
 * packets.h
 *
 *  Created on: 31 янв. 2021 г.
 *      Author: VHEMaster
 */

#ifndef PACKETS_H_
#define PACKETS_H_

#include "main.h"
#include "structs.h"
#include "defines.h"
#include "xCommand.h"

#define PACKET_C(x) extern x##_t x;
#define PACKET_HEADER uint16_t PacketID; uint16_t PacketLength;
#define PACKET_DEFINE(x,n) x##_t; static const uint8_t x##ID = n; PACKET_C(x)

typedef struct
{
  PACKET_HEADER;
}sDummyPacketStruct;

typedef struct
{
  PACKET_HEADER;
  uint32_t RandomPing;

}PACKET_DEFINE(PK_Ping, 1);

typedef struct
{
  PACKET_HEADER;
  uint32_t RandomPong;

}PACKET_DEFINE(PK_Pong, 2);

typedef struct
{
  PACKET_HEADER;

}PACKET_DEFINE(PK_GeneralStatusRequest, 3);

typedef struct
{
  PACKET_HEADER;
  uint8_t tablenum;
  uint8_t check;
  char tablename[TABLE_STRING_MAX];
  float RPM;
  float Pressure;
  float Voltage;
  float EngineTemp;
  float FuelUsage;
}PACKET_DEFINE(PK_GeneralStatusResponse, 4);

typedef struct
{
  PACKET_HEADER;
  uint32_t table;
  uint32_t tablesize;
  uint32_t offset;
  uint32_t size;
}PACKET_DEFINE(PK_TableMemoryRequest, 5);

typedef struct
{
  PACKET_HEADER;
  uint32_t ErrorCode;
  uint32_t table;
  uint32_t tablesize;
  uint32_t offset;
  uint32_t size;
  uint8_t data[PACKET_TABLE_MAX_SIZE];
}PACKET_DEFINE(PK_TableMemoryData, 6);

typedef struct
{
  PACKET_HEADER;
  uint32_t ErrorCode;
  uint32_t table;
  uint32_t tablesize;
  uint32_t offset;
  uint32_t size;
}PACKET_DEFINE(PK_TableMemoryAcknowledge, 7);

typedef struct
{
  PACKET_HEADER;
  uint32_t configsize;
  uint32_t offset;
  uint32_t size;
}PACKET_DEFINE(PK_ConfigMemoryRequest, 8);

typedef struct
{
  PACKET_HEADER;
  uint32_t ErrorCode;
  uint32_t configsize;
  uint32_t offset;
  uint32_t size;
  uint8_t data[PACKET_CONFIG_MAX_SIZE];
}PACKET_DEFINE(PK_ConfigMemoryData, 9);

typedef struct
{
  PACKET_HEADER;
  uint32_t ErrorCode;
  uint32_t configsize;
  uint32_t offset;
  uint32_t size;
}PACKET_DEFINE(PK_ConfigMemoryAcknowledge, 10);

typedef struct
{
  PACKET_HEADER;
}PACKET_DEFINE(PK_SaveConfig, 11);

typedef struct
{
  PACKET_HEADER;
}PACKET_DEFINE(PK_RestoreConfig, 12);

typedef struct
{
  PACKET_HEADER;
  uint32_t ErrorCode;
}PACKET_DEFINE(PK_SaveConfigAcknowledge, 13);

typedef struct
{
  PACKET_HEADER;
  uint32_t ErrorCode;
}PACKET_DEFINE(PK_RestoreConfigAcknowledge, 14);

typedef struct
{
  PACKET_HEADER;
  float FromSpeed;
  float ToSpeed;
}PACKET_DEFINE(PK_DragStart, 15);

typedef struct
{
  PACKET_HEADER;
}PACKET_DEFINE(PK_DragUpdateRequest, 16);

typedef struct
{
  PACKET_HEADER;
  uint8_t ErrorCode;
  float FromSpeed;
  float ToSpeed;
  sDragPoint Data;
  uint32_t TotalPoints;
  uint8_t Started;
  uint8_t Completed;
}PACKET_DEFINE(PK_DragUpdateResponse, 17);

typedef struct
{
  PACKET_HEADER;
  float FromSpeed;
  float ToSpeed;
}PACKET_DEFINE(PK_DragStop, 18);

typedef struct
{
  PACKET_HEADER;
  float FromSpeed;
  float ToSpeed;
  uint32_t PointNumber;
}PACKET_DEFINE(PK_DragPointRequest, 19);

typedef struct
{
  PACKET_HEADER;
  uint8_t ErrorCode;
  float FromSpeed;
  float ToSpeed;
  uint32_t PointNumber;
  sDragPoint Point;
}PACKET_DEFINE(PK_DragPointResponse, 20);

typedef struct
{
  PACKET_HEADER;
  uint8_t ErrorCode;
  float FromSpeed;
  float ToSpeed;
}PACKET_DEFINE(PK_DragStartAcknowledge, 21);

typedef struct
{
  PACKET_HEADER;
  uint8_t ErrorCode;
  float FromSpeed;
  float ToSpeed;
}PACKET_DEFINE(PK_DragStopAcknowledge, 22);

typedef struct
{
  PACKET_HEADER;
}PACKET_DEFINE(PK_PcConnected, 23);

typedef struct
{
  PACKET_HEADER;
  uint8_t FuelSwitchPos;

}PACKET_DEFINE(PK_FuelSwitch, 24);

typedef struct
{
  PACKET_HEADER;
  uint32_t configsize;
  uint32_t offset;
  uint32_t size;
}PACKET_DEFINE(PK_CorrectionsMemoryRequest, 25);

typedef struct
{
  PACKET_HEADER;
  uint32_t ErrorCode;
  uint32_t configsize;
  uint32_t offset;
  uint32_t size;
  uint8_t data[PACKET_CORRECTION_MAX_SIZE];
}PACKET_DEFINE(PK_CorrectionsMemoryData, 26);

typedef struct
{
  PACKET_HEADER;
  uint32_t ErrorCode;
  uint32_t configsize;
  uint32_t offset;
  uint32_t size;
}PACKET_DEFINE(PK_CorrectionsMemoryAcknowledge, 27);

typedef struct
{
  PACKET_HEADER;
  uint32_t configsize;
  uint32_t offset;
  uint32_t size;
}PACKET_DEFINE(PK_CriticalMemoryRequest, 28);

typedef struct
{
  PACKET_HEADER;
  uint32_t ErrorCode;
  uint32_t configsize;
  uint32_t offset;
  uint32_t size;
  uint8_t data[PACKET_CRITICAL_MAX_SIZE];
}PACKET_DEFINE(PK_CriticalMemoryData, 29);

typedef struct
{
  PACKET_HEADER;
  uint32_t ErrorCode;
  uint32_t configsize;
  uint32_t offset;
  uint32_t size;
}PACKET_DEFINE(PK_CriticalMemoryAcknowledge, 30);

typedef struct
{
  PACKET_HEADER;
}PACKET_DEFINE(PK_ParametersRequest, 31);

typedef struct
{
  PACKET_HEADER;
  sParameters Parameters;
}PACKET_DEFINE(PK_ParametersResponse, 32);

typedef struct
{
  PACKET_HEADER;
  sForceParameters Parameters;
}PACKET_DEFINE(PK_ForceParametersData, 33);

typedef struct
{
  PACKET_HEADER;
  uint32_t ErrorCode;
}PACKET_DEFINE(PK_ForceParametersDataAcknowledge, 34);

typedef struct
{
  PACKET_HEADER;
}PACKET_DEFINE(PK_StatusRequest, 35);

typedef struct
{
  PACKET_HEADER;
  uint8_t CheckBitmap[CHECK_BITMAP_SIZE];
  uint8_t CheckBitmapRecorded[CHECK_BITMAP_SIZE];
}PACKET_DEFINE(PK_StatusResponse, 36);

typedef struct
{
  PACKET_HEADER;
}PACKET_DEFINE(PK_ResetStatusRequest, 37);

typedef struct
{
  PACKET_HEADER;
  uint32_t ErrorCode;
}PACKET_DEFINE(PK_ResetStatusResponse, 38);

typedef struct
{
  PACKET_HEADER;
  uint8_t IgnitionEnabled;
  uint8_t InjectionEnabled;
  uint16_t Count;
  uint32_t Period;
  uint32_t IgnitionPulse;
  uint32_t InjectionPulse;
}PACKET_DEFINE(PK_IgnitionInjectionTestRequest, 39);

typedef struct
{
  PACKET_HEADER;
  uint32_t ErrorCode;
}PACKET_DEFINE(PK_IgnitionInjectionTestResponse, 40);

typedef struct
{
  PACKET_HEADER;
  uint32_t addr;
}PACKET_DEFINE(PK_SpecificParameterRequest, 41);

typedef struct
{
  PACKET_HEADER;
  uint32_t addr;
  union {
      uint8_t b[4];
      uint32_t u;
      float f;
  } Parameter;
}PACKET_DEFINE(PK_SpecificParameterResponse, 42);



int16_t PK_Copy(void * dest, void * source);

#undef PACKET_C
#undef PACKET_HEADER
#undef PACKET_DEFINE

void PK_SenderInit(void);
void PK_SenderLoop(void);
void PK_SendCommand(eTransChannels xDest, void *buffer, uint32_t size);
void PK_Sender_RegisterDestination(eTransChannels xDest,
    uint8_t *xQueueBuffer, uint32_t xQueueBufferSize,
    uint8_t *xSendingBuffer, uint32_t xSendingBufferSize);

#endif /* PACKETS_H_ */
