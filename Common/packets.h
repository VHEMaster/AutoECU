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
#define PACKET_HEADER uint8_t PacketID; uint8_t PacketLength; uint8_t Destination; uint8_t Dummy
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
  uint8_t valvenum;
  uint8_t check;
  char tablename[TABLE_STRING_MAX];
  float RealRPM;
  float RPM;
  float Pressure;
  float Load;
  float IgnitionAngle;
  float IgnitionTime;
  float Voltage;
  float Temperature;
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
  uint16_t crc;
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
  uint16_t crc;
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

extern int16_t PK_Copy(void * dest, void * source);

#undef PACKET_C
#undef PACKET_HEADER
#undef PACKET_DEFINE

void PK_SenderInit(void);
void PK_SenderLoop(void);
void PK_SendCommand(eTransChannels xDest, uint8_t *buffer, uint32_t size);

#endif /* PACKETS_H_ */