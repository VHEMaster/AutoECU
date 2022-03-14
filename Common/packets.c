#include "packets.h"
#include "xProFIFO.h"
#include "xCommand.h"

#define PACKET_C(x) x##_t x __attribute__((aligned(32))) = {x##ID,sizeof(x##_t),0,0}

PACKET_C(PK_Ping);
PACKET_C(PK_Pong);
PACKET_C(PK_GeneralStatusRequest);
PACKET_C(PK_GeneralStatusResponse);
PACKET_C(PK_TableMemoryRequest);
PACKET_C(PK_TableMemoryData);
PACKET_C(PK_TableMemoryAcknowledge);
PACKET_C(PK_ConfigMemoryRequest);
PACKET_C(PK_ConfigMemoryData);
PACKET_C(PK_ConfigMemoryAcknowledge);
PACKET_C(PK_SaveConfig);
PACKET_C(PK_RestoreConfig);
PACKET_C(PK_SaveConfigAcknowledge);
PACKET_C(PK_RestoreConfigAcknowledge);
PACKET_C(PK_DragStart);
PACKET_C(PK_DragUpdateRequest);
PACKET_C(PK_DragUpdateResponse);
PACKET_C(PK_DragStop);
PACKET_C(PK_DragPointRequest);
PACKET_C(PK_DragPointResponse);
PACKET_C(PK_DragStartAcknowledge);
PACKET_C(PK_DragStopAcknowledge);
PACKET_C(PK_PcConnected);
PACKET_C(PK_FuelSwitch);
PACKET_C(PK_CorrectionsMemoryRequest);
PACKET_C(PK_CorrectionsMemoryData);
PACKET_C(PK_CorrectionsMemoryAcknowledge);
PACKET_C(PK_CriticalMemoryRequest);
PACKET_C(PK_CriticalMemoryData);
PACKET_C(PK_CriticalMemoryAcknowledge);


#define SENDING_QUEUE_SIZE (MAX_PACK_LEN*4)
static uint8_t buffSendingQueue[SENDING_QUEUE_SIZE];
static sProFIFO fifoSendingQueue;

#define SENDING_BUFFER_SIZE (MAX_PACK_LEN)
static uint8_t buffSendingBuffer[SENDING_BUFFER_SIZE];


int16_t PK_Copy(void * dest, void * source)
{
  if(dest == 0 || source == 0) return -1;
  uint8_t * dest_data = (uint8_t*)dest;
  uint8_t * source_data = (uint8_t*)source;
  int16_t length = dest_data[1];
  if(dest_data[1] != source_data[1] || dest_data[0] != source_data[0] || length == 0 || length > 384) return -2;
  for(uint8_t i=2;i<length+2;i++)
    *dest_data++ = *source_data++;
  return length;
}

void PK_SenderInit(void)
{
  protInit(&fifoSendingQueue, buffSendingQueue, 1, SENDING_QUEUE_SIZE);
}

void PK_SenderLoop(void)
{
  static uint8_t sending = 0;
  static uint8_t destination = 0;
  static uint8_t dummy = 0;
  static uint8_t size = 0;
  uint8_t * pnt;
  int8_t status;

  do
  {
    if(!sending)
    {
      if(protGetSize(&fifoSendingQueue) > 8) {
        protLook(&fifoSendingQueue,1,&size);
        protLook(&fifoSendingQueue,2,&destination);
        protLook(&fifoSendingQueue,3,&dummy);
        if(destination > etrNone && destination < etrCount && dummy == 0)
        {
          if(protGetSize(&fifoSendingQueue) >= size)
          {
            pnt = buffSendingBuffer;
            for(int i = 0; i < size; i++)
              protPull(&fifoSendingQueue, pnt++);
            if(destination)
              sending = 1;
          }
        } else protClear(&fifoSendingQueue);
      }
    } else {
      status = xSender(destination, buffSendingBuffer, size);
      if(status != 0)
      {
        sending = 0;
        continue;
      }
    }
  } while(0);
}

void PK_SendCommand(eTransChannels xDest, void *buffer, uint32_t size)
{
  ((sDummyPacketStruct *)buffer)->Destination = xDest;
  protPushSequence(&fifoSendingQueue, (uint8_t *)buffer, size);
}
