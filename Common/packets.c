#include <string.h>
#include "packets.h"
#include "xProFIFO.h"
#include "xCommand.h"

#define PACKET_C(x) x##_t x __attribute__((aligned(32))) = {x##ID,sizeof(x##_t)}

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
PACKET_C(PK_ParametersRequest);
PACKET_C(PK_ParametersResponse);
PACKET_C(PK_ForceParametersData);
PACKET_C(PK_ForceParametersDataAcknowledge);
PACKET_C(PK_StatusRequest);
PACKET_C(PK_StatusResponse);
PACKET_C(PK_ResetStatusRequest);
PACKET_C(PK_ResetStatusResponse);
PACKET_C(PK_IgnitionInjectionTestRequest);
PACKET_C(PK_IgnitionInjectionTestResponse);
PACKET_C(PK_SpecificParameterRequest);
PACKET_C(PK_SpecificParameterResponse);

#define SENDERS_MAX_COUNT 4

static uint32_t txDestsCount = 0;
static eTransChannels txDests[SENDERS_MAX_COUNT] = {0};

uint32_t buffSendingQueueSize[SENDERS_MAX_COUNT] = {0};
uint32_t buffSendingBufferSize[SENDERS_MAX_COUNT] = {0};

static uint8_t *buffSendingQueue[SENDERS_MAX_COUNT] = {NULL};
static uint8_t *buffSendingBuffer[SENDERS_MAX_COUNT] = {NULL};

static sProFIFO fifoSendingQueue[SENDERS_MAX_COUNT] = {{NULL}};

static uint8_t sender_sending[SENDERS_MAX_COUNT] = {0};
static uint16_t sender_size[SENDERS_MAX_COUNT] = {0};

void PK_Sender_RegisterDestination(eTransChannels xDest,
    uint8_t *xQueueBuffer, uint32_t xQueueBufferSize,
    uint8_t *xSendingBuffer, uint32_t xSendingBufferSize)
{
  if(txDestsCount < SENDERS_MAX_COUNT) {
    txDests[txDestsCount] = xDest;
    buffSendingQueue[txDestsCount] = xQueueBuffer;
    buffSendingQueueSize[txDestsCount] = xQueueBufferSize;
    buffSendingBuffer[txDestsCount] = xSendingBuffer;
    buffSendingBufferSize[txDestsCount] = xSendingBufferSize;

    protInit(&fifoSendingQueue[txDestsCount], buffSendingQueue[txDestsCount], 1, buffSendingQueueSize[txDestsCount]);

    sender_sending[txDestsCount] = 0;

    txDestsCount++;
  }
}

void PK_SendCommand(eTransChannels xDest, void *buffer, uint32_t size)
{
  sProFIFO *fifo = NULL;
  if(size > 0) {
    for(int i = 0; i < SENDERS_MAX_COUNT; i++) {
      if(txDests[i]) {
        if(txDests[i] == xDest) {
          fifo = &fifoSendingQueue[i];
          if(protGetAvail(fifo) > size) {
        	  protPushSequence(fifo, (uint8_t *)buffer, size);
          }
          break;
        }
      } else {
        break;
      }
    }
  }
}

int16_t PK_Copy(void * dest, void * source)
{
  if(!dest || !source) return -1;
  uint8_t * dest_data = (uint8_t*)dest;
  uint8_t * source_data = (uint8_t*)source;
  int16_t length = dest_data[2] | (dest_data[3] << 8);
  if(memcmp(dest_data, source_data, 4) != 0 || length == 0 || length > MAX_PACK_LEN) return -2;
  memcpy(dest, source, length);
  return length;
}

void PK_SenderInit(void)
{

}

void PK_SenderLoop(void)
{
  uint8_t byte;
  uint8_t * pnt;
  int8_t status;

  for(int i = 0; i < SENDERS_MAX_COUNT; i++)
  {
    if(txDests[i]) {
      do {
        if(!sender_sending[i]) {
          if(protGetSize(&fifoSendingQueue[i]) > 4) {
            protLook(&fifoSendingQueue[i],2,&byte);
            sender_size[i] = byte;
            protLook(&fifoSendingQueue[i],3,&byte);
            sender_size[i] |= byte << 8;

            if(protGetSize(&fifoSendingQueue[i]) >= sender_size[i]) {
              pnt = buffSendingBuffer[i];
              for(int j = 0; j < sender_size[i]; j++)
                protPull(&fifoSendingQueue[i], pnt++);
              sender_sending[i] = 1;
              continue;
            }
          }
        } else {
          status = xSender(txDests[i], buffSendingBuffer[i], sender_size[i]);
          if(status != 0) {
            sender_sending[i] = 0;
            continue;
          }
        }
      } while(0);
    } else {
      break;
    }
  }
}
