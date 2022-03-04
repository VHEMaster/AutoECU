/*
 * xControl.xc
 *
 *  Created on: Dec 4, 2020
 *      Author: denys.prokhorov
 */

#include <string.h>
#include "xProFIFO.h"
#include "xCommand.h"
#include "crc.h"
#include "ecu.h"
#include "delay.h"
#include "defines.h"

#ifndef taskENTER_CRITICAL
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 5
#define taskENTER_CRITICAL entercritical
static inline void entercritical(void)
{
  /*
  uint32_t ulNewBASEPRI;
  __asm volatile
  (
    " mov %0, %1                        \n" \
    " cpsid i                         \n" \
    " msr basepri, %0                     \n" \
    " isb                           \n" \
    " dsb                           \n" \
    " cpsie i                         \n" \
    :"=r" (ulNewBASEPRI) : "i" ( configMAX_SYSCALL_INTERRUPT_PRIORITY ) : "memory"
  );
  */
}
#endif

#ifndef taskEXIT_CRITICAL
#define taskEXIT_CRITICAL exitcritical
static inline void exitcritical(void)
{
  /*
  __asm volatile
  (
    " msr basepri, %0 " :: "r" ( 0 ) : "memory"
  );
  */
}
#endif

#define UART_DMA_BUFFER (MAX_PACK_LEN * 2)
#define RETRIES_TIMEOUT_PC 50000
#define RETRIES_TIMEOUT_CTRL 10000
#define RETRIES_MAX 20

typedef struct
{
    uint8_t BufRx[UART_DMA_BUFFER];
    uint8_t BufTx[MAX_PACK_LEN];
    uint8_t xRxFifoBuf[MAX_PACK_LEN*4];
    uint8_t xTxFifoBuf[MAX_PACK_LEN*4];
    uint8_t BufSender[MAX_PACK_LEN];
    uint8_t BufParser[MAX_PACK_LEN];
    UART_HandleTypeDef * xUart;
    eTransChannels xChannels[4];
    uint16_t ReceivedPackets[etrCount][10];
    uint16_t ReceivedPacketId[etrCount];
    sProFIFO xTxFifo;
    sProFIFO xRxFifo;
    uint32_t dataReceiving;
    uint32_t dataLen;
    uint16_t packetId;
    uint32_t RxPointer;
    volatile uint8_t TxBusy;
}sGetterHandle __attribute__((aligned(32)));

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;

static volatile uint16_t RetriesPacket = 0;
static volatile uint16_t NeedAckPacket = 0;
static volatile uint16_t ReceivedAckPacket = 1;
static volatile uint16_t NeededAckPacketId = 0;
static volatile uint32_t LastNotAckedTime = 0;

static sGetterHandle xHandles[] = {
    {{0},{0},{0},{0},{0},{0}, &huart5, {etrCTRL,etrPC,etrNone} },
    {{0},{0},{0},{0},{0},{0}, &huart4, {etrIMMO,etrNone} },
};

static inline int Msg_GetSrc(uint8_t xValue) { return (xValue & 7); }
static inline int Msg_GetDest(uint8_t xValue) { return ((xValue >> 3) & 7); }

static inline void CacheInvalidate(void * buffer, uint32_t size)
{
  uint32_t aligned = (uint32_t)buffer % 32;
  if(aligned == 0)
    SCB_InvalidateDCache_by_Addr((uint32_t*)buffer, size);
  else SCB_InvalidateDCache_by_Addr((uint32_t*)((uint32_t)buffer - aligned), size + aligned);
}

static inline void CacheClean(void * buffer, uint32_t size)
{
  uint32_t aligned = (uint32_t)buffer % 32;
  if(aligned == 0)
    SCB_CleanDCache_by_Addr((uint32_t*)buffer, size);
  else SCB_CleanDCache_by_Addr((uint32_t*)((uint32_t)buffer - aligned), size + aligned);
}

static inline uint16_t calculatePacketId(void)
{
  static uint16_t counter = 0;
  uint16_t returnvalue;
  do
  {
    counter++;
    uint16_t localcounter = counter;
    uint32_t now = Delay_Tick;
    uint8_t crcdata[6] = {(localcounter >> 8) & 0xFF,localcounter & 0xFF, (now >> 24) & 0xFF, (now >> 16) & 0xFF, (now >> 8) & 0xFF, now & 0xFF } ;
    returnvalue = CRC16_Generate(crcdata, sizeof(crcdata));
  } while(returnvalue == 0);
  return returnvalue;

}

static inline sGetterHandle * findHandleForChannel(eTransChannels xChannel)
{
  for(int i = 0; i < ITEMSOF(xHandles); i++)
  {
    for(int j = 0; j < ITEMSOF(xHandles[i].xChannels); j++)
    {
      if(xHandles[i].xChannels[j] == xChannel) {
        return &xHandles[i];
      }
    }
  }
  return NULL;
}

static inline void packager(sGetterHandle* xHandle, uint8_t* xMsgPtr, uint16_t xMsgLen, eTransChannels xChaDest, uint16_t aPacketId) {

    if (xHandle && xMsgLen<MAX_PACK_LEN)
    {
        uint16_t aCrc15 = 0;
        uint16_t aTotLen = xMsgLen ? xMsgLen + 10 : 8;
        uint8_t aHeadByte = etrECU | ( xChaDest << 3 );


        xHandle->BufSender[0] = 0x55;
        xHandle->BufSender[1] = 0x55;
        xHandle->BufSender[2] = aHeadByte;
        xHandle->BufSender[3] = aTotLen & 0xFF;
        xHandle->BufSender[4] = (aTotLen >> 8) & 0xFF;
        xHandle->BufSender[5] = aPacketId & 0xFF;
        xHandle->BufSender[6] = (aPacketId >> 8) & 0xFF;
        xHandle->BufSender[7] = CRC8_Generate(xHandle->BufSender, 7);


        if (xMsgLen)
        {
          memcpy(&xHandle->BufSender[8], xMsgPtr, xMsgLen);
          aCrc15 = CRC16_Generate(xHandle->BufSender, xMsgLen + 8);
        }

        uint8_t handled = 0;

        if(!protIsSome(&xHandle->xTxFifo))
        {
          taskENTER_CRITICAL();
          if(!xHandle->TxBusy)
          {
            xHandle->TxBusy = 1;
            handled = 1;
            taskEXIT_CRITICAL();

            memcpy(&xHandle->BufTx[0],xHandle->BufSender,8);
            if (xMsgLen) {
              memcpy(&xHandle->BufTx[8], &xHandle->BufSender[8],xMsgLen);
              memcpy(&xHandle->BufTx[xMsgLen+8], &aCrc15,2);
            }

            CacheClean(xHandle->BufTx, aTotLen);
            HAL_UART_Transmit_DMA(xHandle->xUart, xHandle->BufTx, aTotLen);
          }
          else taskEXIT_CRITICAL();
        }

        if(!handled)
        {
          protPushSequence(&xHandle->xTxFifo,xHandle->BufSender,8);
          if (xMsgLen) {
              protPushSequence(&xHandle->xTxFifo,&xHandle->BufSender[8],xMsgLen);
              protPushSequence(&xHandle->xTxFifo,&aCrc15,2);
          }
        }
    }
}

static inline void acker(sGetterHandle* xHandle, uint16_t aPacketId, eTransChannels xChaDest) {

    if (xHandle)
    {
        uint16_t aTotLen = 8;
        uint8_t aHeadByte = (( etrECU | ( xChaDest << 3 ) ) | HEADER_ACK_BIT);
        uint8_t header[8];

        header[0] = 0x55;
        header[1] = 0x55;
        header[2] = aHeadByte;
        header[3] = aTotLen & 0xFF;
        header[4] = (aTotLen >> 8) & 0xFF;
        header[5] = aPacketId & 0xFF;
        header[6] = (aPacketId >> 8) & 0xFF;
        header[7] = CRC8_Generate(header, 7);

        uint8_t handled = 0;
        if(!protIsSome(&xHandle->xTxFifo))
        {
          taskENTER_CRITICAL();
          if(!xHandle->TxBusy)
          {
            xHandle->TxBusy = 1;
            handled = 1;
            taskEXIT_CRITICAL();
            memcpy(xHandle->BufTx,header,8);
            CacheClean(xHandle->BufTx, 8);
            HAL_UART_Transmit_DMA(xHandle->xUart, xHandle->BufTx, 8);
          }
          else taskEXIT_CRITICAL();
        }

        if(!handled)
        {
          protPushSequence(&xHandle->xTxFifo,header,8);
        }
    }
}


int8_t xSender(eTransChannels xChaDest, uint8_t* xMsgPtr, uint32_t xMsgLen)
{
  uint32_t now = Delay_Tick;

  sGetterHandle * handle = NULL;

  handle = findHandleForChannel(xChaDest);

  if(!handle)
    return -2;

  taskENTER_CRITICAL();
  if(NeedAckPacket)
  {
    if(ReceivedAckPacket)
    {
      NeedAckPacket = 0;
      NeededAckPacketId = 0;
      taskEXIT_CRITICAL();
      return 1;
    }
    else
    {
      if(DelayDiff(now, LastNotAckedTime) > (xChaDest == etrPC ? RETRIES_TIMEOUT_PC : RETRIES_TIMEOUT_CTRL))
      {
        if(RetriesPacket > RETRIES_MAX)
        {
          NeedAckPacket = 0;
          NeededAckPacketId = 0;
          taskEXIT_CRITICAL();
          return -1;
        }
        LastNotAckedTime = now;
        RetriesPacket++;
        taskEXIT_CRITICAL();
        packager(handle, xMsgPtr, xMsgLen, xChaDest, NeededAckPacketId);
      }
      else taskEXIT_CRITICAL();
    }
  }
  else
  {
    ReceivedAckPacket = 0;
    NeedAckPacket = 1;
    LastNotAckedTime = now;
    RetriesPacket = 0;
    taskEXIT_CRITICAL();
    NeededAckPacketId = calculatePacketId();
    packager(handle, xMsgPtr, xMsgLen, xChaDest, NeededAckPacketId);
  }

  return 0;

}

static inline void parser(sProFIFO* xFifo, uint32_t xPacketId, uint32_t xDataLen, eTransChannels xChaSrc, eTransChannels xChaDest) {

  uint32_t aCount;
  uint8_t data, idis = 0;
  uint32_t sCount;
  sGetterHandle * hDest = NULL;
  uint8_t header[8];

  hDest = findHandleForChannel(xChaSrc);
  if(!hDest)
    return;

  switch (xChaDest)
  {

      case etrECU:
      {
          if (xDataLen)
          {
              for(int i = 0; i < 8; i++)
                protPull(xFifo, &header[i]);

              for (aCount = 0; aCount < xDataLen - 10; aCount++)
              {
                protPull(xFifo, &data);
                hDest->BufParser[aCount]=data;
              }
              protPull(xFifo, &data);
              protPull(xFifo, &data);

              hDest->BufParser[aCount]=0;

              if(hDest) acker(hDest,xPacketId,xChaSrc);

              for(int i = 0; i < 10; i++)
              {
                if(hDest->ReceivedPackets[xChaSrc][i] == xPacketId)
                {
                  idis = 1;
                  break;
                }
              }

              if(!idis)
              {
                hDest->ReceivedPackets[xChaSrc][hDest->ReceivedPacketId[xChaSrc]] = xPacketId;
                if(++hDest->ReceivedPacketId[xChaSrc] >= 10) hDest->ReceivedPacketId[xChaSrc] = 0;
                ecu_parse_command(xChaSrc, hDest->BufParser, aCount);
              }

          // Signal package
          }
          else
          {
              for (aCount = 0; aCount < 8; aCount++)
              {
                for(int i = 0; i < 8; i++)
                  protPull(xFifo, &header[i]);
              }

              taskENTER_CRITICAL();
              if(NeedAckPacket && NeededAckPacketId != 0 && NeededAckPacketId == xPacketId && !ReceivedAckPacket)
              {
                ReceivedAckPacket = 1;
              }
              taskEXIT_CRITICAL();

          }

          break;
      }

      case etrCTRL:
      case etrPC:
      case etrIMMO:
      {
        sCount = (xDataLen > 10) ? xDataLen : 8;

        if(hDest)
        {

          uint8_t handled = 0;
          if(!protIsSome(&hDest->xTxFifo))
          {
            taskENTER_CRITICAL();
            if(!hDest->TxBusy)
            {
              hDest->TxBusy = 1;
              handled = 1;
              taskEXIT_CRITICAL();

              for (aCount = 0; aCount < sCount; aCount++)
              {
                protPull(xFifo, &hDest->BufTx[aCount]);
              }

              CacheClean(hDest->BufTx, sCount);
              HAL_UART_Transmit_DMA(hDest->xUart, hDest->BufTx, sCount);
            }
            else taskEXIT_CRITICAL();
          }

          if(!handled)
          {
            for (aCount = 0; aCount < sCount; aCount++)
            {
              protPull(xFifo, &data);
              protPush(&hDest->xTxFifo, &data);
            }
          }

          break;
        }
      }
      /* no break */

      default:
      {
        sCount = (xDataLen > 10) ? xDataLen : 8;
        for (aCount = 0; aCount < sCount; aCount++)
        {
          protPull(xFifo, &data);
        }
        break;
      }
  }
}

static inline uint8_t lookByte(sProFIFO* xFifo, uint32_t xOffset) { uint8_t aByte; protLook(xFifo,xOffset,&aByte); return aByte; }

static inline uint8_t countCRC8(sGetterHandle * handle) {
    uint32_t i; uint8_t aCrc8 = 0;
    for (i=0; i<7; i++) { handle->BufParser[i] = lookByte(&handle->xRxFifo,i); }
    aCrc8 = CRC8_Generate(handle->BufParser, 7);
    return aCrc8;
}

static inline int32_t countCRC16(sGetterHandle * handle, uint32_t xLen) {
    uint32_t i; int32_t aCrc16 = 0;
    for (i=0; i<xLen-2; i++) { handle->BufParser[i] = lookByte(&handle->xRxFifo,i); }
    aCrc16 = CRC16_Generate(handle->BufParser, xLen-2);
    return aCrc16;
}

static void Getter(sGetterHandle * handle)
{
  uint32_t dataSkip = 0;
  sProFIFO* xFifo = &handle->xRxFifo;
  uint32_t * pDataReceiving = &handle->dataReceiving;
  uint32_t * pDataLen = &handle->dataLen;
  uint16_t * pPacketId = &handle->packetId;

  uint16_t packetId = *pPacketId;
  uint32_t dataLen = *pDataLen;
  uint32_t dataReceiving = *pDataReceiving;
  if(dataReceiving)
  {
    // Check if we got a data
    if (protGetSize(xFifo) >= dataLen)
    {
        if (countCRC16(handle,dataLen) == lookByte(xFifo,dataLen-2) + (lookByte(xFifo,dataLen-1) << 8))
        {
            // Got True package
            parser(xFifo,packetId,dataLen,Msg_GetSrc(lookByte(xFifo,2)),Msg_GetDest(lookByte(xFifo,2)));
        }
        else { dataSkip=1; } // Wrong CRC16, so skip 1 byte
        dataReceiving = 0;
        dataLen = 0;
    }
  }
  else
  {
    if (protGetSize(xFifo) > 7)
    {
      if(lookByte(xFifo,0) == 0x55 && lookByte(xFifo,1) == 0x55)
      {
        if (countCRC8(handle) == lookByte(xFifo,7))
        {
          dataLen = lookByte(xFifo,3) + (lookByte(xFifo,4) << 8);
          packetId = lookByte(xFifo,5) + (lookByte(xFifo,6) << 8);
          if (packetId > 0 && dataLen < MAX_PACK_LEN)
          {
              if (dataLen>10)
              {
                dataReceiving = 1;
              }
              else
              {
                  // Got ShortPackage (Header Only)
                  parser(xFifo,packetId,0,Msg_GetSrc(lookByte(xFifo,2)),Msg_GetDest(lookByte(xFifo,2)));
              }
          }
          else { dataSkip=1; } // Wrong data length or packet id, so skip 1 byte
        }
        else { dataSkip=1; } // Wrong CRC8, so skip 1 byte
      }
      else { dataSkip=1; } // Wrong sync bytes
    }
  }

  if (dataSkip)
  {
    protMoveRead(xFifo,dataSkip);
  }

  *pDataReceiving = dataReceiving;
  *pDataLen = dataLen;
  *pPacketId = packetId;
}



void xDmaTxIrqHandler(UART_HandleTypeDef *huart)
{
  sGetterHandle * handle;
  for(int i = 0; i < ITEMSOF(xHandles); i++)
  {
    handle = &xHandles[i];
    uint32_t length = 0;
    if(huart == handle->xUart)
    {
      if(protIsSome(&handle->xTxFifo))
      {
        handle->TxBusy = 1;
        while(protPull(&handle->xTxFifo, &handle->BufTx[length++])
            && length < MAX_PACK_LEN);
        CacheClean(handle->BufTx, length);
        HAL_UART_Transmit_DMA(handle->xUart, handle->BufTx, length);
      }
      else handle->TxBusy = 0;
      break;
    }
  }
}

void xDmaErIrqHandler(UART_HandleTypeDef *huart)
{
  sGetterHandle * handle;
  for(int i = 0; i < ITEMSOF(xHandles); i++)
  {
    handle = &xHandles[i];
    if(huart == handle->xUart)
    {
      HAL_UART_Receive_DMA(handle->xUart, handle->BufRx, UART_DMA_BUFFER);
      handle->RxPointer = handle->xUart->RxXferSize;
      break;
    }
  }
}


void xFifosInit(void)
{
  for(int i = 0; i < sizeof(xHandles) / sizeof(xHandles[0]); i++)
  {
    protInit(&xHandles[i].xTxFifo,xHandles[i].xTxFifoBuf,1,MAX_PACK_LEN*4);
    protInit(&xHandles[i].xRxFifo,xHandles[i].xRxFifoBuf,1,MAX_PACK_LEN*4);
    xHandles[i].RxPointer = 0xFFFFFFFF;
  }
}

void xGetterInit(void)
{
  sGetterHandle * handle;

  for(int i = 0; i < sizeof(xHandles) / sizeof(xHandles[0]); i++)
  {
    handle = &xHandles[i];
    CacheClean(handle->BufRx, UART_DMA_BUFFER);
    HAL_UART_Receive_DMA(handle->xUart, handle->BufRx, UART_DMA_BUFFER);
    handle->RxPointer = handle->xUart->RxXferSize;
  }
}

void xGetterLoop(void)
{
  sGetterHandle * handle;
  uint32_t dmacnt;
  uint32_t length;
  uint32_t dmasize;
  uint8_t tempbuffer[MAX_PACK_LEN];

  for(int i = 0; i < sizeof(xHandles) / sizeof(xHandles[0]); i++)
  {
    handle = &xHandles[i];
    do
    {
      dmacnt = handle->xUart->hdmarx->Instance->NDTR;
      dmasize = handle->xUart->RxXferSize;
      if(handle->RxPointer == 0xFFFFFFFF) handle->RxPointer = dmacnt;
      if(dmacnt > handle->RxPointer)
        length = (dmasize-dmacnt)+handle->RxPointer;
      else length = handle->RxPointer-dmacnt;

      if(length > MAX_PACK_LEN) length = MAX_PACK_LEN;
      if(length > 0)
      {
        CacheInvalidate(handle->BufRx, UART_DMA_BUFFER);
        for(i=0;i<length;i++)
        {
          tempbuffer[i] = handle->BufRx[dmasize-handle->RxPointer];
          if(handle->RxPointer == 1) handle->RxPointer = dmasize;
          else handle->RxPointer--;
        }

        protPushSequence(&handle->xRxFifo, tempbuffer, length);
      }
    } while(length > 0);

    if(protIsSome(&handle->xRxFifo))
    {
      Getter(handle);
    }

    taskENTER_CRITICAL();
    if(!handle->TxBusy && protIsSome(&handle->xTxFifo))
    {
      length = 0;
      handle->TxBusy = 1;
      taskEXIT_CRITICAL();
      while(protPull(&handle->xTxFifo, &handle->BufTx[length++])
          && length < MAX_PACK_LEN);
      CacheClean(handle->BufTx, length);
      HAL_UART_Transmit_DMA(handle->xUart, handle->BufTx, length);
    }
    else taskEXIT_CRITICAL();
  }
}

