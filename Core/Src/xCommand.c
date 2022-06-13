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
#include "defines.h"
#ifdef VIRTUALCOMPORT
#include "usbd_cdc_if.h"
#endif

#ifndef taskENTER_CRITICAL
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 5
#define taskENTER_CRITICAL entercritical
STATIC_INLINE void entercritical(void)
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
STATIC_INLINE void exitcritical(void)
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

#define SOURCE etrECU

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

    volatile uint16_t ReceivedAckPacket;
    volatile uint16_t RetriesPacket;
    volatile uint16_t NeedAckPacket;
    volatile uint16_t NeededAckPacketId;
    volatile uint32_t LastNotAckedTime;
    volatile uint8_t TxBusy;
    volatile eTransChannels TxDest;

    uint16_t ReceivedPackets[etrCount][10];
    uint16_t ReceivedPacketId[etrCount];
    sProFIFO xTxFifo;
    sProFIFO xRxFifo;
    uint32_t dataReceiving;
    uint32_t dataLen;
    uint16_t packetId;
    uint32_t RxPointer;
}sGetterHandle __attribute__((aligned(32)));

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart8;

static sGetterHandle xHandles[] = {
#ifdef VIRTUALCOMPORT
    {{0},{0},{0},{0},{0},{0}, &huart5, {etrCTRL,etrNone}, 1,0,0,0,0,0, etrNone },
    {{0},{0},{0},{0},{0},{0}, &huart8, {etrBT,etrNone}, 1,0,0,0,0,0, etrNone },
    {{0},{0},{0},{0},{0},{0}, NULL, {etrPC,etrNone}, 1,0,0,0,0,0, etrNone },
#else
    {{0},{0},{0},{0},{0},{0}, &huart5, {etrCTRL,etrPC,etrNone}, 1,0,0,0,0,0, etrNone },
    {{0},{0},{0},{0},{0},{0}, &huart8, {etrBT,etrNone}, 1,0,0,0,0,0, etrNone },
#endif
};

STATIC_INLINE uint16_t calculatePacketId(void)
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

STATIC_INLINE sGetterHandle * findHandleForChannel(eTransChannels xChannel)
{
  for(int i = 0; i < ITEMSOF(xHandles); i++)
  {
    for(int j = 0; j < ITEMSOF(xHandles[i].xChannels); j++)
    {
      if(xHandles[i].xChannels[j] == xChannel) {
        return &xHandles[i];
      } else if(xHandles[i].xChannels[j] == etrNone) {
        break;
      }
    }
  }
  return NULL;
}

STATIC_INLINE void packager(sGetterHandle* xHandle, const uint8_t* xMsgPtr, uint16_t xMsgLen, eTransChannels xChaDest, uint16_t aPacketId) {

    if (xHandle && xMsgLen<MAX_PACK_LEN)
    {
        uint16_t aCrc15 = 0;
        uint16_t aTotLen = xMsgLen ? xMsgLen + 8 + 2 : 8;

        xHandle->BufSender[0] = 0x55;
        xHandle->BufSender[1] = SOURCE;
        xHandle->BufSender[2] = xChaDest;
        xHandle->BufSender[3] = aTotLen & 0xFF;
        xHandle->BufSender[4] = (aTotLen >> 8) & 0xFF;
        xHandle->BufSender[5] = aPacketId & 0xFF;
        xHandle->BufSender[6] = (aPacketId >> 8) & 0xFF;
        xHandle->BufSender[7] = CRC8_Generate(xHandle->BufSender, 7);


        if (xMsgLen)
        {
          memcpy(&xHandle->BufSender[8], xMsgPtr, xMsgLen);
          aCrc15 = CRC16_Generate(xHandle->BufSender, xMsgLen + 8);
          memcpy(&xHandle->BufSender[xMsgLen+8], &aCrc15,2);
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
              memcpy(&xHandle->BufTx[8], &xHandle->BufSender[8], xMsgLen + 2);
            }

            if(xHandle->xUart) {
              if(xHandle->xUart->hdmatx) {
                CacheClean(xHandle->BufTx, aTotLen);
                HAL_UART_Transmit_DMA(xHandle->xUart, xHandle->BufTx, aTotLen);
              }
              else
                HAL_UART_Transmit_IT(xHandle->xUart, xHandle->BufTx, aTotLen);
            } else {
#ifdef VIRTUALCOMPORT
              CDC_Transmit(xHandle->BufTx, aTotLen);
#endif
            }
          }
          else taskEXIT_CRITICAL();
        }

        if(!handled)
        {
          protPushSequence(&xHandle->xTxFifo,xHandle->BufSender,8);
          if (xMsgLen) {
              protPushSequence(&xHandle->xTxFifo,&xHandle->BufSender[8],xMsgLen + 2);
          }
        }
    }
}

STATIC_INLINE void acker(sGetterHandle* xHandle, uint16_t aPacketId, eTransChannels xChaDest) {

    if (xHandle)
    {
        uint16_t aTotLen = 8;
        uint8_t header[8];

        header[0] = 0x55;
        header[1] = SOURCE;
        header[2] = xChaDest;
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
            if(xHandle->xUart) {
              if(xHandle->xUart->hdmatx) {
                CacheClean(xHandle->BufTx, 8);
                HAL_UART_Transmit_DMA(xHandle->xUart, xHandle->BufTx, 8);
              }
              else
                HAL_UART_Transmit_IT(xHandle->xUart, xHandle->BufTx, 8);
            } else {
#ifdef VIRTUALCOMPORT
              CDC_Transmit(xHandle->BufTx, 8);
#endif
            }
          }
          else taskEXIT_CRITICAL();
        }

        if(!handled)
        {
          protPushSequence(&xHandle->xTxFifo,header,8);
        }
    }
}

void xSenderRaw(eTransChannels xChaDest, const uint8_t* xMsgPtr, uint32_t xMsgLen)
{
  sGetterHandle * xHandle = NULL;
  xHandle = findHandleForChannel(xChaDest);

  uint8_t handled = 0;

  if(!protIsSome(&xHandle->xTxFifo))
  {
    taskENTER_CRITICAL();
    if(!xHandle->TxBusy)
    {
      xHandle->TxBusy = 1;
      handled = 1;
      taskEXIT_CRITICAL();

      memcpy(&xHandle->BufTx[0],xMsgPtr,xMsgLen);

      if(xHandle->xUart) {
        if(xHandle->xUart->hdmatx) {
          CacheClean(xHandle->BufTx, xMsgLen);
          HAL_UART_Transmit_DMA(xHandle->xUart, xHandle->BufTx, xMsgLen);
        }
        else
          HAL_UART_Transmit_IT(xHandle->xUart, xHandle->BufTx, xMsgLen);
      } else {
#ifdef VIRTUALCOMPORT
        CDC_Transmit(xHandle->BufTx, xMsgLen);
#endif
      }
    }
    else taskEXIT_CRITICAL();
  }

  if(!handled)
  {
    protPushSequence(&xHandle->xTxFifo,xMsgPtr,xMsgLen);
  }
}

int8_t xSender(eTransChannels xChaDest, const uint8_t* xMsgPtr, uint32_t xMsgLen)
{
  uint32_t now = Delay_Tick;

  sGetterHandle * handle = NULL;

  handle = findHandleForChannel(xChaDest);

  if(!handle)
    return -2;

  taskENTER_CRITICAL();
  if(handle->TxDest != etrNone && xChaDest != handle->TxDest) {
    taskEXIT_CRITICAL();
    return 0;
  }

  if(handle->NeedAckPacket)
  {
    if(handle->ReceivedAckPacket)
    {
      handle->NeedAckPacket = 0;
      handle->NeededAckPacketId = 0;
      handle->TxDest = etrNone;
      taskEXIT_CRITICAL();
      return 1;
    }
    else
    {
      if(DelayDiff(now, handle->LastNotAckedTime) > (xChaDest == etrPC ? RETRIES_TIMEOUT_PC : RETRIES_TIMEOUT_CTRL))
      {
        if(handle->RetriesPacket > RETRIES_MAX)
        {
          handle->NeedAckPacket = 0;
          handle->NeededAckPacketId = 0;
          handle->TxDest = etrNone;
          taskEXIT_CRITICAL();
          return -1;
        }
        handle->LastNotAckedTime = now;
        handle->RetriesPacket++;
        handle->TxDest = xChaDest;
        taskEXIT_CRITICAL();
        packager(handle, xMsgPtr, xMsgLen, xChaDest, handle->NeededAckPacketId);
      }
      else taskEXIT_CRITICAL();
    }
  }
  else
  {
    handle->ReceivedAckPacket = 0;
    handle->NeedAckPacket = 1;
    handle->LastNotAckedTime = now;
    handle->RetriesPacket = 0;
    handle->TxDest = xChaDest;
    taskEXIT_CRITICAL();
    handle->NeededAckPacketId = calculatePacketId();
    packager(handle, xMsgPtr, xMsgLen, xChaDest, handle->NeededAckPacketId);
  }

  return 0;

}

STATIC_INLINE void parser(sProFIFO* xFifo, uint32_t xPacketId, uint32_t xDataLen, eTransChannels xChaSrc, eTransChannels xChaDest) {

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
              if(hDest->NeedAckPacket && hDest->NeededAckPacketId != 0 && hDest->NeededAckPacketId == xPacketId && !hDest->ReceivedAckPacket)
              {
                hDest->ReceivedAckPacket = 1;
              }
              taskEXIT_CRITICAL();

          }

          break;
      }
      case etrCTRL:
      case etrPC:
      case etrBT:
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

              if(hDest->xUart) {
                if(hDest->xUart->hdmatx) {
                  CacheClean(hDest->BufTx, sCount);
                  HAL_UART_Transmit_DMA(hDest->xUart, hDest->BufTx, sCount);
                }
                else
                  HAL_UART_Transmit_IT(hDest->xUart, hDest->BufTx, sCount);
              } else {
#ifdef VIRTUALCOMPORT
                CDC_Transmit(hDest->BufTx, sCount);
#endif
              }
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

STATIC_INLINE uint8_t lookByte(sProFIFO* xFifo, uint32_t xOffset) { uint8_t aByte; protLook(xFifo,xOffset,&aByte); return aByte; }

STATIC_INLINE uint8_t countCRC8(sGetterHandle * handle) {
    uint32_t i; uint8_t aCrc8 = 0;
    for (i=0; i<7; i++) { handle->BufParser[i] = lookByte(&handle->xRxFifo,i); }
    aCrc8 = CRC8_Generate(handle->BufParser, 7);
    return aCrc8;
}

STATIC_INLINE int32_t countCRC16(sGetterHandle * handle, uint32_t xLen) {
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
            parser(xFifo,packetId,dataLen,lookByte(xFifo,1),lookByte(xFifo,2));
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
      if(lookByte(xFifo,0) == 0x55)
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
                  parser(xFifo,packetId,0,lookByte(xFifo,1),lookByte(xFifo,2));
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
        if(handle->xUart) {
          if(handle->xUart->hdmatx) {
            CacheClean(handle->BufTx, length);
            HAL_UART_Transmit_DMA(handle->xUart, handle->BufTx, length);
          }
          else
            HAL_UART_Transmit_IT(handle->xUart, handle->BufTx, length);
        } else {
#ifdef VIRTUALCOMPORT
          CDC_Transmit(handle->BufTx, length);
#endif
        }
      }
      else handle->TxBusy = 0;
      break;
    }
  }
}


#ifdef VIRTUALCOMPORT
void CDC_TxCpltCallback(void)
{
  sGetterHandle * handle;
  for(int i = 0; i < ITEMSOF(xHandles); i++)
  {
    handle = &xHandles[i];
    uint32_t length = 0;
    if(!handle->xUart)
    {
      if(protIsSome(&handle->xTxFifo))
      {
        handle->TxBusy = 1;
        while(protPull(&handle->xTxFifo, &handle->BufTx[length++])
            && length < MAX_PACK_LEN);
        CDC_Transmit(handle->BufTx, length);
      }
      else handle->TxBusy = 0;
      break;
    }
  }
}

void CDC_ReceiveCallback(uint8_t *buffer, uint32_t length)
{
  sGetterHandle * handle;
  for(int i = 0; i < ITEMSOF(xHandles); i++)
  {
    handle = &xHandles[i];
    if(!handle->xUart) {
        protPushSequence(&handle->xRxFifo, buffer, length);
    }
  }
}
#endif

void xDmaRxIrqHandler(UART_HandleTypeDef *huart)
{
  sGetterHandle * handle;
  for(int i = 0; i < ITEMSOF(xHandles); i++)
  {
    handle = &xHandles[i];
    if(huart == handle->xUart && !handle->xUart->hdmarx)
    {
      protPush(&handle->xRxFifo, &handle->BufRx[0]);
      HAL_UART_Receive_IT(handle->xUart, &handle->BufRx[0], 1);
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
    if(handle->xUart) {
      if(handle->xUart->hdmarx) {
        CacheClean(handle->BufRx, UART_DMA_BUFFER);
        HAL_UART_Receive_DMA(handle->xUart, handle->BufRx, UART_DMA_BUFFER);
      } else {
        HAL_UART_Receive_IT(handle->xUart, handle->BufRx, 1);
      }
    }
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
    if(handle->xUart) {
      if(handle->xUart->hdmarx) {
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
      }
    }

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

      if(handle->xUart) {
        if(handle->xUart->hdmatx) {
          CacheClean(handle->BufTx, length);
          HAL_UART_Transmit_DMA(handle->xUart, handle->BufTx, length);
        }
        else
          HAL_UART_Transmit_IT(handle->xUart, handle->BufTx, length);
      } else {
#ifdef VIRTUALCOMPORT
        CDC_Transmit(handle->BufTx, length);
#endif
      }
    }
    else taskEXIT_CRITICAL();
  }
}

