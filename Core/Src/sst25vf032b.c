#include "sst25vf032b.h"
#include "delay.h"

#define SPI_NSS_ON() HAL_GPIO_WritePin(SPI2_NSS_FLASH_GPIO_Port, SPI2_NSS_FLASH_Pin, GPIO_PIN_RESET)
#define SPI_NSS_OFF() HAL_GPIO_WritePin(SPI2_NSS_FLASH_GPIO_Port, SPI2_NSS_FLASH_Pin, GPIO_PIN_SET)

static SPI_HandleTypeDef * hspi;

static uint8_t tx[32] __attribute__((aligned(32)));
static uint8_t rx[32] __attribute__((aligned(32)));

static volatile uint8_t semTx = 0;
static volatile uint8_t semRx = 0;
static volatile uint8_t avoid_nss_reset = 0;

void SST25_ErrorCallback(SPI_HandleTypeDef * _hspi)
{
  if(_hspi == hspi) {
  }
}

void SST25_TxCpltCallback(SPI_HandleTypeDef * _hspi)
{
  if(_hspi == hspi) {
    if(avoid_nss_reset) {
      avoid_nss_reset = 0;
    } else SPI_NSS_OFF();
    semTx = 1;
  }
}

void SST25_RxCpltCallback(SPI_HandleTypeDef * _hspi)
{
  if(_hspi == hspi) {
    if(avoid_nss_reset) {
      avoid_nss_reset = 0;
    } else SPI_NSS_OFF();
    semRx = 1;
  }
}

void SST25_TxRxCpltCallback(SPI_HandleTypeDef * _hspi)
{
  if(_hspi == hspi) {
    if(avoid_nss_reset) {
      avoid_nss_reset = 0;
    } else SPI_NSS_OFF();
    semTx = 1;
    semRx = 1;
  }
}

static uint8_t waitTxCplt()
{
  if(semTx) {
    semTx = 0;
    return 1;
  }
  return 0;
}

static uint8_t waitRxCplt()
{
  if(semRx) {
    semRx = 0;
    return 1;
  }
  return 0;
}

static uint8_t waitTxRxCplt()
{
  if(semRx && semTx) {
    semRx = 0;
    semTx = 0;
    return 1;
  }
  return 0;
}

static HAL_StatusTypeDef SPI_CheckChip(void)
{
  tx[0] = 0x90;
  tx[1] = 0;
  tx[2] = 0;
  tx[3] = 0;
  tx[4] = 0;
  tx[5] = 0;

  SCB_CleanDCache_by_Addr((uint32_t*)tx, 6);

  SPI_NSS_ON();
  HAL_SPI_TransmitReceive_DMA(hspi, tx, rx, 6);
  while(!waitTxRxCplt()) {}
  //SPI_NSS_OFF();

  SCB_InvalidateDCache_by_Addr((uint32_t*)rx, 6);

  if(rx[4] != 0xBF)
    return HAL_ERROR;
  if(rx[5] != 0x4A)
    return HAL_ERROR;

  HAL_GPIO_WritePin(SPI2_WP_GPIO_Port, SPI2_WP_Pin, GPIO_PIN_SET);
  tx[0] = 0x06;
  SPI_NSS_ON();
  HAL_SPI_Transmit_IT(hspi, tx, 1);
  while(!waitTxCplt()) {}
  //SPI_NSS_OFF();

  tx[0] = 0x01;
  tx[1] = 0x00;
  SPI_NSS_ON();
  HAL_SPI_Transmit_IT(hspi, tx, 2);
  while(!waitTxCplt()) {}
  //SPI_NSS_OFF();
  HAL_GPIO_WritePin(SPI2_WP_GPIO_Port, SPI2_WP_Pin, GPIO_PIN_RESET);

  return HAL_OK;
}

static uint8_t SPI_Read(uint32_t address, uint32_t size, uint8_t * buffer)
{
  static uint8_t state = 0;

  switch(state)
  {
    case 0 :
      tx[0] = 0x0B;
      tx[1] = (address >> 16) & 0xFF;
      tx[2] = (address >> 8) & 0xFF;
      tx[3] = address & 0xFF;
      tx[4] = 0xFF;

      SCB_CleanDCache_by_Addr((uint32_t*)tx, 5);

      if((uint32_t)buffer % 32 == 0)
        SCB_CleanDCache_by_Addr((uint32_t*)buffer, size);
      else SCB_CleanDCache_by_Addr((uint32_t*)((uint32_t)buffer - ((uint32_t)buffer%32)), size + ((uint32_t)buffer%32));

      SPI_NSS_ON();
      avoid_nss_reset = 1;
      HAL_SPI_Transmit_DMA(hspi, tx, 5);
      state++;
      break;
    case 1 :
      if(waitTxCplt())
      {
        HAL_SPI_Receive_DMA(hspi, buffer, size);
        state++;
      }
      break;
    case 2 :
      if(waitRxCplt())
      {
        //SPI_NSS_OFF();

        if((uint32_t)buffer % 32 == 0)
          SCB_InvalidateDCache_by_Addr((uint32_t*)buffer, size);
        else SCB_InvalidateDCache_by_Addr((uint32_t*)((uint32_t)buffer - ((uint32_t)buffer%32)), size + ((uint32_t)buffer%32));
        state = 0;
        return 1;
      }
      break;
    default :
      state = 0;
      break;
  }
  return 0;
}
static uint8_t SPI_WaitForWrite(void)
{
  static uint8_t state = 0;
  static uint32_t wait_time = 0;
  uint32_t now = Delay_Tick;
  do
  {
    switch(state)
    {
      case 0 :
        tx[0] = 0x05;
        wait_time = now;
        SPI_NSS_ON();
        state++;
        continue;
      case 1 :
        avoid_nss_reset = 1;
        HAL_SPI_Transmit_IT(hspi, tx, 1);
        state++;
        break;
      case 2 :
        if(waitTxCplt())
        {
          avoid_nss_reset = 1;
          HAL_SPI_Receive_IT(hspi, rx, 1);
          state++;
        }
        break;
      case 3 :
        if(waitRxCplt())
        {
          if(rx[0] & 0x01)
          {
            //Never should get here...
            if(DelayDiff(now, wait_time) > 500000) {
              SPI_NSS_OFF();
              state = 0;
              return -1;
            }
            state = 1;
          }
          else
          {
            SPI_NSS_OFF();
            state = 0;
            return 1;
          }
        }
        break;
      default :
        state = 0;
        continue;
    }
  }
  while(0);
  return 0;
}

static uint8_t SPI_Write(uint32_t addr, uint32_t size, const uint8_t * buffer)
{
  static uint8_t state = 0;
  static uint32_t left = 0;
  static const uint8_t * pointer = NULL;
  static uint8_t ftime = 0;
  static uint32_t tx_time = 0;
  static uint8_t tx_cplt = 0;
  uint8_t writesize;

  do {
    switch(state)
    {
      case 0 :
        HAL_GPIO_WritePin(SPI2_WP_GPIO_Port, SPI2_WP_Pin, GPIO_PIN_SET);
        left = size;
        pointer = buffer;
        ftime = 1;
        state = 1;
        break;
      case 1 :
        tx[0] = 0x06;
        SPI_NSS_ON();
        HAL_SPI_Transmit_IT(hspi, tx, 1);
        state++;
        break;
      case 2 :
        if(waitTxCplt())
        {
          //SPI_NSS_OFF();
          state++;
        }
        continue;
      case 3 :
        if(ftime) {
          ftime = 0;
          tx[0] = 0xAD;
          tx[1] = (addr >> 16) & 0xFF;
          tx[2] = (addr >> 8) & 0xFF;
          tx[3] = addr & 0xFF;
          if(left >= 2) {
            tx[4] = *pointer++;
            tx[5] = *pointer++;
            left -= 2;
          } else {
            tx[4] = *pointer++;
            tx[5] = 0xFF;
            left -= 1;
          }
          writesize = 6;
        } else  {
          tx[0] = 0xAD;
          if(left >= 2) {
            tx[1] = *pointer++;
            tx[2] = *pointer++;
            left -= 2;
          } else {
            tx[1] = *pointer++;
            tx[2] = 0xFF;
            left -= 1;
          }
          writesize = 3;
        }

        SPI_NSS_ON();
        SCB_CleanDCache_by_Addr((uint32_t*)tx, writesize);
        HAL_SPI_Transmit_DMA(hspi, tx, writesize);

        tx_cplt = 0;
        tx_time = Delay_Tick;
        state++;
        break;
      case 4 :
        if(tx_cplt || waitTxCplt()) {
          //SPI_NSS_OFF();
          tx_cplt = 1;
          if(DelayDiff(Delay_Tick, tx_time) > 10 + 1) {
            if(left) {
              state = 3;
              continue;
            } else {
              state++;
            }
          }
        }
        break;
      case 5 :
        tx[0] = 0x04;
        SPI_NSS_ON();
        HAL_SPI_Transmit_IT(hspi, tx, 1);
        state++;
        break;
      case 6 :
        if(waitTxCplt())
        {
          //SPI_NSS_OFF();
          HAL_GPIO_WritePin(SPI2_WP_GPIO_Port, SPI2_WP_Pin, GPIO_PIN_RESET);
          state = 0;
          return 1;
        }
        break;
      default :
        state = 0;
        break;
    }
  } while(0);

  return 0;
}

static uint8_t SPI_EraseBlock(uint32_t address, uint8_t cmd)
{
  static uint8_t state = 0;
  static uint32_t program_time = 0;

  do
  {
    switch(state)
    {
      case 0 :
        HAL_GPIO_WritePin(SPI2_WP_GPIO_Port, SPI2_WP_Pin, GPIO_PIN_SET);
        tx[0] = 0x06;
        SPI_NSS_ON();
        HAL_SPI_Transmit_IT(hspi, tx, 1);
        state++;
        break;
      case 1 :
        if(waitTxCplt())
        {
          //SPI_NSS_OFF();
          tx[0] = cmd;
          tx[1] = (address >> 16) & 0xFF;
          tx[2] = (address >> 8) & 0xFF;
          tx[3] = address & 0xFF;
          SPI_NSS_ON();
          SCB_CleanDCache_by_Addr((uint32_t*)tx, 4);
          HAL_SPI_Transmit_DMA(hspi, tx, 4);
          state++;
        }
        break;
      case 2 :
        if(waitTxCplt())
        {
          //SPI_NSS_OFF();
          program_time = Delay_Tick;
          state++;
        }
        break;
      case 3 :
        if(DelayDiff(Delay_Tick, program_time) >= 25000+3000)
        {
          state++;
          continue;
        }
        break;
      case 4 :
        if(SPI_WaitForWrite())
        {
          state = 0;
          HAL_GPIO_WritePin(SPI2_WP_GPIO_Port, SPI2_WP_Pin, GPIO_PIN_RESET);
          return 1;
        }
        break;
      default :
        state = 0;
        continue;
    }
  } while(0);
  return 0;
}

static uint8_t SPI_ChipErase(void)
{
  static uint8_t state = 0;
  static uint32_t program_time = 0;

  do
  {
    switch(state)
    {
      case 0 :
        HAL_GPIO_WritePin(SPI2_WP_GPIO_Port, SPI2_WP_Pin, GPIO_PIN_SET);
        tx[0] = 0x06;
        SPI_NSS_ON();
        HAL_SPI_Transmit_IT(hspi, tx, 1);
        state++;
        break;
      case 1 :
        if(waitTxCplt())
        {
          //SPI_NSS_OFF();
          tx[0] = 0x60;
          SPI_NSS_ON();
          HAL_SPI_Transmit_IT(hspi, tx, 1);
          state++;
        }
        break;
      case 2 :
        if(waitTxCplt())
        {
          //SPI_NSS_OFF();
          program_time = Delay_Tick;
          state++;
        }
        break;
      case 3 :
        if(DelayDiff(Delay_Tick, program_time) >= 50000+2000)
        {
          state++;
          continue;
        }
        break;
      case 4 :
        if(SPI_WaitForWrite())
        {
          state = 0;
          HAL_GPIO_WritePin(SPI2_WP_GPIO_Port, SPI2_WP_Pin, GPIO_PIN_RESET);
          return 1;
        }
        break;
      default :
        state = 0;
        continue;
    }
  } while(0);
  return 0;
}

HAL_StatusTypeDef SST25_CheckChip(void)
{
  return SPI_CheckChip();
}

HAL_StatusTypeDef SST25_Init(SPI_HandleTypeDef * _hspi)
{
  HAL_StatusTypeDef status = HAL_OK;
  hspi = _hspi;

  SCB_CleanDCache_by_Addr((uint32_t*)tx, sizeof(tx));
  SCB_CleanDCache_by_Addr((uint32_t*)rx, sizeof(rx));

  return status;
}

uint8_t SST25_Erase4KSector(uint32_t address)
{
  return SPI_EraseBlock(address & 0x3FFFFF, 0x20);
}

uint8_t SST25_Erase32KBlock(uint32_t address)
{
  return SPI_EraseBlock(address & 0x3FFFFF, 0x52);
}

uint8_t SST25_Erase64KBlock(uint32_t address)
{
  return SPI_EraseBlock(address & 0x3FFFFF, 0xD8);
}

uint8_t SST25_ChipErase(void)
{
  return SPI_ChipErase();
}

uint8_t SST25_Read(uint32_t address, uint32_t size, uint8_t * buffer)
{
  return SPI_Read(address & 0x3FFFFF, size, buffer);
}

uint8_t SST25_Write(uint32_t address, uint32_t size, const uint8_t * buffer)
{
  return SPI_Write(address & 0x3FFFFF, size, buffer);
}

void SST25_ChipEraseLock(void)
{
  while(!SPI_ChipErase()) {}
}

void SST25_Erase4KSectorLock(uint32_t address)
{
  while(!SPI_EraseBlock(address & 0x3FFFFF, 0x20)) {}
}

void SST25_Erase32KBlockLock(uint32_t address)
{
  while(!SPI_EraseBlock(address & 0x3FFFFF, 0x52)) {}
}

void SST25_Erase64KBlockLock(uint32_t address)
{
  while(!SPI_EraseBlock(address & 0x3FFFFF, 0xD8)) {}
}

void SST25_ReadLock(uint32_t address, uint32_t size, uint8_t * buffer)
{
  while(!SPI_Read(address & 0x3FFFFF, size, buffer)) {}
}

void SST25_WriteLock(uint32_t address, uint32_t size, const uint8_t * buffer)
{
  while(!SPI_Write(address & 0x3FFFFF, size, buffer)) {}
}

