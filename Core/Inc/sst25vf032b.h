#include "main.h"

#define SST25_SECTORSIZE 4096

HAL_StatusTypeDef SST25_Init(SPI_HandleTypeDef * _hspi);
uint8_t SST25_Read(uint32_t address, uint32_t size, uint8_t * buffer);
uint8_t SST25_Write(uint32_t address, uint32_t size, const uint8_t * buffer);
uint8_t SST25_Erase4KSector(uint32_t address);
uint8_t SST25_ChipErase(void);
void SST25_ReadLock(uint32_t address, uint32_t size, uint8_t * buffer);
void SST25_WriteLock(uint32_t address, uint32_t size, const uint8_t * buffer);
void SST25_Erase4KSectorLock(uint32_t address);
void SST25_ChipEraseLock(void);

void SST25_ErrorCallback(SPI_HandleTypeDef * _hspi);
void SST25_TxCpltCallback(SPI_HandleTypeDef * _hspi);
void SST25_RxCpltCallback(SPI_HandleTypeDef * _hspi);
void SST25_TxRxCpltCallback(SPI_HandleTypeDef * _hspi);
