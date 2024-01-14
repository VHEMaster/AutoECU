 /*
 * config.c
 *
 *  Created on: 2 февр. 2021 г.
 *      Author: VHEMaster
 */

#include <string.h>
#include "defines.h"
#include "flash.h"
#include "structs.h"
#include "sst25vf032b.h"
#include "crc.h"

#define BACKUP_VERSION 0x00000025
#define BACKUP_ADDR BKPSRAM_BASE
#define BACKUP_SIZE 4096
#define BACKUP_REGION_SIZE (BACKUP_SIZE / 2)

static uint8_t const *backup_pointer = (uint8_t *)BACKUP_ADDR;
static uint8_t backup_buffer[BACKUP_SIZE / 2] ALIGNED(32);

#define FLASH_VERSION 0x00000020
#define REGION_SIZE 0x200000
#define PAGE_SIZE SST25_32KSIZE
#define PAGES_COUNT (REGION_SIZE / PAGE_SIZE)

static uint8_t page_buffer[PAGE_SIZE] ALIGNED(32) BUFFER_DMA;
static const uint32_t flash_addresses[2] = {0, REGION_SIZE};

static volatile uint16_t save_sectors = 0;
static volatile uint8_t save_page = 0;
static volatile uint32_t save_size_write = 0;
static volatile uint8_t save_active = 0;
static volatile uint8_t save_region = 0;
static volatile uint16_t save_crc16 = 0;

int8_t flash_page_load(void *buffer, uint32_t size, uint8_t page)
{
  if(size >= PAGE_SIZE - 6 || page >= PAGES_COUNT)
    return -1;

  for(int region = 0; region < 2; region++)
  {
    SST25_ReadLock(flash_addresses[region] + page * PAGE_SIZE, size + 6, page_buffer);

    uint16_t crc16 = CRC16_Generate(page_buffer, size + 4);
    uint32_t version = ((uint32_t *)page_buffer)[0];
    uint16_t crc_read = page_buffer[size + 4] | (page_buffer[size + 5] << 8);
    if(crc16 == crc_read && version == FLASH_VERSION) {
      memcpy((uint8_t *)buffer, &page_buffer[4], size);
      return 1;
    }
  }

  return -1;
}

int8_t flash_page_save(const void *buffer, uint32_t size, uint8_t page)
{
  static uint8_t state = 0;
  uint32_t size_write;
  uint16_t crc16 = 0;
  uint16_t crc16_check;

  if(size >= PAGE_SIZE - 6 || page >= PAGES_COUNT)
    return -1;

  switch(state) {
    case 0 :
      size_write = size;
      ((uint32_t *)&page_buffer[0])[0] = FLASH_VERSION;
      memcpy(&page_buffer[4], (uint8_t *)buffer, size_write);
      size_write += 4;
      crc16 = CRC16_Generate(page_buffer, size_write);
      page_buffer[size_write++] = crc16 & 0xFF;
      page_buffer[size_write++] = crc16 >> 8;

      save_region = 0;
      save_sectors = (size_write / SST25_32KSIZE) + ((size_write % SST25_32KSIZE) > 0);
      save_size_write = size_write;
      save_page = page;
      save_crc16 = crc16;

      save_active = 1;

      state++;
      break;
    case 1:
      if(save_active == 0) {
        crc16 = page_buffer[save_size_write - 2] | (page_buffer[save_size_write - 1] << 8);
        crc16_check = CRC16_Generate(page_buffer, save_size_write - 2);
        if(crc16_check == save_crc16 && crc16 == save_crc16) {
          if(++save_region < 2) {
            save_active = 1;
          } else {
            state = 0;
            return 1;
          }
        } else {
          state = 0;
          return -1;
        }
      }
      break;
    default:
      state = 0;
      break;
  }

  return 0;
}

void flash_fast_loop(void)
{
  static uint16_t erased = 0;
  static uint8_t state = 0;
  static uint32_t version_buffer = 0;
  static uint16_t crc16_buffer = 0;
  uint8_t spistatus = 0;

  if(save_active) {
    switch (state) {
      case 0:
        erased = 0;
        state++;
        break;
      case 1:
        //TODO: check if it is really enough to check if the page is equal
        spistatus = SST25_Read(flash_addresses[save_region] + save_page * PAGE_SIZE, 4, (uint8_t *)&version_buffer);
        if(spistatus) {
          if(version_buffer == FLASH_VERSION) {
            state++;
          } else {
            state = 3;
          }
        }
        break;
      case 2:
        //TODO: check if it is really enough to check if the page is equal
        spistatus = SST25_Read(flash_addresses[save_region] + save_page * PAGE_SIZE + save_size_write - 2, 2, (uint8_t *)&crc16_buffer);
        if(spistatus) {
          if(crc16_buffer == save_crc16) {
            save_active = 0;
            state = 0;
          } else {
            state = 3;
          }
        }
        break;
      case 3:
        if(erased < save_sectors) {
          spistatus = SST25_Erase32KBlock(erased * SST25_32KSIZE + flash_addresses[save_region] + save_page * PAGE_SIZE);
          if(spistatus) {
            erased++;
          }
        } else {
          state++;
        }
        break;
      case 4:
        spistatus = SST25_Write(flash_addresses[save_region] + save_page * PAGE_SIZE, save_size_write, page_buffer);
        if(spistatus) {
          state++;
        }
        break;
      case 5:
        spistatus = SST25_Read(flash_addresses[save_region] + save_page * PAGE_SIZE, save_size_write, page_buffer);
        if(spistatus) {
          save_active = 0;
          state = 0;
        }
        break;
      default:
        state = 0;
        break;
    }
  }
}

int8_t flash_bkpsram_load(void *buffer, uint32_t size, uint32_t offset)
{
  uint16_t crc16;
  uint32_t version;
  uint16_t crc_read;
  if(size + offset >= BACKUP_REGION_SIZE - 6)
    return -1;

  for(int region = 0; region < 2; region++)
  {
    memcpy(backup_buffer, (uint8_t *)backup_pointer + BACKUP_REGION_SIZE * region + offset, size + 6);

    crc16 = CRC16_Generate(backup_buffer, size + 4);
    version = ((uint32_t *)&backup_buffer[0])[0];
    crc_read = backup_buffer[size + 4] | (backup_buffer[size + 5] << 8);
    if(crc16 == crc_read && version == BACKUP_VERSION) {
      memcpy((uint8_t *)buffer, &backup_buffer[4], size);
      return 1;
    }
  }

  return -1;
}

int8_t flash_bkpsram_save(const void *buffer, uint32_t size, uint32_t offset)
{
  uint16_t crc16;
  if(size + offset >= BACKUP_REGION_SIZE - 6)
    return -1;

  ((uint32_t *)&backup_buffer[0])[0] = BACKUP_VERSION;
  memcpy(&backup_buffer[4], (uint8_t *)buffer, size);
  size += 4;
  crc16 = CRC16_Generate(backup_buffer, size);
  backup_buffer[size++] = crc16 & 0xFF;
  backup_buffer[size++] = crc16 >> 8;

  for(int region = 0; region < 2; region++) {
    memcpy((uint8_t *)&backup_pointer[BACKUP_REGION_SIZE * region + offset], backup_buffer, size);
  }

  return 1;
}

HAL_StatusTypeDef flash_checkchip(void)
{
  return SST25_CheckChip();
}

