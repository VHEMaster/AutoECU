/*
 * ecu.c
 *
 *  Created on: Mar 4, 2022
 *      Author: VHEMaster
 */

#include "main.h"
#include "delay.h"
#include "ecu.h"
#include "packets.h"
#include "flash.h"
#include "xCommand.h"
#include "csps.h"
#include "misc.h"
#include "adc.h"
#include "speed.h"
#include "sensors.h"
#include "outputs.h"
#include "injector.h"
#include "sst25vf032b.h"
#include "config.h"

#include <string.h>
#include "arm_math.h"

typedef struct {
    union {
        struct {
            HAL_StatusTypeDef Load : 2;
            HAL_StatusTypeDef Save : 2;
            HAL_StatusTypeDef Init : 2;
        }Struct;
        uint8_t Byte;
    }Flash;
}sStatus;

static sEcuTable gEcuTable[TABLE_SETUPS_MAX];
static sEcuParams gEcuParams;
static sEcuCorrections gEcuCorrectives;
static sStatus gStatus = {{{0}}};

static void ecu_config_init(void)
{
  int8_t status;

  memset(gEcuTable, 0, sizeof(gEcuTable));
  memset(&gEcuParams, 0, sizeof(gEcuParams));
  memset(&gEcuCorrectives, 0, sizeof(gEcuCorrectives));
  memset(&gStatus, 0, sizeof(gStatus));

  gStatus.Flash.Struct.Init = config_init();

  while(!(status = config_load_params(&gEcuParams))) {}
  if(status < 0) {
    gStatus.Flash.Struct.Load = HAL_ERROR;
    config_default_params(&gEcuParams);
    while(!(status = config_save_params(&gEcuParams))) {}
    if(status < 0) {
        gStatus.Flash.Struct.Save = HAL_ERROR;
    }
  }

  for(int i = 0; i < TABLE_SETUPS_MAX; i++) {
    while(!(status = config_load_table(&gEcuTable[i], i))) {}
    if(status < 0) {
      gStatus.Flash.Struct.Load = HAL_ERROR;
      config_default_table(&gEcuTable[i], i);
      while(!(status = config_save_table(&gEcuTable[i], i))) {}
      if(status < 0) {
          gStatus.Flash.Struct.Save = HAL_ERROR;
      }
    }
  }

  while(!(status = config_load_correctives(&gEcuCorrectives))) {}
  if(status < 0) {
    gStatus.Flash.Struct.Load = HAL_ERROR;
    config_default_correctives(&gEcuCorrectives);
    while(!(status = config_save_correctives(&gEcuCorrectives))) {}
    if(status < 0) {
        gStatus.Flash.Struct.Save = HAL_ERROR;
    }
  }
}

static float ecu_get_air_destiny(float pressure, float temperature)
{
  const float M = 29.0f;
  const float R = 8314462.61815324f;
  float g_cc3 = (pressure * M) / (R * (temperature + 273.0f));
  return g_cc3;
}

void ecu_init(void)
{

  ecu_config_init();

}

void ecu_loop(void)
{

}

void ecu_irq_slow_loop(void)
{

}

void ecu_irq_fast_loop(void)
{

}

void ecu_parse_command(eTransChannels xChaSrc, uint8_t * msgBuf, uint32_t length)
{

}
