/*
 * config.h
 *
 *  Created on: 7 мар. 2022 г.
 *      Author: VHEMaster
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "main.h"
#include "structs.h"

HAL_StatusTypeDef config_init(void);

int8_t config_load_table(sEcuTable *table, uint8_t number);
int8_t config_save_table(const sEcuTable *table, uint8_t number);
void config_default_table(sEcuTable *table, uint8_t number);

int8_t config_load_params(sEcuParams *params);
int8_t config_save_params(const sEcuParams *params);
void config_default_params(sEcuParams *table);

int8_t config_load_corrections(sEcuCorrections *table);
int8_t config_save_corrections(const sEcuCorrections *table);
void config_default_corrections(sEcuCorrections *table);

void config_default_critical_backup(sEcuCriticalBackup *table);
int8_t config_load_critical_backup(sEcuCriticalBackup *table);
int8_t config_save_critical_backup(const sEcuCriticalBackup *table);

#endif /* INC_CONFIG_H_ */
