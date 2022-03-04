/*
 * mis.h
 *
 *  Created on: Mar 4, 2022
 *      Author: VHEMaster
 */

#ifndef INC_MIS_H_
#define INC_MIS_H_

#include "main.h"

#define O2_OK                0
#define O2_E_NOPOWER         1
#define O2_E_SHORTCIRCUITGND 2
#define O2_E_SHORTCIRCUITBAT 3

typedef struct {
    uint8_t Working;
    uint8_t Valid;
    float FuelRatio;
    union {
        uint8_t Byte;
        struct {
            uint8_t VM : 2;
            uint8_t UN : 2;
            uint8_t IAIP : 2;
            uint8_t DIAHGD : 2;
        } Fields;
    }Diag;
}sO2Status;

void Misc_ErrorCallback(SPI_HandleTypeDef * _hspi);
void Misc_TxCpltCallback(SPI_HandleTypeDef * _hspi);
void Misc_RxCpltCallback(SPI_HandleTypeDef * _hspi);
void Misc_TxRxCpltCallback(SPI_HandleTypeDef * _hspi);
HAL_StatusTypeDef Misc_Init(SPI_HandleTypeDef * _hspi);

HAL_StatusTypeDef Misc_O2_Init(uint32_t pwm_period, volatile uint32_t *pwm_duty);
sO2Status Misc_O2_GetStatus(void);
void Misc_Loop(void);

#endif /* INC_MIS_H_ */
