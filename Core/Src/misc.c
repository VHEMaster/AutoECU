/*
 * misc.c
 *
 *  Created on: Mar 4, 2022
 *      Author: VHEMaster
 */
/*
 * Misc.c
 *
 *  Created on: Mar 4, 2022
 *      Author: VHEMaster
 */

#include "arm_math.h"
#include "misc.h"
#include "adc.h"
#include "delay.h"
#include "csps.h"

static const float o2_afr_table[548] = {
  00.00f, 00.04f, 00.08f, 00.13f, 00.17f, 00.21f, 00.25f, 00.30f, 00.34f, 00.38f, 00.42f, 00.47f, 00.51f, 00.55f, 00.59f, 00.64f, 00.68f, 00.72f, 00.76f, 00.81f,
  00.85f, 00.89f, 00.93f, 00.98f, 01.02f, 01.06f, 01.10f, 01.15f, 01.19f, 01.23f, 01.27f, 01.31f, 01.36f, 01.40f, 01.44f, 01.48f, 01.53f, 01.57f, 01.61f, 01.65f,
  01.70f, 01.74f, 01.78f, 01.82f, 01.86f, 01.91f, 01.95f, 01.99f, 02.03f, 02.08f, 02.12f, 02.16f, 02.20f, 02.24f, 02.29f, 02.33f, 02.37f, 02.41f, 02.45f, 02.50f,
  02.54f, 02.58f, 02.62f, 02.66f, 02.71f, 02.75f, 02.79f, 02.83f, 02.87f, 02.92f, 02.96f, 03.00f, 03.04f, 03.08f, 03.13f, 03.17f, 03.21f, 03.25f, 03.29f, 03.33f,
  03.38f, 03.42f, 03.46f, 03.50f, 03.54f, 03.58f, 03.63f, 03.67f, 03.71f, 03.75f, 03.79f, 03.83f, 03.88f, 03.92f, 03.96f, 04.00f, 04.04f, 04.08f, 04.12f, 04.17f,
  04.21f, 04.25f, 04.29f, 04.33f, 04.37f, 04.41f, 04.45f, 04.50f, 04.54f, 04.58f, 04.62f, 04.66f, 04.70f, 04.74f, 04.78f, 04.82f, 04.86f, 04.91f, 04.95f, 04.99f,
  05.03f, 05.07f, 05.11f, 05.15f, 05.19f, 05.23f, 05.27f, 05.31f, 05.35f, 05.39f, 05.44f, 05.48f, 05.52f, 05.56f, 05.60f, 05.64f, 05.68f, 05.72f, 05.76f, 05.80f,
  05.84f, 05.88f, 05.92f, 05.96f, 06.00f, 06.04f, 06.08f, 06.12f, 06.16f, 06.20f, 06.24f, 06.28f, 06.32f, 06.36f, 06.40f, 06.44f, 06.48f, 06.52f, 06.56f, 06.60f,
  06.64f, 06.68f, 06.72f, 06.76f, 06.80f, 06.84f, 06.88f, 06.92f, 06.96f, 07.00f, 07.03f, 07.07f, 07.11f, 07.15f, 07.19f, 07.23f, 07.27f, 07.31f, 07.35f, 07.39f,
  07.43f, 07.47f, 07.51f, 07.55f, 07.59f, 07.62f, 07.66f, 07.70f, 07.74f, 07.78f, 07.82f, 07.86f, 07.90f, 07.94f, 07.98f, 08.02f, 08.06f, 08.09f, 08.13f, 08.17f,
  08.21f, 08.25f, 08.29f, 08.33f, 08.37f, 08.41f, 08.45f, 08.49f, 08.52f, 08.56f, 08.60f, 08.64f, 08.68f, 08.72f, 08.76f, 08.80f, 08.84f, 08.88f, 08.91f, 08.95f,
  08.99f, 09.03f, 09.07f, 09.11f, 09.15f, 09.19f, 09.23f, 09.26f, 09.30f, 09.34f, 09.38f, 09.42f, 09.46f, 09.50f, 09.54f, 09.57f, 09.61f, 09.65f, 09.69f, 09.73f,
  09.77f, 09.81f, 09.85f, 09.89f, 09.92f, 09.96f, 10.00f, 10.04f, 10.08f, 10.12f, 10.16f, 10.19f, 10.23f, 10.27f, 10.31f, 10.35f, 10.39f, 10.43f, 10.47f, 10.50f,
  10.54f, 10.58f, 10.62f, 10.66f, 10.70f, 10.73f, 10.77f, 10.81f, 10.85f, 10.89f, 10.93f, 10.97f, 11.00f, 11.04f, 11.08f, 11.12f, 11.16f, 11.20f, 11.23f, 11.27f,
  11.31f, 11.35f, 11.39f, 11.43f, 11.46f, 11.50f, 11.54f, 11.58f, 11.62f, 11.66f, 11.69f, 11.73f, 11.77f, 11.81f, 11.85f, 11.89f, 11.92f, 11.96f, 12.00f, 12.04f,
  12.08f, 12.11f, 12.15f, 12.19f, 12.23f, 12.27f, 12.30f, 12.34f, 12.38f, 12.42f, 12.46f, 12.49f, 12.53f, 12.57f, 12.61f, 12.65f, 12.68f, 12.72f, 12.76f, 12.80f,
  12.84f, 12.87f, 12.91f, 12.95f, 12.99f, 13.03f, 13.06f, 13.10f, 13.14f, 13.18f, 13.21f, 13.25f, 13.29f, 13.33f, 13.36f, 13.40f, 13.44f, 13.48f, 13.51f, 13.55f,
  13.59f, 13.63f, 13.67f, 13.70f, 13.74f, 13.78f, 13.82f, 13.85f, 13.89f, 13.93f, 13.96f, 14.00f, 14.04f, 14.08f, 14.11f, 14.15f, 14.19f, 14.23f, 14.26f, 14.30f,
  14.34f, 14.38f, 14.41f, 14.45f, 14.49f, 14.52f, 14.56f, 14.60f, 14.64f, 14.67f, 14.71f, 14.75f, 14.78f, 14.82f, 14.86f, 14.90f, 14.93f, 14.97f, 15.01f, 15.04f,
  15.08f, 15.12f, 15.15f, 15.19f, 15.23f, 15.26f, 15.30f, 15.34f, 15.37f, 15.41f, 15.45f, 15.48f, 15.52f, 15.56f, 15.59f, 15.63f, 15.67f, 15.70f, 15.74f, 15.78f,
  15.81f, 15.85f, 15.89f, 15.92f, 15.96f, 16.00f, 16.03f, 16.07f, 16.11f, 16.14f, 16.18f, 16.22f, 16.25f, 16.29f, 16.32f, 16.36f, 16.40f, 16.43f, 16.47f, 16.51f,
  16.54f, 16.58f, 16.61f, 16.65f, 16.69f, 16.72f, 16.76f, 16.79f, 16.83f, 16.87f, 16.90f, 16.94f, 16.97f, 17.01f, 17.05f, 17.08f, 17.12f, 17.15f, 17.19f, 17.22f,
  17.26f, 17.30f, 17.33f, 17.37f, 17.40f, 17.44f, 17.47f, 17.51f, 17.55f, 17.58f, 17.62f, 17.65f, 17.69f, 17.72f, 17.76f, 17.79f, 17.83f, 17.86f, 17.90f, 17.94f,
  17.97f, 18.01f, 18.04f, 18.08f, 18.11f, 18.15f, 18.18f, 18.22f, 18.25f, 18.29f, 18.32f, 18.36f, 18.39f, 18.43f, 18.46f, 18.50f, 18.53f, 18.57f, 18.60f, 18.64f,
  18.67f, 18.71f, 18.74f, 18.78f, 18.81f, 18.85f, 18.88f, 18.92f, 18.95f, 18.98f, 19.02f, 19.05f, 19.09f, 19.12f, 19.16f, 19.19f, 19.23f, 19.26f, 19.30f, 19.33f,
  19.36f, 19.40f, 19.43f, 19.47f, 19.50f, 19.54f, 19.57f, 19.60f, 19.64f, 19.67f, 19.71f, 19.74f, 19.77f, 19.81f, 19.84f, 19.88f, 19.91f, 19.94f, 19.98f, 20.01f,
  20.05f, 20.08f, 20.11f, 20.15f, 20.18f, 20.22f, 20.25f, 20.28f, 20.32f, 20.35f, 20.38f, 20.42f, 20.45f, 20.48f, 20.52f, 20.55f, 20.58f, 20.62f, 20.65f, 20.68f,
  20.72f, 20.75f, 20.78f, 20.82f, 20.85f, 20.88f, 20.92f, 20.95f
};

#define KNOCK_HOLD() HAL_GPIO_WritePin(KNOCK_INT_GPIO_Port, KNOCK_INT_Pin, GPIO_PIN_RESET)
#define KNOCK_INTEGRATE() HAL_GPIO_WritePin(KNOCK_INT_GPIO_Port, KNOCK_INT_Pin, GPIO_PIN_SET)

#define SPI_NSS_INJ_ON() HAL_GPIO_WritePin(SPI4_NSS_INJ_GPIO_Port, SPI4_NSS_INJ_Pin, GPIO_PIN_RESET)
#define SPI_NSS_INJ_OFF() HAL_GPIO_WritePin(SPI4_NSS_INJ_GPIO_Port, SPI4_NSS_INJ_Pin, GPIO_PIN_SET)
#define SPI_NSS_OUTS1_ON() HAL_GPIO_WritePin(SPI4_NSS_OUTS1_GPIO_Port, SPI4_NSS_OUTS1_Pin, GPIO_PIN_RESET)
#define SPI_NSS_OUTS1_OFF() HAL_GPIO_WritePin(SPI4_NSS_OUTS1_GPIO_Port, SPI4_NSS_OUTS1_Pin, GPIO_PIN_SET)
#define SPI_NSS_OUTS2_ON() HAL_GPIO_WritePin(SPI4_NSS_OUTS2_GPIO_Port, SPI4_NSS_OUTS2_Pin, GPIO_PIN_RESET)
#define SPI_NSS_OUTS2_OFF() HAL_GPIO_WritePin(SPI4_NSS_OUTS2_GPIO_Port, SPI4_NSS_OUTS2_Pin, GPIO_PIN_SET)
#define SPI_NSS_O2_ON() HAL_GPIO_WritePin(SPI4_NSS_O2_GPIO_Port, SPI4_NSS_O2_Pin, GPIO_PIN_RESET)
#define SPI_NSS_O2_OFF() HAL_GPIO_WritePin(SPI4_NSS_O2_GPIO_Port, SPI4_NSS_O2_Pin, GPIO_PIN_SET)
#define SPI_NSS_KNOCK_ON() HAL_GPIO_WritePin(SPI4_NSS_KNOCK_GPIO_Port, SPI4_NSS_KNOCK_Pin, GPIO_PIN_RESET)
#define SPI_NSS_KNOCK_OFF() HAL_GPIO_WritePin(SPI4_NSS_KNOCK_GPIO_Port, SPI4_NSS_KNOCK_Pin, GPIO_PIN_SET)

static SPI_HandleTypeDef * hspi;

static uint8_t tx[32] __attribute__((aligned(32)));
static uint8_t rx[32] __attribute__((aligned(32)));

static volatile uint8_t semTx = 0;
static volatile uint8_t semRx = 0;

static uint8_t O2Device = 0;
static uint8_t O2Version = 0;
static uint32_t O2PwmPeriod = 255;
static volatile uint32_t *O2PwmDuty = NULL;
static volatile float KnockRawValue = 0.0f;
static volatile float KnockValue = 0.0f;
static HAL_StatusTypeDef KnockInitialStatus = HAL_OK;
static volatile HAL_StatusTypeDef KnockStatus = HAL_OK;

static uint8_t OutputsDiagBytes[MiscDiagChCount] = {0};
static uint8_t OutputsDiagnosticStored[MiscDiagChCount] = {0};
static HAL_StatusTypeDef OutputsAvailability[MiscDiagChCount] = {HAL_OK, HAL_OK, HAL_OK};

static arm_pid_instance_f32 o2_pid;


static sO2Status O2Status = {0};


void Misc_ErrorCallback(SPI_HandleTypeDef * _hspi)
{
  if(_hspi == hspi) {

  }
}

void Misc_TxCpltCallback(SPI_HandleTypeDef * _hspi)
{
  if(_hspi == hspi) {
    semTx = 1;
  }
}

void Misc_RxCpltCallback(SPI_HandleTypeDef * _hspi)
{
  if(_hspi == hspi) {
    semRx = 1;
  }
}

void Misc_TxRxCpltCallback(SPI_HandleTypeDef * _hspi)
{
  if(_hspi == hspi) {
    semTx = 1;
    semRx = 1;
  }
}

static uint8_t waitTxCplt(void)
{
  if(semTx) {
    semTx = 0;
    return 1;
  }
  return 0;
}

static uint8_t waitRxCplt(void)
{
  if(semRx) {
    semRx = 0;
    return 1;
  }
  return 0;
}

static uint8_t waitTxRxCplt(void)
{
  if(semRx && semTx) {
    semRx = 0;
    semTx = 0;
    return 1;
  }
  return 0;
}

#define KNOCK_CMD_GAIN      0x80
#define KNOCK_CMD_BP_FREQ   0x00
#define KNOCK_CMD_INT_TIME  0xC0
#define KNOCK_CMD_CH_SEL    0xE0
#define KNOCK_CMD_SO_MODE   0x40

static int8_t Knock_Cmd(uint8_t cmd, uint8_t *read)
{
  static uint8_t state = 0;
  switch(state) {
    case 0:
      tx[0] = cmd;
      SPI_NSS_KNOCK_ON();
      HAL_SPI_TransmitReceive_IT(hspi, tx, rx, 1);
      state++;

      break;
    case 1:
      if(waitTxRxCplt())
      {
        SPI_NSS_KNOCK_OFF();
        if(read)
          *read = rx[0];
        state = 0;
        return 1;
      }
      break;
    default:
      state = 0;
      break;
  }

  return 0;
}

#define O2_IDENT_DEVICE_CJ125 0x60
#define O2_IDENT_MASK_DEVICE 0xF8
#define O2_IDENT_MASK_VERSION 0x07
#define O2_IDENT_REG_RD 0x48
#define O2_DIAG_REG_RD 0x78
#define O2_INIT_REG1_RD 0x6C
#define O2_INIT_REG1_WR 0x56
#define O2_INIT_REG2_RD 0x7E
#define O2_INIT_REG2_WR 0x5A

#define O2_REG1_CALIBR 0x9D
#define O2_REG1_NORMAL 0x89

static int8_t O2_Write(uint8_t cmd, uint8_t data)
{
  static uint8_t state = 0;
  switch(state) {
    case 0:
      tx[0] = cmd;
      tx[1] = data;
      SPI_NSS_O2_ON();
      HAL_SPI_TransmitReceive_IT(hspi, tx, rx, 2);
      state++;

      break;
    case 1:
      if(waitTxRxCplt())
      {
        SPI_NSS_O2_OFF();
        state = 0;
        if((rx[0] & 0x38) != 0x28 || rx[1] != 0x00) {
          return -1;
        }

        return 1;
      }
      break;
    default:
      state = 0;
      break;
  }

  return 0;
}

static int8_t O2_Read(uint8_t cmd, uint8_t *data)
{
  static uint8_t state = 0;
  *data = 0;
  switch(state) {
    case 0:
      tx[0] = cmd;
      tx[1] = 0;
      SPI_NSS_O2_ON();
      HAL_SPI_TransmitReceive_IT(hspi, tx, rx, 2);
      state++;

      break;
    case 1:
      if(waitTxRxCplt())
      {
        SPI_NSS_O2_OFF();
        state = 0;
        if((rx[0] & 0x38) != 0x28) {
          return -1;
        }

        *data = rx[1];

        return 1;
      }
      break;
    default:
      state = 0;
      break;
  }

  return 0;
}

static void O2_SetHeaterDutyCycle(float dutycycle)
{
  if(dutycycle > 1.0f)
    dutycycle = 1.0f;
  *O2PwmDuty = (float)O2PwmPeriod * dutycycle;
}

static void O2_SetHeaterVoltage(float voltage)
{
  float dutycycle;
  float power = ADC_GetVoltage(AdcChPowerVoltage);

  if(voltage > power)
    voltage = power;

  dutycycle = voltage/power;
  O2_SetHeaterDutyCycle(dutycycle);

}

static float O2_GetFuelRatio(void)
{
  float lambda = 14.7;
  float voltage = ADC_GetVoltage(AdcChO2UA);
  uint32_t array_pos = voltage / 5.0f * 1024.0f;

  if(array_pos < 307)
    array_pos = 307;
  else if(array_pos > 854)
    array_pos = 854;

  lambda = o2_afr_table[array_pos - 307];

  return lambda;
}

static int8_t O2_GetDevice(uint8_t *device)
{
  return O2_Read(O2_IDENT_REG_RD, device);
}

static int8_t O2_GetDiag(uint8_t *diag)
{
  return O2_Read(O2_DIAG_REG_RD, diag);
}

static int8_t O2_Calibrate(void)
{
  return O2_Write(O2_INIT_REG1_WR, O2_REG1_CALIBR);
}

static int8_t O2_Enable(void)
{
  return O2_Write(O2_INIT_REG1_WR, O2_REG1_NORMAL);
}

static void O2_CriticalLoop(void)
{
  O2Status.FuelRatio = O2_GetFuelRatio();
}

static int8_t O2_Loop(void)
{
  static uint8_t is_engine_runned = 0;
  static uint8_t state = 0;
  static uint32_t last_spi_check = 0;
  static uint32_t calibrate_timestamp = 0;
  static float o2heater = 0.0f;
  static float o2reference = 0.0f;
  float voltage;
  uint32_t now = Delay_Tick;
  uint8_t device,diag;
  int8_t status;

  if(is_engine_runned) {
    if(!csps_isrotates())
      is_engine_runned = 0;
  } else {
    if(csps_isrotates() && csps_getrpm() > 800)
      is_engine_runned = 1;
  }

  float engine_temperature = 50.0f;

  switch(state) {
    case 0 :
      if(DelayDiff(now, last_spi_check) > 500000) {
        last_spi_check = now;
        state = 1;
      } else if(O2Status.Valid) {
        state = 8;
      }
      else if(is_engine_runned) {
        state = 3;
      }
      else {
        state = 0;
      }
      break;
    case 1 :
      status = O2_GetDevice(&device);
      if(status == -1) {
        O2Status.Available = 0;
        O2Status.Working = 0;
        O2Status.Valid = 0;
        O2_SetHeaterVoltage(0);
        state = 0;
        return -1;
      } else if(status == 1) {
        O2Status.Available = 1;
        state++;
      }
      break;
    case 2 :
      status = O2_GetDiag(&diag);
      if(status) {
        O2Status.Diag.Byte = diag;
        if(is_engine_runned) {
          O2Status.Working = 1;
          if(O2Status.Valid) {
            state = 8;
          }
          else state++;
        }
        else state = 0;
      }
      break;
    case 3 :
      status = O2_Calibrate();
      if(status) {
        state++;
      }
      break;
    case 4 :
      if(engine_temperature > 30.0f) {
        calibrate_timestamp = now;
        state++;
      }
      break;
    case 5 :
      if(!is_engine_runned) {
        O2_SetHeaterVoltage(0.0f);
        O2Status.Valid = 0;
        state = 0;
        break;
      }
      O2_SetHeaterVoltage(1.5f);
      if(DelayDiff(now, calibrate_timestamp) > 10000) {
        o2heater = 8.5f;
        O2_SetHeaterVoltage(o2heater);
        calibrate_timestamp = now;
        state++;
      }
      break;
    case 6 :
      if(!is_engine_runned) {
        O2_SetHeaterVoltage(0.0f);
        O2Status.Valid = 0;
        state = 0;
        break;
      }
      voltage = ADC_GetVoltage(AdcChPowerVoltage);
      if(o2heater >= voltage) {
        O2_SetHeaterVoltage(0);
        state++;
      }
      if(DelayDiff(now, calibrate_timestamp) > 100000) {
        o2heater += 0.4f / 10.0f;
        O2_SetHeaterVoltage(o2heater);
        calibrate_timestamp = now;
      }
      break;
    case 7 :
      o2reference = ADC_GetVoltage(AdcChO2UR);
      status = O2_Enable();
      if(status) {
        O2Status.Valid = 1;
        calibrate_timestamp = now;
        state++;
      }
      break;
    case 8 :
      voltage = ADC_GetVoltage(AdcChO2UR);
      o2heater = arm_pid_f32(&o2_pid, o2reference - voltage);
      O2_SetHeaterVoltage(o2heater);
      state = 0;
      break;
    default:
      state = 0;
      break;
  }
  return 0;
}

sO2Status Misc_O2_GetStatus(void)
{
  return O2Status;
}

static int8_t Outs_Loop(void)
{
  static uint8_t state = 0;
  static uint8_t failure_stored = 0;
  GPIO_PinState pin;
  eMiscDiagChannels channel = 0;

  do {
    switch(state) {
      case 0:
        SPI_NSS_INJ_OFF(); SPI_NSS_OUTS1_OFF(); SPI_NSS_OUTS2_OFF();

        switch(channel) {
          case MiscDiagChInjectors: SPI_NSS_INJ_ON(); break;
          case MiscDiagChOutputs1: SPI_NSS_OUTS1_ON(); break;
          case MiscDiagChOutputs2: SPI_NSS_OUTS2_ON(); break;
          default: channel = 0; continue;
        }
        state++;
        break;
      case 1:
        pin = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5); //SPI4_MISO
        failure_stored = pin != GPIO_PIN_RESET;
        HAL_SPI_Receive_IT(hspi, rx, 1);
        state++;
        break;
      case 2:
        if(waitRxCplt()) {
          SPI_NSS_INJ_OFF(); SPI_NSS_OUTS1_OFF(); SPI_NSS_OUTS2_OFF();

          if(OutputsDiagnosticStored[channel] == 0 || failure_stored == 1 || rx[0] != 0xFF) {
            OutputsDiagnosticStored[channel] = 1;
            OutputsDiagBytes[channel] = rx[0];
            if((failure_stored == 1 && rx[0] == 0xFF) || (failure_stored == 0 && rx[0] != 0xFF))
              OutputsAvailability[channel] = HAL_ERROR;
            else OutputsAvailability[channel] = HAL_OK;
          }

          state = 0;
          if(++channel >= MiscDiagChCount) {
            channel = 0;
            return 1;
          }
          else continue;
        }
        break;
      default:
        state = 0;
        break;
    }
  } while(0);

  return 0;
}

static int8_t Knock_Loop(void)
{
  static uint8_t state = 0;

  if(KnockInitialStatus != HAL_OK)
    return 1;

  switch(state) {
    case 0:
      //TODO: in case of changed parameters, perform the setup on the fly
      KnockStatus = HAL_OK;
      return 1;
      break;
    default:
      state = 0;
      break;
  }

  return 0;
}

static void Knock_CriticalLoop(void)
{
  static uint8_t state = 0;
  static uint32_t knock_prev = 0;
  uint32_t now = Delay_Tick;
  uint32_t diff;
  float voltage,corrected,rpm;
  float koff;
  uint8_t rotates;

  if(KnockInitialStatus != HAL_OK)
    return;

  switch(state) {
    case 0: //First ms
      voltage = ADC_GetVoltage(AdcChKnock);
      KNOCK_INTEGRATE();
      state++;

      rpm = csps_getrpm();
      rotates = csps_isfound();
      diff = DelayDiff(now, knock_prev);
      knock_prev = now;

      if(rotates && rpm > 60.0f && diff > 0) {
        corrected = voltage / rpm * 60.0f;
        koff = 1.0f / (60000000.0f / rpm / diff);
      }
      else koff = 0.025f;

      KnockRawValue = KnockRawValue * (1.0f - koff) + voltage * koff;
      KnockValue = KnockValue * (1.0f - koff) + corrected * koff;
      KnockStatus = HAL_OK;

      break;
    case 1: //2-5th
    case 2:
    case 3:
    case 4:
      state++;
      break;
    case 5:
      //Give to ADC the time to measure
      KNOCK_HOLD();
      state = 0;
      break;
    default:
      break;
  }
}

static volatile uint8_t IdleValvePositionCurrent = 0;
static volatile uint8_t IdleValvePositionTarget = 0;
static volatile HAL_StatusTypeDef IdleValvePositionStatus = HAL_OK;

inline uint8_t Misc_GetIdleValvePosition(void)
{
  return IdleValvePositionCurrent;
}

inline void Misc_SetIdleValvePosition(uint8_t position)
{
  IdleValvePositionTarget = position;
}

inline HAL_StatusTypeDef Misc_GetIdleValveStatus(void)
{
  return IdleValvePositionStatus;
}

static void IdleValve_CriticalLoop(void)
{
  GPIO_PinState idle_valve_state_pin = HAL_GPIO_ReadPin(STEP_ERR_GPIO_Port, STEP_ERR_Pin);

  IdleValvePositionStatus = idle_valve_state_pin == GPIO_PIN_RESET ? HAL_ERROR : HAL_OK;
}

static uint32_t StepPhase = 0;

//Works for STEP_PH_GPIO_Port == STEP_PH2_GPIO_Port, STEP_PH1_Pin < STEP_PH2_Pin and close together
#define STEP_APPEND() { STEP_PH1_GPIO_Port->BSRR = StepPhase | (StepPhase ^ (STEP_PH1_Pin | STEP_PH2_Pin)) << 16; }
#define STEP_INCREMENT() { StepPhase = (StepPhase + STEP_PH1_Pin) & (STEP_PH1_Pin | STEP_PH2_Pin); }
#define STEP_DECREMENT() { StepPhase = (StepPhase - STEP_PH1_Pin) & (STEP_PH1_Pin | STEP_PH2_Pin); }

//Works for STEP_I0_GPIO_Port == STEP_I1_GPIO_Port
#define STEP_IDLE() { STEP_I0_GPIO_Port->BSRR = (STEP_I0_Pin | STEP_I1_Pin) << 16; }
#define STEP_HOLD() { STEP_I0_GPIO_Port->BSRR = STEP_I0_Pin | (STEP_I1_Pin << 16); }
#define STEP_NORMAL() { STEP_I0_GPIO_Port->BSRR = STEP_I1_Pin | (STEP_I0_Pin << 16); }
#define STEP_ACCELERATE() { STEP_I0_GPIO_Port->BSRR = STEP_I1_Pin | STEP_I0_Pin; }

#define STEP_MAX_SPEED_FROM_START_TO_END 0.2f
#define STEP_MAX_FREQ ((uint32_t)(STEP_MAX_SPEED_FROM_START_TO_END * 1000000.0f / 256.0f))

static void IdleValve_FastLoop(void)
{
  static uint32_t last_tick = 0;
  static uint8_t is_hold = 0;
  static uint32_t last_move = 0;
  uint32_t now = Delay_Tick;
  uint8_t current = IdleValvePositionCurrent;
  uint8_t target = IdleValvePositionTarget;

  if(DelayDiff(now, last_tick) > STEP_MAX_FREQ) {
    last_tick = now;
    if(current != target) {
      STEP_ACCELERATE();
      if(current < target) {
        IdleValvePositionCurrent++;
        STEP_INCREMENT();
      } else if(current > target) {
        IdleValvePositionCurrent--;
        STEP_DECREMENT();
      }
      last_move = now;
      is_hold = 0;
      STEP_APPEND();
    } else {
      if(is_hold) {
        STEP_HOLD();
      } else {
        if(DelayDiff(now, last_move) > 1000000) {
          STEP_HOLD();
          is_hold = 1;
        } else {
          STEP_NORMAL();
        }
      }
    }
  }
}


void Misc_Fast_Loop(void)
{

}

void Misc_Loop(void)
{
  static uint8_t work_o2 = 0;
  static uint8_t work_knock = 0;
  static uint8_t work_outs = 0;

  static uint32_t lastO2Exec = 0;
  static uint32_t lastKnockExec = 0;
  static uint32_t lastOutsExec = 0;

  uint32_t now = Delay_Tick;

  if(!work_o2 && !work_knock && !work_outs) {
    if(DelayDiff(now, lastO2Exec) >= 10000) {
      lastO2Exec = now;
      work_o2 = 1;
    }
    if(DelayDiff(now, lastKnockExec) >= 100000) {
      lastKnockExec = now;
      work_knock = 1;
    }
    if(DelayDiff(now, lastOutsExec) >= 100000) {
      lastOutsExec = now;
      work_outs = 1;
    }
  }

  O2_CriticalLoop();
  Knock_CriticalLoop();
  IdleValve_CriticalLoop();

  if(work_o2) {
    if(O2_Loop())
      work_o2 = 0;
  }
  else if(work_knock) {
    if(Knock_Loop())
      work_knock = 0;
  }
  else if(work_outs) {
    if(Outs_Loop())
      work_outs = 0;
  }
}

#define KNOCK_CMD_GAIN      0x80
#define KNOCK_CMD_BP_FREQ   0x00
#define KNOCK_CMD_INT_TIME  0xC0
#define KNOCK_CMD_CH_SEL    0xE0
#define KNOCK_CMD_SO_MODE   0x40

HAL_StatusTypeDef Mics_Knock_Init(void)
{
  const uint8_t bandpass_filter_frequency = 42; //7.27kHz for D=79mm
  const uint8_t gain_value = 14;                //1.0
  const uint8_t integrator_time_constant = 22;  //280uS
  const uint8_t oscillator_freq_select = 0;
  const uint8_t channel_select = 0;
  const uint8_t diagnostic_mode = 0;
  const uint8_t so_output_mode = 0;
  uint8_t data,read;

  KNOCK_INTEGRATE();
  DelayUs(100);
  KNOCK_HOLD();

  //TODO: perform such setup on the fly

  data = KNOCK_CMD_GAIN | (gain_value & 0x3F);
  while(!Knock_Cmd(data, &read)) {}

  data = KNOCK_CMD_BP_FREQ | (bandpass_filter_frequency & 0x3F);
  while(!Knock_Cmd(data, &read)) {}

  data = KNOCK_CMD_INT_TIME | (integrator_time_constant & 0x1F);
  while(!Knock_Cmd(data, &read)) {}

  data = KNOCK_CMD_CH_SEL | (channel_select & 0x1) | ((diagnostic_mode & 0xF) << 1);
  while(!Knock_Cmd(data, &read)) {}

  data = KNOCK_CMD_SO_MODE | ((oscillator_freq_select & 0xF) << 1) | (so_output_mode & 0x1);
  while(!Knock_Cmd(data, &read)) {}

  KnockInitialStatus = HAL_OK;
  KnockStatus = HAL_OK;

  return HAL_OK;
}

HAL_StatusTypeDef Misc_O2_Init(uint32_t pwm_period, volatile uint32_t *pwm_duty)
{
  uint8_t device;

  O2PwmPeriod = pwm_period + 1;
  O2PwmDuty = pwm_duty;

  O2_SetHeaterVoltage(0);
  O2Status.Available = 1;
  O2Status.FuelRatio = 0;
  O2Status.Valid = 0;
  O2Status.Working = 0;

  o2_pid.Kp = 0.5f;
  o2_pid.Ki = 0.1f;
  o2_pid.Kd = 0.1f;
  arm_pid_init_f32(&o2_pid, 1);

  while(!O2_GetDevice(&device)) {};

  O2Device = device & O2_IDENT_MASK_DEVICE;
  O2Version = device & O2_IDENT_MASK_VERSION;
  if(O2Device != O2_IDENT_DEVICE_CJ125) {
    O2Status.Available = 0;
    return HAL_ERROR;
  }

  O2Status.Working = 1;

  return HAL_OK;
}

HAL_StatusTypeDef Misc_Init(SPI_HandleTypeDef * _hspi)
{
  HAL_StatusTypeDef result = HAL_OK;
  SCB_CleanDCache_by_Addr((uint32_t*)tx, sizeof(tx));
  SCB_CleanDCache_by_Addr((uint32_t*)rx, sizeof(rx));
  hspi = _hspi;

  for(int i = 0; i < MiscDiagChCount; i++)
    OutputsDiagBytes[i] = 0xFF;

  return result;
}

HAL_StatusTypeDef Misc_Outs_GetDiagnostic(eMiscDiagChannels channel, uint8_t *byte)
{
  HAL_StatusTypeDef result = OutputsAvailability[channel];
  *byte = OutputsDiagBytes[channel];
  OutputsDiagnosticStored[channel] = 0;
  return result;
}

HAL_StatusTypeDef Misc_GetKnockValueByRPM(float *value)
{
  *value = KnockValue;
  return KnockStatus;
}

HAL_StatusTypeDef Misc_GetKnockValueRaw(float *value)
{
  *value = KnockRawValue;
  return KnockStatus;
}

