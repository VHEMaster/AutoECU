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

#include "pid.h"
#include "misc.h"
#include "adc.h"
#include "delay.h"
#include "csps.h"
#include "sensors.h"
#include "defines.h"
#include "interpolation.h"
#include <string.h>
#include <stdlib.h>

#define O2_PID_P  -30.0f
#define O2_PID_I  -2.0f
#define O2_PID_D  -0.01f

#define O2_TEMP_OVERHEAT_THRESHOLD      1000
#define O2_HEATUP_TEMP_THRESHOLD        600
#define O2_HEATUP_TEMP_TIMEOUT_MS       10000
#define O2_DEFAULT_TEMP_TIMEOUT_MS      3000
#define O2_HEATER_OPENLOAD_TIMEOUT_MS   1000
#define O2_SPI_POLL_PERIOD_MS           200

#define O2_MAX_VOLTAGE            13.0f // 13.0 by Datasheet
#define O2_HEATUP_MAX_VOLTAGE     11.0f // 13.0 by Datasheet
#define O2_HEATUP_SLEW_RATE       0.3f  // 0.4 by Datasheet
#define O2_HEATUP_START_VOLTAGE   2.0f  // 8.5 by Datasheet
#define O2_PREHEAT_VOLTAGE        1.5f

#define O2_IDENT_DEVICE_CJ125 0x60
#define O2_IDENT_MASK_DEVICE 0xF8
#define O2_IDENT_MASK_VERSION 0x07
#define O2_IDENT_REG_RD 0x48
#define O2_DIAG_REG_RD 0x78
#define O2_INIT_REG1_RD 0x6C
#define O2_INIT_REG1_WR 0x56
#define O2_INIT_REG2_RD 0x7E
#define O2_INIT_REG2_WR 0x5A

#define O2_REG1_CALIBR 0x9C
#define O2_REG1_NORMAL 0x88
#define O2_REG2_NORMAL 0x00

#define KNOCK_CMD_GAIN      0x80
#define KNOCK_CMD_BP_FREQ   0x00
#define KNOCK_CMD_INT_TIME  0xC0
#define KNOCK_CMD_CH_SEL    0xE0
#define KNOCK_CMD_SO_MODE   0x40

#define KNOCK_HOLD() HAL_GPIO_WritePin(KNOCK_INT_GPIO_Port, KNOCK_INT_Pin, GPIO_PIN_RESET)
#define KNOCK_INTEGRATE() HAL_GPIO_WritePin(KNOCK_INT_GPIO_Port, KNOCK_INT_Pin, GPIO_PIN_SET)

typedef enum {
  LambdaStateInitial = 0,
  LambdaStateDeviceCheck,
  LambdaStateDiagCheck,
  LambdaStatePollCalibrate,
  LambdaStateSetCalibrate,
  LambdaStatePollEnable,
  LambdaStatePollPumpReset,
  LambdaStateWaitToHeat,
  LambdaStateHeating,
  LambdaStatePollInit1,
  LambdaStatePollInit2,
  LambdaStateCheckTemperature
}eLambdaState;

static const float o2_lambda[24] = { 0.65f, 0.7f, 0.75f, 0.8f, 0.822f, 0.85f, 0.9f, 0.95f, 0.97f, 0.99f, 1.003f, 1.01f, 1.05f, 1.1f, 1.132f, 1.179f, 1.429f, 1.701f, 1.99f, 2.434f, 3.413f, 5.391f, 7.506f, 10.119f };

static const float o2_ua_voltage[O2AmplificationFactorCount][24] = {
    { 0.51f, 0.707f, 0.884f, 1.041f, 1.104f, 1.177f, 1.299f, 1.409f, 1.448f, 1.48f, 1.5f, 1.507f, 1.548f, 1.596f, 1.624f, 1.663f, 1.832f, 1.964f, 2.069f, 2.186f, 2.342f, 2.49f, 2.565f, 2.614f },
    { 0.015f, 0.050f, 0.192f, 0.525f, 0.658f, 0.814f, 1.074f, 1.307f, 1.388f, 1.458f, 1.5f, 1.515f, 1.602f, 1.703f, 1.763f, 1.846f, 2.206f, 2.487f, 2.71f, 2.958f, 3.289f, 3.605f, 3.762f, 3.868f }
};

static const float o2_ur_voltage[15] = { 0.441f, 0.466f, 0.490f, 0.515f, 0.539f, 0.784f, 1.029f, 1.273f, 1.518f, 1.763f, 2.008f, 2.253f, 2.498f, 2.743f, 4.10f };
static const float o2_temperature[15] = { 1180, 1108, 1056, 1021, 992, 847, 780, 739, 710, 689, 670, 653, 640, 630, 0 };

static volatile uint32_t StepPhase = 0;

//Works for STEP_PH_GPIO_Port == STEP_PH2_GPIO_Port
static const uint32_t StepPos[4] = {
    (STEP_PH1_Pin | STEP_PH2_Pin) << 16,
    STEP_PH1_Pin | (STEP_PH2_Pin << 16),
    (STEP_PH1_Pin | STEP_PH2_Pin),
    (STEP_PH1_Pin << 16) | STEP_PH2_Pin,

};

#define STEP_APPEND() { STEP_PH1_GPIO_Port->BSRR = StepPos[StepPhase]; }
#define STEP_INCREMENT() { StepPhase = (StepPhase + 1) & 0x3; }
#define STEP_DECREMENT() { StepPhase = (StepPhase - 1) & 0x3; }

//Works for STEP_I0_GPIO_Port == STEP_I1_GPIO_Port
#define STEP_IDLE() { IdleValveStepMode = 0; if(STEP_I0_GPIO_Port == STEP_I1_GPIO_Port) { STEP_I0_GPIO_Port->BSRR = (STEP_I0_Pin | STEP_I1_Pin) << 16; } else { STEP_I0_GPIO_Port->BSRR = STEP_I0_Pin << 16; STEP_I1_GPIO_Port->BSRR = STEP_I1_Pin << 16; } }
//#define STEP_HOLD() { IdleValveStepMode = 1; if(STEP_I0_GPIO_Port == STEP_I1_GPIO_Port) { STEP_I0_GPIO_Port->BSRR = STEP_I0_Pin | (STEP_I1_Pin << 16); } else { STEP_I0_GPIO_Port->BSRR = STEP_I0_Pin; STEP_I1_GPIO_Port->BSRR = STEP_I1_Pin << 16; } }
//#define STEP_NORMAL() { IdleValveStepMode = 2; if(STEP_I0_GPIO_Port == STEP_I1_GPIO_Port) { STEP_I0_GPIO_Port->BSRR = STEP_I1_Pin | (STEP_I0_Pin << 16); } else { STEP_I0_GPIO_Port->BSRR = STEP_I0_Pin << 16; STEP_I1_GPIO_Port->BSRR = STEP_I1_Pin; } }
//#define STEP_ACCELERATE() { IdleValveStepMode = 3; if(STEP_I0_GPIO_Port == STEP_I1_GPIO_Port) { STEP_I0_GPIO_Port->BSRR = STEP_I1_Pin | STEP_I0_Pin; } else { STEP_I0_GPIO_Port->BSRR = STEP_I0_Pin; STEP_I1_GPIO_Port->BSRR = STEP_I1_Pin; } }

#define STEP_HOLD() { IdleValveStepMode = 1; if(STEP_I0_GPIO_Port == STEP_I1_GPIO_Port) { STEP_I0_GPIO_Port->BSRR = STEP_I1_Pin | STEP_I0_Pin; } else { STEP_I0_GPIO_Port->BSRR = STEP_I0_Pin; STEP_I1_GPIO_Port->BSRR = STEP_I1_Pin; } }
#define STEP_NORMAL() { IdleValveStepMode = 2; if(STEP_I0_GPIO_Port == STEP_I1_GPIO_Port) { STEP_I0_GPIO_Port->BSRR = STEP_I1_Pin | STEP_I0_Pin; } else { STEP_I0_GPIO_Port->BSRR = STEP_I0_Pin; STEP_I1_GPIO_Port->BSRR = STEP_I1_Pin; } }
#define STEP_ACCELERATE() { IdleValveStepMode = 3; if(STEP_I0_GPIO_Port == STEP_I1_GPIO_Port) { STEP_I0_GPIO_Port->BSRR = STEP_I1_Pin | STEP_I0_Pin; } else { STEP_I0_GPIO_Port->BSRR = STEP_I0_Pin; STEP_I1_GPIO_Port->BSRR = STEP_I1_Pin; } }

#define STEP_MAX_SPEED_FROM_START_TO_END_SLOW_ACCEL    2.0f //in seconds
#define STEP_MAX_SPEED_FROM_START_TO_END_MIDL_ACCEL    1.8f //in seconds
#define STEP_MAX_SPEED_FROM_START_TO_END_FAST_ACCEL    1.5f //in seconds
#define STEP_MAX_SPEED_FROM_START_TO_END_CALIBRATE     1.0f //in seconds
#define STEP_MAX_FREQ_SLOW_ACCEL  ((uint32_t)(STEP_MAX_SPEED_FROM_START_TO_END_SLOW_ACCEL * 1000000.0f / (float)IDLE_VALVE_POS_MAX))
#define STEP_MAX_FREQ_MIDL_ACCEL  ((uint32_t)(STEP_MAX_SPEED_FROM_START_TO_END_MIDL_ACCEL * 1000000.0f / (float)IDLE_VALVE_POS_MAX))
#define STEP_MAX_FREQ_FAST_ACCEL  ((uint32_t)(STEP_MAX_SPEED_FROM_START_TO_END_FAST_ACCEL * 1000000.0f / (float)IDLE_VALVE_POS_MAX))
#define STEP_MAX_FREQ_CALIBRATE   ((uint32_t)(STEP_MAX_SPEED_FROM_START_TO_END_CALIBRATE * 1000000.0f / (float)IDLE_VALVE_POS_MAX))
#define STEP_ACC_TO_NORM_DELAY    (STEP_MAX_FREQ_SLOW_ACCEL * 5.0f)

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

static uint8_t tx[32] ALIGNED(32) BUFFER_DMA;
static uint8_t rx[32] ALIGNED(32) BUFFER_DMA;

static volatile uint8_t semTx = 0;
static volatile uint8_t semRx = 0;

static uint8_t O2Device = 0;
static uint8_t O2Version = 0;
static uint32_t O2PwmPeriod = 255;
static volatile uint32_t *O2PwmDuty = NULL;
static HAL_StatusTypeDef KnockInitialStatus = HAL_OK;
static HAL_StatusTypeDef O2InitState = HAL_OK;

static uint8_t OutputsDiagBytes[MiscDiagChCount] = {0};
static uint8_t OutputsDiagnosticStored[MiscDiagChCount] = {0};
static HAL_StatusTypeDef OutputsAvailability[MiscDiagChCount] = {0};

static sMathPid o2_pid;

static sO2Status O2Status = {0};

static uint8_t knock_bandpass_filter_frequency = 42; //7.27kHz for D=79mm
static uint8_t knock_gain_value = 14;                //1.0
static uint8_t knock_integrator_time_constant = 22;  //280uS
static uint8_t knock_oscillator_freq_select = 6; //16MHz
static uint8_t knock_channel_select = 0;
static uint8_t knock_diagnostic_mode = 0;
static uint8_t knock_so_output_mode = 0;

static volatile sLambdaConfig LambdaConfig = {0};
static volatile sKnockConfig KnockConfig = {0};
static volatile uint8_t KnockConfigChanged = 0;
static volatile uint8_t IdleValveStepMode = 0;

static volatile HAL_StatusTypeDef IdleValvePositionStatus = HAL_OK;
static volatile uint8_t IdleValvePositionCurrent = 0;
static volatile uint8_t IdleValvePositionTarget = 0;
static volatile uint8_t IdleValveCalibratedOk = 0;
static volatile uint8_t IdleValveCalibrate = 0;
static volatile uint8_t IdleValveResetPosition = 0;
static volatile uint8_t IdleValveEnabled = 0;

volatile uint8_t nss_o2_off = 0;
volatile uint8_t nss_knock_off = 0;
volatile uint8_t nss_out1_off = 0;
volatile uint8_t nss_out2_off = 0;
volatile uint8_t nss_inj_off = 0;

STATIC_INLINE void Misc_CpltNssCheck(void)
{
  if(nss_o2_off) {
    SPI_NSS_O2_OFF();
    nss_o2_off = 0;
  }
  if(nss_knock_off) {
    SPI_NSS_KNOCK_OFF();
    nss_knock_off = 0;
  }
  if(nss_out1_off) {
    SPI_NSS_OUTS1_OFF();
    nss_out1_off = 0;
  }
  if(nss_out2_off) {
    SPI_NSS_OUTS2_OFF();
    nss_out2_off = 0;
  }
  if(nss_inj_off) {
    SPI_NSS_INJ_OFF();
    nss_inj_off = 0;
  }
}

void Misc_ErrorCallback(SPI_HandleTypeDef * _hspi)
{
  if(_hspi == hspi) {
    Misc_CpltNssCheck();

  }
}

void Misc_TxCpltCallback(SPI_HandleTypeDef * _hspi)
{
  if(_hspi == hspi) {
    Misc_CpltNssCheck();
    semTx = 1;
  }
}

void Misc_RxCpltCallback(SPI_HandleTypeDef * _hspi)
{
  if(_hspi == hspi) {
    Misc_CpltNssCheck();
    semRx = 1;
  }
}

void Misc_TxRxCpltCallback(SPI_HandleTypeDef * _hspi)
{
  if(_hspi == hspi) {
    Misc_CpltNssCheck();
    semTx = 1;
    semRx = 1;
  }
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

static int8_t Knock_Cmd(uint8_t cmd, uint8_t *read)
{
  static uint8_t state = 0;
  switch(state) {
    case 0:
      tx[0] = cmd;
      nss_knock_off = 1;
      SPI_NSS_KNOCK_ON();
      HAL_SPI_TransmitReceive_IT(hspi, tx, rx, 1);
      state++;

      break;
    case 1:
      if(waitTxRxCplt())
      {
        //SPI_NSS_KNOCK_OFF();
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

static int8_t O2_Write(uint8_t cmd, uint8_t data)
{
  static uint8_t state = 0;
  switch(state) {
    case 0:
      tx[0] = cmd;
      tx[1] = data;
      nss_o2_off = 1;
      SPI_NSS_O2_ON();
      HAL_SPI_TransmitReceive_IT(hspi, tx, rx, 2);
      state++;

      break;
    case 1:
      if(waitTxRxCplt())
      {
        //SPI_NSS_O2_OFF();
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
      nss_o2_off = 1;
      SPI_NSS_O2_ON();
      HAL_SPI_TransmitReceive_IT(hspi, tx, rx, 2);
      state++;

      break;
    case 1:
      if(waitTxRxCplt())
      {
        //SPI_NSS_O2_OFF();
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
  else if(dutycycle < 0.0f)
    dutycycle = 0.0f;

  *O2PwmDuty = (float)O2PwmPeriod * dutycycle;
}

static void O2_SetHeaterVoltage(float voltage)
{
  float dutycycle;
  float power = adc_get_voltage(AdcChPowerVoltage);

  if(power < 5.0f) {
    dutycycle = 0;
  } else if(voltage > power) {
    dutycycle = 1.0f;
    O2Status.HeaterVoltage = power;
  } else if(voltage < 0.0f) {
    dutycycle = 0.0f;
    O2Status.HeaterVoltage = 0.0f;
  } else {
    dutycycle = voltage / power;
    O2Status.HeaterVoltage = voltage;
  }

  O2_SetHeaterDutyCycle(dutycycle * dutycycle);

}

static float O2_GetLambda(float voltage, float offset)
{
  float lambda = 1.0f;

  sMathInterpolateInput ipVoltage = math_interpolate_input(voltage + offset, o2_ua_voltage[O2Status.AmplificationFactor], ITEMSOF(o2_lambda));
  lambda = math_interpolate_1d(ipVoltage, o2_lambda);
  if(lambda < 0)
    lambda = 0.0f;

  return lambda;
}

static float O2_GetTemperature(float voltage)
{
  float temperature;

  sMathInterpolateInput ipVoltage = math_interpolate_input(voltage, o2_ur_voltage, ITEMSOF(o2_ur_voltage));
  temperature = math_interpolate_1d(ipVoltage, o2_temperature);
  if(temperature < 0)
    temperature = 0.0f;

  return temperature;
}

static int8_t O2_GetDevice(uint8_t *device)
{
  return O2_Read(O2_IDENT_REG_RD, device);
}

static int8_t O2_GetDiag(uint8_t *diag)
{
  return O2_Read(O2_DIAG_REG_RD, diag);
}

static int8_t O2_SetInit1Reg(uint8_t reg)
{
  return O2_Write(O2_INIT_REG1_WR, reg);
}

static int8_t O2_SetInit2Reg(uint8_t reg)
{
  return O2_Write(O2_INIT_REG2_WR, reg);
}

static int8_t O2_SetPumpReferenceCurrent(uint8_t current)
{
  current &= 0x0F;

  return O2_SetInit2Reg(O2_REG2_NORMAL + current);
}

static int8_t O2_Calibrate(eO2AmplificationFactor factor)
{
  return O2_SetInit1Reg(O2_REG1_CALIBR + factor);
}

static int8_t O2_Enable(eO2AmplificationFactor factor)
{
  return O2_SetInit1Reg(O2_REG1_NORMAL + factor);
}

static void O2_CriticalLoop(void)
{
  static const float const_heatup_lpf_koff = 0.33f; //3 sec
  static float heatup_max_voltage_lpf = O2_HEATUP_MAX_VOLTAGE;
  static float heatup_max_voltage = O2_HEATUP_MAX_VOLTAGE;
  static uint32_t last_process = 0;
  float temperaturevoltage = adc_get_voltage(AdcChO2UR);
  float lambdavoltage = adc_get_voltage(AdcChO2UA);
  float offsetvoltage = O2Status.OffsetVoltage;
  float o2heater;
  float heatup_lpf_koff;
  uint32_t now = Delay_Tick;
  uint32_t diff = DelayDiff(now, last_process);
  O2Status.Lambda = O2_GetLambda(lambdavoltage, offsetvoltage);
  O2Status.Temperature = O2_GetTemperature(temperaturevoltage);
  O2Status.TemperatureVoltage = temperaturevoltage;

  if(O2Status.Available && O2Status.Working && O2Status.Valid) {
    heatup_max_voltage = O2_MAX_VOLTAGE;
    if(diff >= 5000) {
      last_process = now;
      o2heater = math_pid_update(&o2_pid, temperaturevoltage, now);
      O2_SetHeaterVoltage(o2heater);
    }
  } else {
    heatup_max_voltage = O2_HEATUP_MAX_VOLTAGE;
    last_process = now;
  }

  heatup_lpf_koff = (float)diff * 0.000001f * const_heatup_lpf_koff;
  if(heatup_max_voltage > heatup_max_voltage_lpf) {
    heatup_max_voltage_lpf = heatup_max_voltage * heatup_lpf_koff + heatup_max_voltage_lpf * (1.0f - heatup_lpf_koff);
  } else {
    heatup_max_voltage_lpf = heatup_max_voltage;
  }
  math_pid_set_clamp(&o2_pid, 0.0f, heatup_max_voltage_lpf);
}

static int8_t O2_Loop(void)
{
  static uint8_t was_engine_running = 0;
  static eLambdaState state = LambdaStateInitial;
  static uint32_t last_spi_check = 0;
  static uint32_t calibrate_timestamp = 0;
  static uint32_t temperature_wait_timestamp = 0;
  static float o2heater = 0.0f;
  static uint8_t device,diag;
  static eO2AmplificationFactor factor = 0;
  static uint8_t pump_ref_current = 0;
  eO2AmplificationFactor factor_tmp;
  uint8_t pump_ref_current_tmp;
  static uint8_t need_reset_init1 = 0;
  static uint8_t need_reset_init2 = 0;
  static uint32_t temperature_timeout = O2_HEATUP_TEMP_TIMEOUT_MS;
  static uint32_t heater_openload_time = 0;
  uint8_t is_engine_running = 0;
  uint8_t rotates = csps_isrotates();
  uint8_t running = csps_isrunning();
  uint8_t ignore_heater_error = 1;
  uint32_t now = Delay_Tick;
  uint32_t diff;
  int8_t status;
  int8_t retvalue = 1;
  sMathInterpolateInput ipLambda;
  uint8_t force = LambdaConfig.isLambdaForceEnabled;
  float offset_voltage;
  float expected_voltage;
  float engine_temperature;
  uint32_t heatup_time_diff;

  is_engine_running = force || running;
  if(was_engine_running != is_engine_running) {
    was_engine_running = is_engine_running;
  }

  if(!need_reset_init1) {
    factor_tmp = O2Status.AmplificationFactor;
    if(factor_tmp != factor) {
      factor = factor_tmp;
      need_reset_init1 = 1;
    }
  }

  if(!need_reset_init2) {
    pump_ref_current_tmp = O2Status.PumpReferenceCurrent;
    if(pump_ref_current_tmp != pump_ref_current) {
      pump_ref_current = pump_ref_current_tmp;
      need_reset_init2 = 1;
    }
  }

  if(O2InitState != HAL_OK || O2Status.TemperatureStatus != HAL_OK || O2Status.HeaterStatus != HAL_OK) {
    o2heater = 0.0f;
    O2_SetHeaterVoltage(o2heater);
    return retvalue;
  }

  if(O2Status.Temperature > O2_TEMP_OVERHEAT_THRESHOLD) {
    O2Status.TemperatureStatus = HAL_ERROR;
    O2Status.HeaterStatus = HAL_ERROR;
    O2Status.Valid = 0;
    o2heater = 0.0f;
    O2_SetHeaterVoltage(o2heater);
    return retvalue;
  }

  switch(state) {
    case LambdaStateInitial :
      if(DelayDiff(now, last_spi_check) > O2_SPI_POLL_PERIOD_MS * 1000) {
        last_spi_check = now;
        state = LambdaStateDeviceCheck;
      } else if(O2Status.Valid) {
        state = LambdaStateCheckTemperature;
      } else if(O2Status.Available && is_engine_running) {
        state = LambdaStatePollCalibrate;
      }
      else {
        if(O2Status.Valid && need_reset_init1) {
          state = LambdaStatePollInit1;
        } else if(O2Status.Valid && need_reset_init2) {
            state = LambdaStatePollInit2;
        } else {
          state = LambdaStateInitial;
        }
      }
      break;
    case LambdaStateDeviceCheck :
      status = O2_GetDevice(&device);
      retvalue = 0;
      if(status == 1) {
        retvalue = 1;
        O2Device = device & O2_IDENT_MASK_DEVICE;
        O2Version = device & O2_IDENT_MASK_VERSION;
        if(O2Device == O2_IDENT_DEVICE_CJ125) {
          O2Status.Available = 1;
          state = LambdaStateDiagCheck;
        } else {
          status = -1;
        }
      }
      if(status == -1) {
        retvalue = -1;
        O2Status.Available = 0;
        O2Status.Working = 0;
        O2Status.Valid = 0;
        O2_SetHeaterVoltage(0);
        state = LambdaStateInitial;
      }
      break;
    case LambdaStateDiagCheck :
      status = O2_GetDiag(&diag);
      retvalue = 0;
      if(status) {
        retvalue = 1;
        O2Status.Diag.Byte = diag ^ 0xFF;
        O2Status.Working = 1;
        if(O2Status.Valid) {
          O2Status.Diag.Fields.IAIP = O2DiagOK;
          O2Status.Diag.Fields.UN = O2DiagOK;
          O2Status.Diag.Fields.VM = O2DiagOK;
          state = LambdaStateCheckTemperature;
        }
        else if(force || is_engine_running) {
          state = LambdaStatePollCalibrate;
        }
        else {
          state = LambdaStateInitial;
        }

        if(O2Status.Diag.Fields.DIAHGD != O2DiagOK) {
          if(O2Status.Diag.Fields.DIAHGD == O2DiagNoPower) {
            if(heater_openload_time == 0 || (rotates && !running)) {
              heater_openload_time = now;
            } else if(DelayDiff(now, heater_openload_time) > O2_HEATER_OPENLOAD_TIMEOUT_MS) {
              ignore_heater_error = 0;
            }
          } else {
            heater_openload_time = 0;
            ignore_heater_error = 0;
          }
          if(ignore_heater_error) {
            O2Status.Diag.Fields.DIAHGD = O2DiagOK;
          } else {
            O2Status.HeaterStatus = HAL_ERROR;
            O2Status.Valid = 0;
            o2heater = 0.0f;
            O2_SetHeaterVoltage(o2heater);
            state = LambdaStateInitial;
          }
        } else {
          heater_openload_time = 0;
        }
      }
      break;
    case LambdaStatePollCalibrate :
      O2_SetHeaterVoltage(O2_PREHEAT_VOLTAGE);
      status = O2_Calibrate(factor);
      retvalue = 0;
      if(status) {
        need_reset_init1 = 0;
        need_reset_init2 = 0;
        retvalue = 1;
        calibrate_timestamp = now;
        state = LambdaStateSetCalibrate;
      }
      break;
    case LambdaStateSetCalibrate :
      diff = DelayDiff(now, calibrate_timestamp);
      if(diff > 100000) {
        O2Status.ReferenceVoltage = adc_get_voltage(AdcChO2UR);
        offset_voltage = adc_get_voltage(AdcChO2UA);

        ipLambda = math_interpolate_input(1.0f, o2_lambda, ITEMSOF(o2_lambda));
        expected_voltage = math_interpolate_1d(ipLambda, o2_ua_voltage[factor]);

        O2Status.OffsetVoltage = expected_voltage - offset_voltage;

        state = LambdaStatePollEnable;
      }
      break;
    case LambdaStatePollEnable :
      status = O2_Enable(factor);
      retvalue = 0;
      if(status) {
        need_reset_init1 = 0;
        retvalue = 1;
        calibrate_timestamp = now;
        state = LambdaStatePollPumpReset;
      }
      break;
    case LambdaStatePollPumpReset :
      status = O2_SetPumpReferenceCurrent(pump_ref_current);
      retvalue = 0;
      if(status) {
        need_reset_init2 = 0;
        retvalue = 1;
        calibrate_timestamp = now;
        state = LambdaStateWaitToHeat;
      }
      break;
    case LambdaStateWaitToHeat :
      if(!force && !is_engine_running) {
        O2_SetHeaterVoltage(O2_PREHEAT_VOLTAGE);
        O2Status.Valid = 0;
        state = LambdaStateInitial;
        break;
      }
      O2_SetHeaterVoltage(O2_PREHEAT_VOLTAGE);
      sens_get_engine_temperature(&engine_temperature);
      diff = DelayDiff(now, calibrate_timestamp);
      if(diff > 28000000 ||
          (engine_temperature > 75.0f && diff > 5000000) ||
          (engine_temperature > 50.0f && diff > 8000000) ||
          (engine_temperature > 30.0f && diff > 20000000) ||
          (engine_temperature > 20.0f && diff > 24000000) ||
          (engine_temperature > 10.0f && diff > 26000000)) {
        o2heater = O2_HEATUP_START_VOLTAGE;
        O2_SetHeaterVoltage(o2heater);
        calibrate_timestamp = now;
        state = LambdaStateHeating;
      }
      break;
    case LambdaStateHeating :
      if(o2heater >= O2_HEATUP_MAX_VOLTAGE || O2Status.Temperature > O2_HEATUP_TEMP_THRESHOLD) {
        math_pid_set_target(&o2_pid, O2Status.ReferenceVoltage);
        math_pid_reset(&o2_pid);
        calibrate_timestamp = now;
        temperature_wait_timestamp = now;
        state = LambdaStatePollInit1;
        need_reset_init1 = 1;
        need_reset_init2 = 1;
        temperature_timeout = O2_HEATUP_TEMP_TIMEOUT_MS;
      }
      heatup_time_diff = DelayDiff(now, calibrate_timestamp);
      if(heatup_time_diff > 100000) {
        o2heater += O2_HEATUP_SLEW_RATE * ((float)heatup_time_diff * 0.000001f);
        O2_SetHeaterVoltage(o2heater);
        calibrate_timestamp = now;
      }
      break;
    case LambdaStatePollInit1 :
      if(need_reset_init1 && retvalue == 1) {
        status = O2_Enable(factor);
        retvalue = 0;
        if(status) {
          need_reset_init1 = 0;
          retvalue = 1;
          state = LambdaStatePollInit2;
        }
      } else {
        state = LambdaStatePollInit2;
      }
      /* no break */
    case LambdaStatePollInit2 :
      if(need_reset_init2 && retvalue == 1) {
        status = O2_SetPumpReferenceCurrent(pump_ref_current);
        retvalue = 0;
        if(status) {
          need_reset_init2 = 0;
          retvalue = 1;
          state = LambdaStateCheckTemperature;
        }
      } else {
        state = LambdaStateCheckTemperature;
      }
      /* no break */
    case LambdaStateCheckTemperature :
      if(O2Status.Temperature > O2_HEATUP_TEMP_THRESHOLD) {
        O2Status.Valid = 1;
        O2Status.TemperatureStatus = HAL_OK;
        temperature_wait_timestamp = now;
        temperature_timeout = O2_DEFAULT_TEMP_TIMEOUT_MS;
        state = LambdaStateInitial;
      } else {
        if(DelayDiff(now, temperature_wait_timestamp) > temperature_timeout * 1000) {
          O2Status.TemperatureStatus = HAL_ERROR;
          O2Status.Valid = 0;
          o2heater = 0.0f;
          O2_SetHeaterVoltage(o2heater);
          state = LambdaStateInitial;
        }
      }
      break;
    default:
      state = LambdaStateInitial;
      break;
  }
  return retvalue;
}

sO2Status Misc_O2_GetStatus(void)
{
  return O2Status;
}

static int8_t Outs_Loop(void)
{
  static uint8_t state = 0;
  static eMiscDiagChannels channel = 0;
  static uint8_t failure_stored = 0;
  GPIO_PinState pin;

  do {
    switch(state) {
      case 0:
        SPI_NSS_INJ_OFF(); SPI_NSS_OUTS1_OFF(); SPI_NSS_OUTS2_OFF();

        switch(channel) {
          case MiscDiagChInjectors: SPI_NSS_INJ_ON(); nss_inj_off = 1; break;
          case MiscDiagChOutputs1: SPI_NSS_OUTS1_ON(); nss_out1_off = 1; break;
          case MiscDiagChOutputs2: SPI_NSS_OUTS2_ON(); nss_out2_off = 1; break;
          default: channel = 0; continue;
        }
        state++;
        break;
      case 1:
        pin = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5); //SPI4_MISO
        failure_stored = pin != GPIO_PIN_RESET;
        tx[0] = 0xFF;
        HAL_SPI_TransmitReceive_IT(hspi, tx, rx, 1);
        state++;
        break;
      case 2:
        if(waitTxRxCplt()) {
          //SPI_NSS_INJ_OFF(); SPI_NSS_OUTS1_OFF(); SPI_NSS_OUTS2_OFF();

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
  static uint8_t data = 0;
  uint8_t check[2];
  int8_t retval = 1;
  int8_t status;
  uint8_t read;

  if(KnockInitialStatus != HAL_OK)
    return 1;

  switch(state) {
    case 0:
      check[0] = KnockConfig.gain_value;
      if(check[0] != knock_gain_value) {
        knock_gain_value = check[0];
        data = KNOCK_CMD_GAIN | (knock_gain_value & 0x3F);
        state++;
      } else {
        state += 2;
      }
      break;
    case 2:
      check[0] = KnockConfig.bandpass_filter_frequency;
      if(check[0] != knock_bandpass_filter_frequency) {
        knock_bandpass_filter_frequency = check[0];
        data = KNOCK_CMD_BP_FREQ | (knock_bandpass_filter_frequency & 0x3F);
        state++;
      } else {
        state += 2;
      }
      break;
    case 4:
      check[0] = KnockConfig.integrator_time_constant;
      if(check[0] != knock_integrator_time_constant) {
        knock_integrator_time_constant = check[0];
        data = KNOCK_CMD_INT_TIME | (knock_integrator_time_constant & 0x1F);
        state++;
      } else {
        state += 2;
      }
      break;
    case 6:
      check[0] = KnockConfig.channel_select;
      check[1] = KnockConfig.diagnostic_mode;
      if(check[0] != knock_channel_select || check[1] != knock_diagnostic_mode) {
        knock_channel_select = check[0];
        knock_diagnostic_mode = check[1];
        data = KNOCK_CMD_CH_SEL | (knock_channel_select & 0x1) | ((knock_diagnostic_mode & 0xF) << 1);
        state++;
      } else {
        state += 2;
      }
      break;
    case 8:
      check[0] = KnockConfig.oscillator_freq_select;
      check[1] = KnockConfig.so_output_mode;
      if(check[0] != knock_oscillator_freq_select || check[1] != knock_so_output_mode) {
        knock_oscillator_freq_select = check[0];
        knock_so_output_mode = check[1];
        data = KNOCK_CMD_SO_MODE | ((knock_oscillator_freq_select & 0xF) << 1) | (knock_so_output_mode & 0x1);
        state++;
      } else {
        state += 2;
      }
      break;
    case 1:
    case 3:
    case 5:
    case 7:
    case 9:
      retval = 0;
      status = Knock_Cmd(data, &read);
      if(status) {
        retval = 1;
        state++;
      }
      break;
    case 10:
      KnockConfigChanged = 0;
      state = 0;
      break;
    default:
      state = 0;
      break;
  }

  return retval;
}

static void Knock_CriticalLoop(void)
{
  if(KnockInitialStatus != HAL_OK)
    return;
}

INLINE uint8_t Misc_GetIdleValvePosition(void)
{
  return IdleValvePositionCurrent;
}

INLINE uint8_t Misc_IsIdleValveMoving(void)
{
  return IdleValveStepMode > 1;
}

INLINE void Misc_SetIdleValvePosition(uint8_t position)
{
  IdleValvePositionTarget = position;
}

INLINE int8_t Misc_ResetIdleValve(uint8_t reset_position)
{
  static uint32_t calibrate_time = 0;
  static uint8_t state = 0;
  uint32_t now = Delay_Tick;

  switch(state) {
    case 0:
      IdleValveResetPosition = reset_position;
      IdleValveCalibratedOk = 0;
      IdleValveCalibrate = 1;
      calibrate_time = now;
      state++;
      break;
    case 1:
      if(IdleValveCalibratedOk) {
        IdleValveCalibrate = 0;
        IdleValveCalibratedOk = 0;
        state = 0;
        return 1;
      } else {
        if(DelayDiff(now, calibrate_time) > STEP_MAX_SPEED_FROM_START_TO_END_CALIBRATE * 3 * 1000000) {
          state = 0;
          IdleValveCalibrate = 0;
          IdleValveCalibratedOk = 0;
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

INLINE HAL_StatusTypeDef Misc_GetIdleValveStatus(void)
{
  return IdleValvePositionStatus;
}

static void IdleValve_CriticalLoop(void)
{
  GPIO_PinState idle_valve_state_pin = HAL_GPIO_ReadPin(STEP_ERR_GPIO_Port, STEP_ERR_Pin);

  IdleValvePositionStatus = idle_valve_state_pin == GPIO_PIN_RESET ? HAL_ERROR : HAL_OK;
}

INLINE void Misc_EnableIdleValvePosition(uint8_t enablement_position)
{
  IdleValvePositionCurrent = enablement_position;
  IdleValvePositionTarget = enablement_position;
  StepPhase = 0;
  for(int i = 0; i < (enablement_position & 3); i++)
    STEP_INCREMENT();
  STEP_APPEND();
  DelayUs(20);
  STEP_HOLD();
  IdleValveEnabled = 1;
}

static void IdleValve_FastLoop(void)
{
  static uint32_t last_tick = 0;
  static uint8_t is_hold = 0;
  static uint32_t last_move = 0;
  uint32_t now = Delay_Tick;
  uint8_t current = IdleValvePositionCurrent;
  uint8_t target = IdleValvePositionTarget;
  uint32_t step_period = STEP_MAX_FREQ_SLOW_ACCEL;
  uint32_t target_diff_abs;
  static uint8_t is_calibrating = 0;
  static uint16_t calibration_steps = 0;
  static uint8_t mode = 1;
  static uint8_t mode_prev = 1;

  if(IdleValveEnabled) {
    if(!!IdleValveCalibrate == !!IdleValveCalibratedOk || !IdleValveCalibrate) {
      is_calibrating = 0;
      calibration_steps = 0;
      target_diff_abs = abs(current - target);
      if(target_diff_abs > 30) {
        step_period = STEP_MAX_FREQ_FAST_ACCEL;
      } else if(target_diff_abs > 15) {
        step_period = STEP_MAX_FREQ_MIDL_ACCEL;
      } else {
        step_period = STEP_MAX_FREQ_SLOW_ACCEL;
      }
      if(DelayDiff(now, last_tick) > step_period) {
        if(current != target) {
          STEP_ACCELERATE();
          last_tick = now;
          mode = 3;
          if(mode != mode_prev) {
            mode_prev = 3;
            return;
          }

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
            mode = mode_prev = 1;
          } else {
            if(DelayDiff(now, last_move) > 1000000) {
              mode = mode_prev = 1;
              STEP_HOLD();
              is_hold = 1;
              last_move = now;
            } else if(DelayDiff(now, last_move) > STEP_ACC_TO_NORM_DELAY) {
              mode = mode_prev = 2;
              STEP_NORMAL();
            }
          }
        }
      }
    } else if(IdleValveCalibrate) {
      if(!is_calibrating) {
        is_calibrating = 1;
        calibration_steps = 0;
        IdleValvePositionCurrent = 0;
        //STEP_ACCELERATE();
        STEP_NORMAL();
        mode = mode_prev = 2;
        last_tick = now;
      } else {
        if(is_calibrating == 1) {
          step_period = STEP_MAX_FREQ_CALIBRATE;
          if(DelayDiff(now, last_tick) > step_period) {
            last_tick = now;
            STEP_DECREMENT();
            STEP_APPEND();

            if(++calibration_steps >= IDLE_VALVE_POS_MAX && StepPhase == 0) {
              calibration_steps = 0;
              is_calibrating = 2;
              IdleValvePositionCurrent = 0;
              STEP_NORMAL();
            }
          }
        } else if(is_calibrating == 2) {
          step_period = STEP_MAX_FREQ_CALIBRATE;
          if(DelayDiff(now, last_tick) > step_period) {
            last_tick = now;
            STEP_INCREMENT();
            STEP_APPEND();

            calibration_steps++;
            IdleValvePositionCurrent = calibration_steps;
            IdleValvePositionTarget = calibration_steps;

            if(calibration_steps >= IdleValveResetPosition) {
              is_calibrating = 0;
              calibration_steps = 0;
              mode = mode_prev = 2;
              STEP_NORMAL();
              last_move = now;
              last_tick = now;
              IdleValveCalibratedOk = 1;
            }
          }
        } else {
        	is_calibrating = 1;
        }
      }
    }
  } else {
    last_tick = now;
    last_move = now;
  }
}


void Misc_Fast_Loop(void)
{
  IdleValve_FastLoop();
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
    if(DelayDiff(now, lastKnockExec) >= 50000 || KnockConfigChanged) {
      lastKnockExec = now;
      work_knock = 1;
    }
    if(DelayDiff(now, lastO2Exec) >= 10000) {
      lastO2Exec = now;
      work_o2 = 1;
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

HAL_StatusTypeDef Mics_Knock_Init(void)
{
  uint8_t data,read;

  KnockConfig.bandpass_filter_frequency = knock_bandpass_filter_frequency;
  KnockConfig.gain_value = knock_gain_value;
  KnockConfig.integrator_time_constant = knock_integrator_time_constant;
  KnockConfig.oscillator_freq_select = knock_oscillator_freq_select;
  KnockConfig.channel_select = knock_channel_select;
  KnockConfig.diagnostic_mode = knock_diagnostic_mode;
  KnockConfig.so_output_mode = knock_so_output_mode;

  KNOCK_INTEGRATE();
  DelayUs(100);
  KNOCK_HOLD();

  data = KNOCK_CMD_GAIN | (knock_gain_value & 0x3F);
  while(!Knock_Cmd(data, &read)) {}

  DelayUs(5);

  data = KNOCK_CMD_BP_FREQ | (knock_bandpass_filter_frequency & 0x3F);
  while(!Knock_Cmd(data, &read)) {}

  DelayUs(5);

  data = KNOCK_CMD_INT_TIME | (knock_integrator_time_constant & 0x1F);
  while(!Knock_Cmd(data, &read)) {}

  DelayUs(5);

  data = KNOCK_CMD_CH_SEL | (knock_channel_select & 0x1) | ((knock_diagnostic_mode & 0xF) << 1);
  while(!Knock_Cmd(data, &read)) {}

  DelayUs(5);

  data = KNOCK_CMD_SO_MODE | ((knock_oscillator_freq_select & 0xF) << 1) | (knock_so_output_mode & 0x1);
  while(!Knock_Cmd(data, &read)) {}

  KnockInitialStatus = HAL_OK;

  return HAL_OK;
}

HAL_StatusTypeDef Misc_O2_Init(uint32_t pwm_period, volatile uint32_t *pwm_duty)
{
  uint8_t device;

  O2PwmPeriod = pwm_period + 1;
  O2PwmDuty = pwm_duty;

  O2Status.Available = 1;
  O2Status.Lambda = 1.0f;
  O2Status.Valid = 0;
  O2Status.Working = 0;
  O2Status.AmplificationFactor = O2AmplificationFactor8;
  O2Status.PumpReferenceCurrent = 0;
  O2Status.TemperatureStatus = HAL_OK;
  O2Status.HeaterStatus = HAL_OK;

  math_pid_init(&o2_pid);
  math_pid_set_koffs(&o2_pid, O2_PID_P, O2_PID_I, O2_PID_D);
  math_pid_set_clamp(&o2_pid, 0.0f, O2_HEATUP_MAX_VOLTAGE);

  while(!O2_GetDevice(&device)) {};

  O2Device = device & O2_IDENT_MASK_DEVICE;
  O2Version = device & O2_IDENT_MASK_VERSION;
  if(O2Device != O2_IDENT_DEVICE_CJ125) {
    O2Status.Available = 0;
    O2InitState = HAL_ERROR;
    return HAL_ERROR;
  }

  O2InitState = HAL_OK;

  O2Status.Working = 1;

  return HAL_OK;
}

HAL_StatusTypeDef Misc_Init(SPI_HandleTypeDef * _hspi)
{
  HAL_StatusTypeDef result = HAL_OK;

  HAL_GPIO_WritePin(O2_NRST_GPIO_Port, O2_NRST_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SW_NRST_GPIO_Port, SW_NRST_Pin, GPIO_PIN_SET);
  DelayMs(5);

  memset(tx, 0, sizeof(tx));
  memset(rx, 0, sizeof(tx));

  SCB_CleanDCache_by_Addr((uint32_t*)tx, sizeof(tx));
  SCB_CleanDCache_by_Addr((uint32_t*)rx, sizeof(rx));
  hspi = _hspi;

  for(int i = 0; i < MiscDiagChCount; i++)
    OutputsDiagBytes[i] = 0xFF;

  STEP_IDLE();

  return result;
}

HAL_StatusTypeDef Misc_Outs_GetDiagnostic(eMiscDiagChannels channel, uint8_t *byte)
{
  HAL_StatusTypeDef result = OutputsAvailability[channel];
  *byte = OutputsDiagBytes[channel] ^ 0xFF;
  OutputsDiagnosticStored[channel] = 0;
  return result;
}

INLINE void Knock_SetState(eKnockState state)
{
  if(state == KnockStateIntegrate) {
    KNOCK_INTEGRATE();
  } else if (state == KnockStateHold){
    KNOCK_HOLD();
  }
}

INLINE void Knock_SetBandpassFilterFrequency(uint8_t value)
{
  value = CLAMP(value, 0, 63);
  if(value != KnockConfig.bandpass_filter_frequency) {
    KnockConfig.bandpass_filter_frequency = value;
    KnockConfigChanged = 1;
  }
}

INLINE void Knock_SetGainValue(uint8_t value)
{
  value = CLAMP(value, 0, 63);
  if(value != KnockConfig.gain_value) {
    KnockConfig.gain_value = value;
    KnockConfigChanged = 1;
  }
}

INLINE void Knock_SetIntegratorTimeConstant(uint8_t value)
{
  value = CLAMP(value, 0, 31);
  if(value != KnockConfig.integrator_time_constant) {
    KnockConfig.integrator_time_constant = value;
    KnockConfigChanged = 1;
  }
}

INLINE void Knock_SetOscillatorFrequency(uint8_t value)
{
  if(value != KnockConfig.oscillator_freq_select) {
    KnockConfig.oscillator_freq_select = value;
    KnockConfigChanged = 1;
  }
}

INLINE void Knock_SetChannel(uint8_t value)
{
  if(value != KnockConfig.channel_select) {
    KnockConfig.channel_select = value;
    KnockConfigChanged = 1;
  }
}

INLINE void Knock_SetDiagnosticMode(uint8_t value)
{
  if(value != KnockConfig.diagnostic_mode) {
    KnockConfig.diagnostic_mode = value;
    KnockConfigChanged = 1;
  }
}

INLINE void Knock_SetSoOutputMode(uint8_t value)
{
  if(value != KnockConfig.so_output_mode) {
    KnockConfig.so_output_mode = value;
    KnockConfigChanged = 1;
  }
}

INLINE HAL_StatusTypeDef Knock_GetStatus(void)
{
  return KnockInitialStatus;
}

INLINE uint8_t Knock_GetBandpassFilterFrequency(void)
{
  return KnockConfig.bandpass_filter_frequency;
}

INLINE uint8_t Knock_GetGainValue(void)
{
  return KnockConfig.gain_value;
}

INLINE uint8_t Knock_GetIntegratorTimeConstant(void)
{
  return KnockConfig.integrator_time_constant;
}

INLINE uint8_t Knock_GetOscillatorFrequency(void)
{
  return KnockConfig.oscillator_freq_select;
}

INLINE uint8_t Knock_GetChannel(void)
{
  return KnockConfig.channel_select;
}

INLINE uint8_t Knock_GetDiagnosticMode(void)
{
  return KnockConfig.diagnostic_mode;
}

INLINE uint8_t Knock_GetSoOutputMode(void)
{
  return KnockConfig.so_output_mode;
}

INLINE void O2_SetAmplificationFactor(eO2AmplificationFactor factor)
{
  if(factor < O2AmplificationFactorCount)
    O2Status.AmplificationFactor = factor;
}

INLINE eO2AmplificationFactor O2_GetAmplificationFactor(void)
{
  return O2Status.AmplificationFactor;
}

INLINE void O2_SetLambdaForceEnabled(uint8_t enabled)
{
  LambdaConfig.isLambdaForceEnabled = enabled;
}

