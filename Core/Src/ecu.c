/*
 * ecu.c
 *
 *  Created on: Mar 4, 2022
 *      Author: VHEMaster
 */

/* TODO:
 *
 * О холодном пуске: https://www.drive2.ru/l/469206692622499850/
 * https://www.drive2.ru/l/526420604807546577
 * https://forum.injectorservice.com.ua/viewtopic.php?t=8147
 *
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
#include "bluetooth.h"
#include "outputs.h"
#include "injector.h"
#include "sst25vf032b.h"
#include "interpolation.h"
#include "math_misc.h"
#include "config.h"
#include "pid.h"
#include "crc.h"
#include "failures.h"
#include "can.h"
#include "kline.h"
#include "xProFIFO.h"

#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include <float.h>
#include "arm_math.h"
#include "math_fast.h"

#define DEFAULT_IDLE_VALVE_POSITION 100
#define PRESSURE_ACCEPTION_FEATURE  0
#define IGNITION_ACCEPTION_FEATURE  1
#define KNOCK_DETONATION_INCREASING_ADVANCE 0
#define KNOCK_LOW_NOISE_ON_ENGINE_TEMP_THRESHOLD 70.0f
#define FUEL_PUMP_ON_INJ_CH1_ONLY   1
#define INJECTORS_ON_INJ_CH1_ONLY   1
#define ACCELERATION_POINTS_COUNT   12
#define LEARN_ENRICHMENT_POST_CYCLES_DELAY  (ECU_CYLINDERS_COUNT * 8)
#define IDLE_ACCELERATE_POST_CYCLES_DELAY   (ECU_CYLINDERS_COUNT * 8)

typedef float (*math_interpolate_2d_set_func_t)(sMathInterpolateInput input_x, sMathInterpolateInput input_y,
    uint32_t y_size, float (*table)[], float new_value, float limit_l, float limit_h);
typedef float (*math_interpolate_2d_func_t)(sMathInterpolateInput input_x, sMathInterpolateInput input_y,
    uint32_t y_size, const float (*table)[]);

#define ENRICHMENT_LOAD_STATES_COUNT     (64)
#define ASYNC_INJECTION_FIFO_SIZE   (32)
#define FAN_HIGH_SWITCH_TIME        (500 * 1000)
#define FUEL_PUMP_TIMEOUT           (1 * 1000 * 1000)
#define FAN_TIMEOUT                 (3 * 1000 * 1000)

typedef enum {
  KnockStatusOk = 0,
  KnockStatusLowNoise = 1,
  KnockStatusDedonation = 2,
  KnockStatusStrongDedonation = 4,
  KnockStatusContinuousDedonation = 8,
}eKnockStatus;

typedef struct {
    union {
        struct {
            HAL_StatusTypeDef Load : 2;
            HAL_StatusTypeDef Save : 2;
            HAL_StatusTypeDef Init : 2;
        }Struct;
        uint8_t Byte;
    }Flash;
    union {
        struct {
            HAL_StatusTypeDef CriticalSave : 2;
            HAL_StatusTypeDef CriticalLoad : 2;
            HAL_StatusTypeDef CorrsSave : 2;
            HAL_StatusTypeDef CorrsLoad : 2;
        }Struct;
        uint8_t Byte;
    }Bkpsram;
    union {
        struct {
            HAL_StatusTypeDef Map : 2;
            HAL_StatusTypeDef Knock : 2;
            HAL_StatusTypeDef Csps : 2;
            HAL_StatusTypeDef Tsps : 2;
            HAL_StatusTypeDef AirTemp : 2;
            HAL_StatusTypeDef EngineTemp : 2;
            HAL_StatusTypeDef ThrottlePos : 2;
            HAL_StatusTypeDef ReferenceVoltage : 2;
            HAL_StatusTypeDef PowerVoltage : 2;
            HAL_StatusTypeDef Lambda : 2;
        }Struct;
        uint32_t Dword;
    }Sensors;
    HAL_StatusTypeDef OutputStatus;
    sOutputDiagnostic OutputDiagnostic;
    HAL_StatusTypeDef IdleValvePosition;
    HAL_StatusTypeDef O2Status;
    HAL_StatusTypeDef O2TemperatureStatus;
    HAL_StatusTypeDef O2HeaterStatus;
    HAL_StatusTypeDef AdcStatus;
    HAL_StatusTypeDef CanInitStatus;
    HAL_StatusTypeDef CanTestStatus;
    HAL_StatusTypeDef KlineProtocolStatus;
    HAL_StatusTypeDef KlineLoopbackStatus;
    sO2Diagnostic O2Diagnostic;

    struct {
        eKnockStatus GeneralStatus;
        uint32_t DetonationLast;
        uint32_t StrongDetonationLast;
        uint32_t LowNoiseLast;
        uint32_t DetonationCount;
        uint32_t DetonationCountCy[ECU_CYLINDERS_COUNT];
        float Voltages[ECU_CYLINDERS_COUNT];
        float VoltagesLpf[ECU_CYLINDERS_COUNT];
        float Denoised[ECU_CYLINDERS_COUNT];
        float Detonates[ECU_CYLINDERS_COUNT];
        float Advances[ECU_CYLINDERS_COUNT];
        uint32_t LastTime[ECU_CYLINDERS_COUNT];
        uint32_t Period[ECU_CYLINDERS_COUNT];
        float Voltage;
        float Filtered;
        float Detonate;
        float StatusVoltage;
        float StatusFiltered;
        float StatusDetonate;
        float Advance;
        float DetonationCountPerSecond;
        float DetonationRelation;
        float AdaptationDetonate;
        uint8_t Updated[ECU_CYLINDERS_COUNT];
        uint8_t UpdatedInternally[ECU_CYLINDERS_COUNT];
        uint8_t UpdatedAdaptation[ECU_CYLINDERS_COUNT];
    }Knock;
    struct {
        uint8_t is_error;
        uint32_t error_time;
        uint32_t error_last;
    }InjectionUnderflow;
    struct {
        uint8_t is_error;
        uint32_t error_time;
        uint32_t error_last;
    }MapTpsRelation;
    struct {
        uint8_t is_error;
        uint32_t error_time;
        uint32_t error_last;
    }LeanMixture;
    struct {
        uint8_t is_error;
        uint32_t error_time;
        uint32_t error_last;
    }RichMixture;
    struct {
        uint8_t is_error;
        uint32_t error_time;
        uint32_t error_last;
    }LeanIdleMixture;
    struct {
        uint8_t is_error;
        uint32_t error_time;
        uint32_t error_last;
    }RichIdleMixture;
    struct {
        uint8_t is_error;
        uint8_t is_running;
        uint32_t run_time;
        uint32_t error_time;
        uint32_t error_started;
    }OilPressure;
    struct {
        uint8_t is_error;
        uint8_t is_running;
        uint32_t run_time;
    }BatteryCharge;
    struct {
        uint8_t sync_error;
        uint32_t sync_error_time;
        uint32_t sync_error_last;
    }Tsps;
}sStatus;

typedef volatile struct {
    uint8_t savereq;
    uint8_t loadreq;
    uint8_t issaving;
    uint8_t isloading;
    uint8_t lock;
    uint8_t critical_lock;

    eTransChannels savereqsrc;
    eTransChannels loadreqsrc;
}sMem;

typedef volatile struct {
    uint8_t Processing;
    uint8_t Cutting;
    uint8_t FuelCutoff;
}sCutoff;

typedef volatile struct {
    uint8_t Thresholded;
    uint8_t Shifting;
    uint8_t Tilled;

}sShift;

typedef volatile struct {
    uint8_t IgnitionEnabled;
    uint8_t InjectionEnabled;
    uint16_t Count;
    uint32_t Period;
    uint32_t IgnitionPulse;
    uint32_t InjectionPulse;
    uint32_t StartedTime;
    uint16_t CompletedCount;
    uint8_t InjectionTriggered;
} sIgnitionInjectionTest;

union {
    struct {
        uint8_t is_not_running : 1;
        uint8_t is_idle : 1;
        uint8_t is_enrichment : 1;
        uint8_t is_fuel_cutoff : 1;
        uint8_t is_use_lambda : 1;
        uint8_t is_knock_zone : 1;
        uint8_t is_adsorber : 1;
        uint8_t is_lambda_teach : 1;

        uint8_t not_used1 : 1;
        uint8_t is_idle_last : 1;
        uint8_t is_idle_lock_last : 1;
        uint8_t is_knock_zone_last : 1;
        uint8_t is_adsorber_last : 1;
        uint8_t is_knock : 1;
        uint8_t is_lambda_last : 1;
        uint8_t is_lambda_now : 1;
    } Bits;
    struct {
        uint8_t byte[2];
    } Bytes;
} gDiagWorkingMode;

union {
    struct {
        uint8_t csps_error : 1;
        uint8_t not_used1 : 1;
        uint8_t eeprom_error : 1;
        uint8_t lambda_heater : 1;
        uint8_t tsps_error : 1;
        uint8_t reset_error : 1;
        uint8_t ram_error : 1;
        uint8_t flash_error : 1;

        uint8_t low_voltage : 1;
        uint8_t not_used2 : 1;
        uint8_t not_used3 : 1;
        uint8_t engine_temp_low : 1;
        uint8_t lambda_low : 1;
        uint8_t tps_low : 1;
        uint8_t maf_low : 1;
        uint8_t low_noise : 1;

        uint8_t high_voltage : 1;
        uint8_t not_used4 : 1;
        uint8_t not_used5 : 1;
        uint8_t engine_temp_high : 1;
        uint8_t lambda_high : 1;
        uint8_t tps_high : 1;
        uint8_t maf_high : 1;
        uint8_t high_noise : 1;

        uint8_t knock_error : 1;
        uint8_t no_immo : 1;
        uint8_t not_used6 : 1;
        uint8_t lambda_no_activity : 1;
        uint8_t lambda_no_on_lean : 1;
        uint8_t lambda_no_on_rich : 1;
        uint8_t speed_error : 1;
        uint8_t idle_error : 1;

    } Bits;
    struct {
        uint8_t byte[4];
    } Bytes;
} gDiagErrors;

struct {
    volatile uint32_t RequestFillLast;
    volatile uint32_t ParametersAcceptLast;
#if defined(PRESSURE_ACCEPTION_FEATURE) && PRESSURE_ACCEPTION_FEATURE > 0
    volatile float MapAcceptValue;
    volatile float TpsAcceptValue;
    volatile uint32_t MapAcceptLast;
#endif
    uint32_t RunningTime;
    uint32_t LastRunned;

    uint8_t PhasedInjection;
    uint8_t EnrichmentTriggered;

    volatile float InjectionDensityPressure;
}gLocalParams;

static GPIO_TypeDef * const gIgnPorts[ECU_CYLINDERS_COUNT] = { IGN_1_GPIO_Port, IGN_2_GPIO_Port, IGN_3_GPIO_Port, IGN_4_GPIO_Port };
static const uint16_t gIgnPins[ECU_CYLINDERS_COUNT] = { IGN_1_Pin, IGN_2_Pin, IGN_3_Pin, IGN_4_Pin };

static GPIO_TypeDef * const gInjPorts[ECU_CYLINDERS_COUNT] = { INJ_1_GPIO_Port, INJ_2_GPIO_Port, INJ_3_GPIO_Port, INJ_4_GPIO_Port };
static const uint16_t gInjPins[ECU_CYLINDERS_COUNT] = { INJ_1_Pin, INJ_2_Pin, INJ_3_Pin, INJ_4_Pin };

static GPIO_TypeDef * const gInjChPorts[2] = { INJ_CH1_GPIO_Port, INJ_CH2_GPIO_Port };
static const uint16_t gInjChPins[2] = { INJ_CH1_Pin, INJ_CH2_Pin};

static RTC_HandleTypeDef *hrtc = NULL;
static sEcuTable gEcuTable[TABLE_SETUPS_MAX];
static sEcuParams gEcuParams;
static sEcuCorrections gEcuCorrections;
static sEcuCorrectionsProgress gEcuCorrectionsProgress;
static sEcuCriticalBackup gEcuCriticalBackup;
static uint8_t volatile gStatusReset = 0;
static sStatus gStatus = {{{0}}};
static sParameters gParameters = {0};
static sParameters gSharedParameters = {0};
static sForceParameters gForceParameters = {0};
static sIgnitionInjectionTest gIITest = {0};
static uint8_t gCheckBitmap[CHECK_BITMAP_SIZE] = {0};
static sProFIFO fifoAsyncInjection = {0};
static uint32_t fifoAsyncInjectionBuffer[ASYNC_INJECTION_FIFO_SIZE] = {0};
static float enrichmentLoadStates[ENRICHMENT_LOAD_STATES_COUNT] = {0};

static uint8_t gEcuImmoStartAllowed = 0;
static uint8_t gEcuIgnStartAllowed = 0;

volatile static uint32_t gSpecificParametersOverflow = 0;
volatile static uint32_t gSpecificParametersReadPointer = 0;
volatile static uint32_t gSpecificParametersWritePointer = 0;
volatile static uint8_t gSpecificParametersConfigured = 0;
volatile static uint8_t gSpecificParametersPeriod = 0;
static uint32_t gSpecificParametersAddrs[SPECIFIC_PARAMETERS_ARRAY_MAX_ITEMS] = {0};
static uDword gSpecificParametersArray[SPECIFIC_PARAMETERS_ARRAY_MAX_ITEMS][SPECIFIC_PARAMETERS_ARRAY_POINTS];

static volatile uint8_t gEcuInitialized = 0;
static volatile uint8_t gEcuIsError = 0;
static uint8_t gCanTestStarted = 0;

static volatile int8_t gCriticalStatus = 0;
static volatile int8_t gBackupStatus = 0;

static sMathPid gPidIdleValveAirFlow = {0};
static sMathPid gPidIdleValveRpm = {0};
static sMathPid gPidIdleIgnition = {0};
static sMathPid gPidShortTermCorr = {0};

static const float gKnockDetectStartAngle = -100;
static const float gKnockDetectEndAngle = 70;

static sDrag Drag = {0};
static sMem Mem = {0};
static sCutoff Cutoff = {0};
static sShift Shift = {0};

static enum {
  PhaseDetectDisabled = 0,
  PhaseDetectByDisabling,
  PhaseDetectByFueling
} gPhaseDetectMethod = PhaseDetectByDisabling;

static volatile uint8_t gPhaseDetectCompleted = 0;
static uint8_t gPhaseDetectActive = 0;
static float gPhaseDetectAccelerations[ECU_CYLINDERS_COUNT][ACCELERATION_POINTS_COUNT] = {{0}};
static uint8_t gPhaseDetectAccelerationsCount[ECU_CYLINDERS_COUNT] = {0};

static volatile uint8_t gIgnCanShutdown = 0;
#ifndef SIMULATION
static volatile HAL_StatusTypeDef gIgnState = GPIO_PIN_SET;
static volatile uint8_t gIgnShutdownReady = 0;

static int8_t ecu_shutdown_process(void);
#endif

static int8_t ecu_can_process_message(const sCanMessage *message);

static void ecu_async_injection_init(void)
{
  protInit(&fifoAsyncInjection, fifoAsyncInjectionBuffer, sizeof(fifoAsyncInjectionBuffer[0]), ITEMSOF(fifoAsyncInjectionBuffer));
}

static void ecu_async_injection_push(uint32_t time)
{
  if(time > 0) {
    protPush(&fifoAsyncInjection, &time);
  }
}

static uint8_t ecu_async_injection_pull(uint32_t *p_time)
{
  *p_time = 0;
  return protPull(&fifoAsyncInjection, p_time);
}

static uint8_t ecu_async_injection_check(void)
{
  return protGetSize(&fifoAsyncInjection) > 0;
}

static uint8_t ecu_get_table(void)
{
  return gParameters.CurrentTable;
}

static RTC_TimeTypeDef sTime = {0};
static RTC_DateTypeDef sDate = {0};

static HAL_StatusTypeDef ecu_rtc_reset(void)
{
  HAL_StatusTypeDef status = HAL_OK;

  if(!hrtc)
    return HAL_ERROR;

  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.SubSeconds = 0;
  sTime.TimeFormat = RTC_HOURFORMAT12_AM;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  status |= HAL_RTC_SetTime(hrtc, &sTime, FORMAT_BIN);

  sDate.WeekDay = RTC_WEEKDAY_SATURDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;
  status |= HAL_RTC_SetDate(hrtc, &sDate, FORMAT_BIN);

  return status;
}

static HAL_StatusTypeDef ecu_rtc_get_time(uint32_t *p_time)
{
  uint32_t seconds_since_reset = 0;
  HAL_StatusTypeDef status = HAL_OK;

  if(!hrtc)
    return HAL_ERROR;

  status |= HAL_RTC_GetTime(hrtc, &sTime, FORMAT_BIN);
  status |= HAL_RTC_GetDate(hrtc, &sDate, FORMAT_BIN);

  /* No need to return precise timestamp since reset */
  if(status == HAL_OK) {
    seconds_since_reset += sDate.Year;
    seconds_since_reset *= 12;
    seconds_since_reset += sDate.Month - 1;
    seconds_since_reset *= 31;
    seconds_since_reset += sDate.Date - 1;

    seconds_since_reset *= 24;
    seconds_since_reset += sTime.Hours;
    seconds_since_reset *= 60;
    seconds_since_reset += sTime.Minutes;
    seconds_since_reset *= 60;
    seconds_since_reset += sTime.Seconds;

    if(p_time)
      *p_time = seconds_since_reset;
  }

  return status;
}

static void ecu_set_table(uint8_t number)
{
  gParameters.SwitchPosition = number;
}

static void ecu_config_init(void)
{
  int8_t status;

  memset(gEcuTable, 0, sizeof(gEcuTable));
  memset(&gEcuParams, 0, sizeof(gEcuParams));
  memset(&gEcuCorrections, 0, sizeof(gEcuCorrections));
  memset(&gEcuCriticalBackup, 0, sizeof(gEcuCriticalBackup));
  memset(&gStatus, 0, sizeof(gStatus));

  gStatus.Flash.Struct.Init = config_init();

  if(gStatus.Flash.Struct.Init == HAL_OK) {
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
  } else {
    config_default_params(&gEcuParams);
    gStatus.Flash.Struct.Load = HAL_ERROR;
    gStatus.Flash.Struct.Save = HAL_ERROR;
    for(int i = 0; i < TABLE_SETUPS_MAX; i++)
      config_default_table(&gEcuTable[i], i);
  }

  while(!(status = config_load_corrections(&gEcuCorrections))) {}
  if(status < 0) {
    gStatus.Bkpsram.Struct.CorrsLoad = HAL_ERROR;
    config_default_corrections(&gEcuCorrections);
    while(!(status = config_save_corrections(&gEcuCorrections))) {}
    if(status < 0) {
      gStatus.Bkpsram.Struct.CorrsSave = HAL_ERROR;
    }
  }

  while(!(status = config_load_critical_backup(&gEcuCriticalBackup))) {}
  if(status < 0) {
    gStatus.Bkpsram.Struct.CriticalLoad = HAL_ERROR;
    config_default_critical_backup(&gEcuCriticalBackup);
    while(!(status = config_save_critical_backup(&gEcuCriticalBackup))) {}
    if(status < 0) {
      gStatus.Bkpsram.Struct.CriticalSave = HAL_ERROR;
    }
  }
}

static float ecu_get_air_density(float pressure, float temperature)
{
  //Output in mg/cc
  const float M = 29.0f;
  const float R = 8314.46261815324f;
  float mg_cc3 = (pressure * M) / (R * (temperature + 273.15f));
  return mg_cc3;
}

static void ecu_update_current_table(void)
{
  uint32_t table = 0;

  if(gEcuParams.isForceTable) {
      table = gEcuParams.forceTable;
  } else if(gEcuParams.isSwitchByExternal) {
    //TODO:
  } else {
    switch(gParameters.SwitchPosition) {
      case 0 : table = gEcuParams.switchPos0Table; break;
      case 1 : table = gEcuParams.switchPos1Table; break;
      case 2 : table = gEcuParams.switchPos2Table; break;
      default : break;
    }
  }

  gParameters.CurrentTable = table;
}

STATIC_INLINE void ecu_pid_update(uint8_t isidle, float idle_wish_to_rpm_relation)
{
  uint32_t table_number = gParameters.CurrentTable;
  sEcuTable *table = &gEcuTable[table_number];
  sMathInterpolateInput ipIdleWishToRpmRelation = math_interpolate_input(idle_wish_to_rpm_relation, table->idle_pids_rpm_koffs, table->idle_pids_rpm_koffs_count);

  math_pid_set_clamp(&gPidIdleIgnition, table->idle_ign_deviation_min, table->idle_ign_deviation_max);

  if(isidle) {
    math_pid_set_koffs(&gPidIdleValveAirFlow, math_interpolate_1d(ipIdleWishToRpmRelation, table->idle_valve_to_massair_pid_p),
        math_interpolate_1d(ipIdleWishToRpmRelation, table->idle_valve_to_massair_pid_i), math_interpolate_1d(ipIdleWishToRpmRelation, table->idle_valve_to_massair_pid_d));
    math_pid_set_koffs(&gPidIdleValveRpm, math_interpolate_1d(ipIdleWishToRpmRelation, table->idle_valve_to_rpm_pid_p),
        math_interpolate_1d(ipIdleWishToRpmRelation, table->idle_valve_to_rpm_pid_i), math_interpolate_1d(ipIdleWishToRpmRelation, table->idle_valve_to_rpm_pid_d));
    math_pid_set_koffs(&gPidIdleIgnition, math_interpolate_1d(ipIdleWishToRpmRelation, table->idle_ign_to_rpm_pid_p),
        math_interpolate_1d(ipIdleWishToRpmRelation, table->idle_ign_to_rpm_pid_i), math_interpolate_1d(ipIdleWishToRpmRelation, table->idle_ign_to_rpm_pid_d));
  } else {
    math_pid_set_koffs(&gPidIdleValveAirFlow, 0, 0, 0);
    math_pid_set_koffs(&gPidIdleValveRpm, 0, 0, 0);
    math_pid_set_koffs(&gPidIdleIgnition, 0, 0, 0);
  }
  math_pid_set_koffs(&gPidShortTermCorr, table->short_term_corr_pid_p, table->short_term_corr_pid_i, table->short_term_corr_pid_d);
}

static void ecu_pid_init(void)
{
  math_pid_init(&gPidIdleValveAirFlow);
  math_pid_init(&gPidShortTermCorr);
  math_pid_init(&gPidIdleValveRpm);
  math_pid_init(&gPidIdleIgnition);

  math_pid_set_clamp(&gPidIdleValveAirFlow, -IDLE_VALVE_POS_MAX, IDLE_VALVE_POS_MAX);
  math_pid_set_clamp(&gPidIdleValveRpm, -IDLE_VALVE_POS_MAX, IDLE_VALVE_POS_MAX);
  math_pid_set_clamp(&gPidShortTermCorr, -0.25f, 0.25f);

  ecu_pid_update(0, 0);
}

static void ecu_update_shared_parameters(void)
{
  uint8_t running = csps_isrunning();
  uint32_t params_accept_last = gLocalParams.ParametersAcceptLast;
  uint32_t now = Delay_Tick;

  if(!running || params_accept_last == 0 || DelayDiff(now, params_accept_last) > 100000) {
    memcpy(&gSharedParameters, &gParameters, sizeof(sParameters));
    gLocalParams.ParametersAcceptLast = now;
  } else {
    gSharedParameters.AdcAirTemp = gParameters.AdcAirTemp;
    gSharedParameters.AdcEngineTemp = gParameters.AdcEngineTemp;
    gSharedParameters.AdcKnockVoltage = gParameters.AdcKnockVoltage;
    gSharedParameters.AdcLambdaUA = gParameters.AdcLambdaUA;
    gSharedParameters.AdcLambdaUR = gParameters.AdcLambdaUR;
    gSharedParameters.AdcManifoldAirPressure = gParameters.AdcManifoldAirPressure;
    gSharedParameters.AdcPowerVoltage = gParameters.AdcPowerVoltage;
    gSharedParameters.AdcReferenceVoltage = gParameters.AdcReferenceVoltage;
    gSharedParameters.AdcThrottlePosition = gParameters.AdcThrottlePosition;
    gSharedParameters.CylinderIgnitionBitmask = gParameters.CylinderIgnitionBitmask;
    gSharedParameters.CylinderInjectionBitmask = gParameters.CylinderInjectionBitmask;
  }

}

#ifdef SIMULATION
float gDebugMap = 50000;
float gDebugAirTemp = 30.0f;
float gDebugEngineTemp = 90.0f;
float gDebugThrottle = 0;
float gDebugReferenceVoltage = 5.1f;
float gDebugPowerVoltage = 14.4f;
#endif

static void ecu_update(void)
{
  static uint32_t adaptation_last = 0;
  static uint32_t phased_last = 0;
  static uint32_t running_time_last = 0;
  static float fuel_consumed = 0;
  static float km_driven = 0;
  static uint32_t updated_last = 0;
  static float idle_advance_correction = 0;
  static float idle_valve_pos_correction = 0;
  static uint8_t is_cold_start = 1;
  static float cold_start_idle_temperature = 0.0f;
#if defined(PRESSURE_ACCEPTION_FEATURE) && PRESSURE_ACCEPTION_FEATURE > 0
  uint32_t params_map_accept_last = gLocalParams.MapAcceptLast;
#endif
  uint32_t table_number = gParameters.CurrentTable;
  sEcuTable *table = &gEcuTable[table_number];
  uint8_t calibration = gEcuParams.performAdaptation;
  uint8_t calibration_permitted_to_perform = 0;
  uint8_t idle_calibration = gEcuParams.performIdleAdaptation;
  uint32_t now = Delay_Tick;
  uint32_t hal_now = HAL_GetTick();
  float adapt_diff = DelayDiff(now, adaptation_last);
  float diff = DelayDiff(now, updated_last);
  float diff_sec = diff * 0.000001f;
  sMathInterpolateInput ipRpm = {0};
  sMathInterpolateInput ipIdleRpm = {0};
  sMathInterpolateInput ipDensity = {0};
  sMathInterpolateInput ipEngineTemp = {0};
  sMathInterpolateInput ipColdStartTemp = {0};
  sMathInterpolateInput ipAirTemp = {0};
  sMathInterpolateInput ipSpeed = {0};
  sMathInterpolateInput ipThr = {0};
  sMathInterpolateInput ipVoltages = {0};
  sMathInterpolateInput ipEnrLoadStart = {0};
  sMathInterpolateInput ipEnrLoadDeriv = {0};

  sMathInterpolateInput ipIdleFillingRpm = {0};
  sMathInterpolateInput ipIdleFillingDensity = {0};

  sMathInterpolateInput ipFilling = {0};

  static uint32_t short_term_last = 0;
  static uint32_t rotates_last = 0;
  uint32_t request_fill_last = gLocalParams.RequestFillLast;

  float rpm;
  float pressure;
  float density_when_injected;
  float fuel_ratio;
  float lambda_value;
  float lambda_temperature;
  float lambda_heatervoltage;
  float lambda_temperaturevoltage;
  float air_temp;
  float engine_temp;
  float throttle;
  float power_voltage;
  float reference_voltage;
  float speed;
  float acceleration;
  float idle_valve_position;
  float idle_valve_econ_position;
  float uspa;
  float period;
  float period_half;
  float cold_start_idle_mult;
  float cold_start_idle_time;
  float cold_start_idle_corr;

  float fuel_ratio_diff;
  float wish_fuel_ratio;
  float idle_fill_correction_density;
  float idle_filling;
  float filling;
  float pressure_from_throttle;
  float effective_volume;
  float ignition_advance;
  float start_ignition_advance;
  float ignition_time;
  float injector_lag;
  float injector_lag_mult;
  float cycle_air_flow;
  float cycle_air_flow_injection;
  float cycle_air_flow_injection_startup;
  float mass_air_flow;
  float injection_time;

  float min_injection_time;
  float injection_phase_duration;
  float injection_start_mult;
  float knock_threshold;
  float air_density;
  float filling_relative;
  float fuel_flow_per_us;
  float knock_noise_level;
  static float knock_zone = 0;
  static float knock_low_noise_state = 0;
  float knock_low_noise_lpf = 0;
  float knock_cy_level_diff;
  static float knock_cy_level_multiplier[ECU_CYLINDERS_COUNT] = {0};
  static float knock_cy_level_multiplier_correction[ECU_CYLINDERS_COUNT] = {0};

  float min, max;
  static uint8_t ventilation_flag = 0;
  static uint32_t prev_halfturns = 0;
  uint32_t halfturns;
  uint32_t halfturns_performed = 0;
  float running_time = 0;
  static float injection_phase = 100;
  static float short_term_correction = 0.0f;
  float short_term_correction_pid = 0.0f;
  float injection_phase_start;
  float injection_phase_table;
  float injection_phase_lpf;
  float fuel_amount_per_cycle;

  static float enrichment_phase_state = 0;
  static float enrichment_ignition_state = 0;
  float enrichment_injection_phase_decay_time;
  float enrichment_injection_phase;
  float enrichment_ign_corr;
  float enrichment_temp_mult;
  float enrichment_result;
  float enrichment_rate;
  static float enrichment_amount_sync = 0;
  static float enrichment_amount_async = 0;
  float enrichment_async_time;

  int32_t enrichment_load_type;
  float enrichment_load_dead_band;
  float enrichment_accel_dead_band;
  float enrichment_ign_corr_decay_time;
  float enrichment_detect_duration;
  float enrichment_async_pulses_divider;
  float enrichment_async_period;

  float enrichment_load_value_start = 0;
  static float enrichment_load_value_start_accept = 0;
  float enrichment_load_value = -1;
  static float enrichment_load_derivative_final = 0;
  static float enrichment_load_derivative_accept = 0;
  static float enrichment_time_pass = 0;
  static uint32_t enrichment_async_last = 0;
  float enrichment_load_diff;
  float enrichment_load_derivative;
  uint32_t enrichment_load_values_count = 0;
  uint32_t enrichment_load_values_divider = 0;
  static uint32_t enrichment_load_values_counter = 0;
  float enrichment_lpf;
  static uint8_t enrichment_triggered = 0;
  uint8_t enrichment_triggered_once = 0;
  static uint8_t enrichment_triggered_async = 0;
  uint8_t enrichment_post_injection_enabled;
  static uint32_t enrichment_post_cycles = 0;
  static uint32_t idle_accelerate_post_cycles = 0;

  float warmup_mixture;
  float warmup_mix_koff;
  float warmup_mix_corr;
  float air_temp_mix_corr;
  float air_temp_ign_corr;
  float engine_temp_mix_corr;
  float engine_temp_ign_corr;

  float fuel_pressure;
  float fuel_abs_pressure;
  float fuel_consumption_per_distance;
  float fuel_consumption_hourly;
  float km_drive = 0;
  float fuel_consumption = 0;

  float idle_rpm_pid_act_1;
  float idle_rpm_pid_act_2;
  float idle_reg_rpm_1;
  float idle_reg_rpm_2;
  float idle_wish_rpm;
  float idle_wish_massair;
  float idle_wish_ignition;
  float idle_wish_ignition_table;
  float idle_wish_ignition_static;
  float idle_rpm_shift;
  float idle_table_valve_pos;
  float idle_wish_valve_pos;
  float injection_dutycycle;

  float calib_cur_progress;
  float percentage;
  float filling_diff;
  float map_diff_thr;
  float lpf_calculation;
  float fill_correction_density;
  float map_correction_thr;
  float idle_valve_pos_adaptation;
  float idle_valve_pos_dif;
  float ignition_correction;
  float detonation_count_table;
  float ignition_corr_final;
  float fan_ign_corr;
  float fan_air_corr;
  float wish_fault_rpm;
  float tsps_rel_pos = 0;
  float tsps_desync_thr;
  float start_async_filling;
  float start_large_filling;
  float start_small_filling;
  float start_filling_time;
  float start_filling_mult;
  float async_flow_per_cycle;
  float start_async_time;
  float idle_wish_to_rpm_relation;
  float idle_ignition_time_by_tps;
  float idle_ignition_time_by_tps_lpf;
  float idle_econ_delay;
  float start_econ_delay;
  static float idle_econ_time = 0;

  uint8_t econ_flag;
  uint8_t enrichment_async_enabled;
  uint8_t enrichment_sync_enabled;

  uint8_t use_idle_filling;

  static uint8_t idle_rpm_flag = 0;
  float idle_rpm_flag_koff = 0;
  static float idle_rpm_flag_value = 0;
  static float idle_ignition_time_by_tps_value = 0;
  static uint8_t was_rotating = 0;
  static uint8_t was_found = 0;
  static uint8_t was_start_async = 1;
  static uint32_t start_halfturns = 0;
  HAL_StatusTypeDef knock_status;
  HAL_StatusTypeDef injector_status;
  uint32_t start_large_count;
  uint8_t rotates;
  uint8_t found;
  uint8_t running;
  uint8_t phased;
  uint8_t idle_flag;
  static uint8_t was_idle = 0;
  uint8_t throttle_idle_flag;
  uint8_t idle_corr_flag;
  uint8_t o2_valid = 0;
  uint8_t phased_mode = gEcuParams.phasedMode;
  uint8_t shift_processing = Shift.Shifting;
  uint8_t cutoff_processing = Cutoff.Processing;
  static uint8_t knock_running = 0;
  static float knock_running_time = 0;
  sCspsData csps = csps_data();
  sO2Status o2_data = sens_get_o2_status();

  if(!now)
    now++;

  ecu_update_shared_parameters();

  idle_valve_position = out_get_idle_valve();

  math_interpolate_2d_set_func_t corr_math_interpolate_2d_set_func = math_interpolate_2d_set;
  math_interpolate_2d_func_t corr_math_interpolate_2d_func = math_interpolate_2d;

  if(calibration == 2) {
    corr_math_interpolate_2d_func = math_interpolate_2d_point;
    corr_math_interpolate_2d_set_func = math_interpolate_2d_set_point;
  }

  gStatus.AdcStatus = sens_get_adc_status();

  halfturns = csps_gethalfturns();
  found = csps_isfound();
  running = csps_isrunning();
  rotates = csps_isrotates();
  phased = csps_isphased(csps);
  uspa = csps_getuspa(csps);
  period = csps_getperiod(csps);
  period_half = period * 0.5f;
  fuel_pressure = table->fuel_pressure;

  gStatus.Sensors.Struct.Csps = csps_iserror() == 0 ? HAL_OK : HAL_ERROR;
  rpm = csps_getrpm(csps);
  speed = speed_getspeed();
  acceleration = speed_getacceleration();
  knock_status = Knock_GetStatus();

  if(rotates)
    rotates_last = now;

  if(running) {
    if(DelayDiff(now, running_time_last) > 1000000) {
      running_time_last = now;
      gLocalParams.RunningTime++;
    }
  } else {
    running_time_last = now;
  }

  running_time = gLocalParams.RunningTime;
  if(running)
    running_time += (float)DelayDiff(now, running_time_last) * 0.000001f;

  while(halfturns != prev_halfturns) {
    halfturns_performed++;
    prev_halfturns++;
  }

#ifdef SIMULATION
  pressure = gDebugMap;
  air_temp = gDebugAirTemp;
  engine_temp = gDebugEngineTemp;
  throttle = gDebugThrottle;
  reference_voltage = gDebugReferenceVoltage;
  power_voltage = gDebugPowerVoltage;
#else
  gStatus.Sensors.Struct.PowerVoltage = sens_get_power_voltage(&power_voltage);
  gStatus.Sensors.Struct.ReferenceVoltage = sens_get_reference_voltage(&reference_voltage);
  gStatus.Sensors.Struct.ThrottlePos = sens_get_throttle_position(&throttle);
  gStatus.Sensors.Struct.Map = sens_get_map(&pressure);
  gStatus.Sensors.Struct.AirTemp = sens_get_air_temperature(&air_temp);
  gStatus.Sensors.Struct.EngineTemp = sens_get_engine_temperature(&engine_temp);
#endif

  if(gStatus.OutputDiagnostic.Injectors.Availability != HAL_OK || gStatus.OutputDiagnostic.Injectors.Diagnostic.Byte != HAL_OK) {
    injector_status = HAL_ERROR;
  } else {
    injector_status = HAL_OK;
  }

  density_when_injected = gLocalParams.InjectionDensityPressure;

  if(found != was_found) {
    was_found = found;
    if(found) {
      was_start_async = 0;
    }
  }

  if(rotates != was_rotating) {
    was_rotating = rotates;
  }

  if(!rotates && DelayDiff(now, rotates_last) > 3000000) {
    if(pressure < 70000 && (idle_valve_position > 10 || throttle > 2.0f))
      gStatus.Sensors.Struct.Map = HAL_ERROR;
  }

#if defined(PRESSURE_ACCEPTION_FEATURE) && PRESSURE_ACCEPTION_FEATURE > 0
  if(gStatus.Sensors.Struct.Map == HAL_OK && running) {
    if(DelayDiff(now, params_map_accept_last) < 100000) {
      pressure = gLocalParams.MapAcceptValue;
    }
  } else {
    gLocalParams.TpsAcceptValue = throttle;
    gLocalParams.MapAcceptValue = pressure;
    gLocalParams.MapAcceptLast = now;
  }
#endif

  ipRpm = math_interpolate_input(rpm, table->rotates, table->rotates_count);

  tsps_rel_pos = csps_gettspsrelpos() - math_interpolate_1d(ipRpm, table->tsps_relative_pos);
  tsps_desync_thr = fabsf(math_interpolate_1d(ipRpm, table->tsps_desync_thr));

  if(gStatus.Sensors.Struct.EngineTemp != HAL_OK)
    engine_temp = 0.0f;
  if(gStatus.Sensors.Struct.AirTemp != HAL_OK)
    air_temp = 10.0f;

  fuel_ratio = table->fuel_afr;
  lambda_value = 1.0f;
  lambda_temperature = 0;
  lambda_temperaturevoltage = 0;
  lambda_heatervoltage = 0;
  if(gEcuParams.useLambdaSensor) {
    gStatus.Sensors.Struct.Lambda = sens_get_o2_labmda(&o2_data, &lambda_value, &o2_valid);
    if(gStatus.Sensors.Struct.Lambda == HAL_OK) {
      sens_get_o2_temperature(&o2_data, &lambda_temperature);
    }
    sens_get_o2_temperaturevoltage(&o2_data, &lambda_temperaturevoltage);
    sens_get_o2_heatervoltage(&o2_data, &lambda_heatervoltage);
    if(o2_valid) {
      fuel_ratio = lambda_value * table->fuel_afr;
    }
  } else {
    gStatus.Sensors.Struct.Lambda = HAL_OK;
  }

  if(phased_mode != PhasedModeDisabled && phased) {
    enrichment_async_enabled = table->enrichment_ph_async_enabled;
    enrichment_sync_enabled = table->enrichment_ph_sync_enabled;
    enrichment_post_injection_enabled = table->enrichment_ph_post_injection_enabled;
  } else  {
    enrichment_async_enabled = table->enrichment_pp_async_enabled;
    enrichment_sync_enabled = table->enrichment_pp_sync_enabled;
    enrichment_post_injection_enabled = table->enrichment_pp_post_injection_enabled;
  }

  if(running && phased_mode == PhasedModeWithSensor && !phased) {
    if(DelayDiff(now, phased_last) > 500000) {
      gStatus.Sensors.Struct.Tsps = HAL_ERROR;
      phased_last = now;
    }
  } else {
    phased_last = now;
    gStatus.Sensors.Struct.Tsps = HAL_OK;
  }

  if(running && phased_mode == PhasedModeWithSensor && phased && fabsf(tsps_rel_pos) >= tsps_desync_thr) {
    if(!gStatus.Tsps.sync_error) {
      gStatus.Tsps.sync_error_time = 0;
    } else {
      gStatus.Tsps.sync_error_time += HAL_DelayDiff(hal_now, gStatus.Tsps.sync_error_last);
    }
    gStatus.Tsps.sync_error = 1;
  } else {
    gStatus.Tsps.sync_error = 0;
  }
  gStatus.Tsps.sync_error_last = hal_now;

  if(gEcuParams.useKnockSensor) {
    gStatus.Sensors.Struct.Knock = knock_status;
  } else {
    gStatus.Sensors.Struct.Knock = HAL_OK;
  }

  throttle_idle_flag = throttle < 0.2f;
  idle_flag = throttle_idle_flag && running;

  if(was_idle != idle_flag) {
    was_idle = idle_flag;
    idle_accelerate_post_cycles = 0;
  } else {
    for(int ht = 0; idle_accelerate_post_cycles < UINT_MAX && ht < halfturns_performed; ht++) {
      idle_accelerate_post_cycles++;
    }
  }

  if(gStatus.Sensors.Struct.Map == HAL_OK && gStatus.Sensors.Struct.ThrottlePos != HAL_OK) {
    wish_fault_rpm = 1400.0f;
  } else if(gStatus.Sensors.Struct.Map != HAL_OK && gStatus.Sensors.Struct.ThrottlePos == HAL_OK) {
    wish_fault_rpm = 1700.0f;
  } else {
    wish_fault_rpm = 0.0f;
  }
  ipIdleRpm = math_interpolate_input(rpm, table->idle_rotates, table->idle_rotates_count);
  ipThr = math_interpolate_input(throttle, table->throttles, table->throttles_count);

  pressure_from_throttle = math_interpolate_2d(ipRpm, ipThr, TABLE_ROTATES_MAX, table->map_by_thr);
  map_correction_thr = corr_math_interpolate_2d_func(ipRpm, ipThr, TABLE_ROTATES_MAX, gEcuCorrections.map_by_thr);

  pressure_from_throttle *= map_correction_thr + 1.0f;

  if(gStatus.Sensors.Struct.ThrottlePos == HAL_OK && gStatus.Sensors.Struct.Map != HAL_OK) {
    pressure = pressure_from_throttle;
  }

  fuel_abs_pressure = fuel_pressure;
  fuel_flow_per_us = table->injector_performance * 1.66666667e-8f * table->fuel_mass_per_cc; // perf / 60.000.000
  if(table->is_fuel_pressure_const) {
    fuel_abs_pressure += (1.0f - pressure * 0.00001f);
    fuel_flow_per_us *= fast_rsqrt(fuel_pressure / fuel_abs_pressure);
  }

  air_density = ecu_get_air_density(pressure, air_temp);

  ipEngineTemp = math_interpolate_input(engine_temp, table->engine_temps, table->engine_temp_count);
  ipAirTemp = math_interpolate_input(air_temp, table->air_temps, table->air_temp_count);
  ipSpeed = math_interpolate_input(speed, table->speeds, table->speeds_count);
  ipDensity = math_interpolate_input(air_density, table->densities, table->densities_count);
  ipVoltages = math_interpolate_input(power_voltage, table->voltages, table->voltages_count);

  filling = math_interpolate_2d(ipRpm, ipDensity, TABLE_ROTATES_MAX, table->fill_by_density);

  fill_correction_density = corr_math_interpolate_2d_func(ipRpm, ipDensity, TABLE_ROTATES_MAX, gEcuCorrections.fill_by_density);
  idle_fill_correction_density = 0;

  filling *= fill_correction_density + 1.0f;


  use_idle_filling = 0;

  if(table->use_idle_filling && table->idle_filling_rotates_count >= 2 && table->idle_filling_densities_count >= 2) {
    if(rpm > table->idle_filling_rotates[0] && rpm < table->idle_filling_rotates[table->idle_filling_rotates_count - 1] &&
        pressure > table->idle_filling_densities[0] && pressure < table->idle_filling_densities[table->idle_filling_densities_count - 1]) {
      ipIdleFillingRpm = math_interpolate_input(rpm, table->idle_filling_rotates, table->idle_filling_rotates_count);
      ipIdleFillingDensity = math_interpolate_input(pressure, table->idle_filling_densities, table->idle_filling_densities_count);
      idle_filling = math_interpolate_2d(ipIdleFillingRpm, ipIdleFillingDensity, TABLE_ROTATES_MAX, table->idle_filling_by_density);

      idle_fill_correction_density = corr_math_interpolate_2d_func(ipIdleFillingRpm, ipIdleFillingDensity, TABLE_ROTATES_MAX, gEcuCorrections.idle_filling_by_density);
      idle_filling *= idle_fill_correction_density + 1.0f;

      use_idle_filling = 1;
    }
  }

  if(use_idle_filling && idle_flag) {
    filling = idle_filling;
  }

  filling_relative = filling;

  effective_volume = filling * gEcuParams.engineVolume;

  cycle_air_flow = effective_volume * 0.25f * air_density;
  mass_air_flow = rpm * 0.03333333f * cycle_air_flow * 0.001f * 3.6f; // rpm / 60 * 2

  if(is_cold_start) {
    if(running) {
      is_cold_start = 0;
    }
    cold_start_idle_temperature = engine_temp;
  }

  ipColdStartTemp = math_interpolate_input(cold_start_idle_temperature, table->engine_temps, table->engine_temp_count);
  cold_start_idle_corr = math_interpolate_1d(ipColdStartTemp, table->cold_start_idle_corrs);
  cold_start_idle_time = math_interpolate_1d(ipColdStartTemp, table->cold_start_idle_times);
  start_async_filling = math_interpolate_1d(ipColdStartTemp, table->start_async_filling);
  start_large_filling = math_interpolate_1d(ipColdStartTemp, table->start_large_filling);
  start_small_filling = math_interpolate_1d(ipColdStartTemp, table->start_small_filling);
  start_filling_time = math_interpolate_1d(ipColdStartTemp, table->start_filling_time);
  enrichment_temp_mult = math_interpolate_1d(ipEngineTemp, table->enrichment_temp_mult);
  enrichment_injection_phase = math_interpolate_1d(ipRpm, table->enrichment_injection_phase);

  ipFilling = math_interpolate_input(cycle_air_flow, table->fillings, table->fillings_count);

  start_ignition_advance = math_interpolate_1d(ipEngineTemp, table->start_ignition);
  ignition_advance = math_interpolate_2d(ipRpm, ipFilling, TABLE_ROTATES_MAX, table->ignitions);
  idle_ignition_time_by_tps = math_interpolate_1d(ipThr, table->idle_ignition_time_by_tps);
  idle_econ_delay = math_interpolate_1d(ipEngineTemp, table->idle_econ_delay);
  start_econ_delay = math_interpolate_1d(ipEngineTemp, table->start_econ_delay);

  ignition_correction = corr_math_interpolate_2d_func(ipRpm, ipFilling, TABLE_ROTATES_MAX, gEcuCorrections.ignitions);
  air_temp_ign_corr = math_interpolate_2d(ipFilling, ipAirTemp, TABLE_FILLING_MAX, table->air_temp_ign_corr);
  engine_temp_ign_corr = math_interpolate_2d(ipFilling, ipEngineTemp, TABLE_FILLING_MAX, table->engine_temp_ign_corr);

  idle_wish_rpm = math_interpolate_1d(ipEngineTemp, table->idle_wish_rotates);
  idle_wish_massair = math_interpolate_1d(ipEngineTemp, table->idle_wish_massair);
  idle_wish_ignition_static = math_interpolate_1d(ipIdleRpm, table->idle_wish_ignition_static);
  idle_wish_ignition_table = math_interpolate_1d(ipEngineTemp, table->idle_wish_ignition);
  idle_valve_pos_adaptation = math_interpolate_1d(ipEngineTemp, gEcuCorrections.idle_valve_position);

  idle_rpm_shift = math_interpolate_1d(ipSpeed, table->idle_rpm_shift);
  knock_noise_level = math_interpolate_1d(ipRpm, table->knock_noise_level);
  knock_threshold = math_interpolate_1d(ipRpm, table->knock_threshold);

  idle_wish_rpm += idle_rpm_shift;

  if(gForceParameters.Enable.WishIdleRPM)
    idle_wish_rpm = gForceParameters.WishIdleRPM;

  if(idle_wish_rpm < wish_fault_rpm) {
    idle_wish_rpm = wish_fault_rpm;
    idle_wish_massair *= wish_fault_rpm / idle_wish_rpm;
  }

  idle_wish_to_rpm_relation = rpm / idle_wish_rpm;

  idle_rpm_pid_act_1 = math_interpolate_1d(ipEngineTemp, table->idle_rpm_pid_act_1);
  idle_rpm_pid_act_2 = math_interpolate_1d(ipEngineTemp, table->idle_rpm_pid_act_2);
  idle_reg_rpm_1 = idle_wish_rpm * (idle_rpm_pid_act_1 + 1.0f);
  idle_reg_rpm_2 = idle_wish_rpm * (idle_rpm_pid_act_2 + 1.0f);

  if(idle_reg_rpm_1 > idle_reg_rpm_2) {
    idle_reg_rpm_1 = idle_reg_rpm_2 = (idle_reg_rpm_1 + idle_reg_rpm_2) * 0.5f;
  }

  if(!running) {
    idle_rpm_flag = 1;
  } else {
    if(idle_rpm_flag) {
      if(rpm > idle_reg_rpm_2)
        idle_rpm_flag = 0;
    } else {
      if(rpm < idle_reg_rpm_1)
        idle_rpm_flag = 1;
    }
  }

  idle_corr_flag = idle_flag && idle_rpm_flag;
  econ_flag = idle_flag && !idle_rpm_flag;
  if(econ_flag)
    idle_econ_time += diff_sec;
  else idle_econ_time = 0;

  ignition_corr_final = ignition_correction;

  idle_rpm_flag_koff = diff_sec * 0.25f; // 0.25 sec
  idle_rpm_flag_koff = CLAMP(idle_rpm_flag_koff, 0.000001f, 0.90f);
  idle_rpm_flag_value = idle_rpm_flag * idle_rpm_flag_koff + idle_rpm_flag_value * (1.0f - idle_rpm_flag_koff);

  if(idle_ignition_time_by_tps < diff_sec) {
    idle_ignition_time_by_tps_lpf = 1.0f;
  } else {
    idle_ignition_time_by_tps_lpf = diff_sec * (1.0f / idle_ignition_time_by_tps);
    idle_ignition_time_by_tps_lpf = CLAMP(idle_ignition_time_by_tps_lpf, 0.0f, 1.0f);
  }

  if(!running) {
    idle_wish_ignition = start_ignition_advance;
    idle_ignition_time_by_tps_value = 0;
  } else {
    idle_wish_ignition = idle_wish_ignition_static * idle_rpm_flag_value + idle_wish_ignition_table * (1.0f - idle_rpm_flag_value);
    idle_ignition_time_by_tps_value = idle_flag * idle_ignition_time_by_tps_lpf + idle_ignition_time_by_tps_value * (1.0f - idle_ignition_time_by_tps_lpf);
  }

  if(idle_flag) {
    if(out_get_fan(NULL) != GPIO_PIN_RESET) {
      if(out_get_fan_switch(NULL) != GPIO_PIN_RESET) {
        fan_ign_corr = table->idle_ign_fan_high_corr;
        fan_air_corr = table->idle_air_fan_high_corr;
      } else {
        fan_ign_corr = table->idle_ign_fan_low_corr;
        fan_air_corr = table->idle_air_fan_low_corr;
      }

      ignition_corr_final += fan_ign_corr;
      idle_wish_massair += fan_air_corr;

      if(ignition_corr_final > table->idle_ign_deviation_max)
        ignition_corr_final = table->idle_ign_deviation_max;
      else if(ignition_corr_final < table->idle_ign_deviation_min)
        ignition_corr_final = table->idle_ign_deviation_min;

    }
  }

  ignition_advance = idle_wish_ignition * idle_ignition_time_by_tps_value + ignition_advance * (1.0f - idle_ignition_time_by_tps_value);

  ignition_advance += air_temp_ign_corr;
  ignition_advance += engine_temp_ign_corr;
  ignition_advance += ignition_corr_final;

  wish_fuel_ratio = math_interpolate_2d(ipRpm, ipFilling, TABLE_ROTATES_MAX, table->fuel_mixtures);
  injection_phase_table = math_interpolate_2d(ipRpm, ipFilling, TABLE_ROTATES_MAX, table->injection_phase);

  injection_phase_start = math_interpolate_1d(ipEngineTemp, table->start_injection_phase);
  air_temp_mix_corr = math_interpolate_2d(ipFilling, ipAirTemp, TABLE_FILLING_MAX, table->air_temp_mix_corr);
  engine_temp_mix_corr = math_interpolate_2d(ipFilling, ipEngineTemp, TABLE_FILLING_MAX, table->engine_temp_mix_corr);
  injection_phase_lpf = math_interpolate_1d(ipRpm, table->injection_phase_lpf);
  injection_phase_lpf = CLAMP(injection_phase_lpf, 0.01f, 1.00f);

  if(running) {
    if(injection_phase_lpf >= 0.99f) {
      injection_phase = injection_phase_table;
    } else {
      for(int ht = 0; ht < halfturns_performed; ht++) {
        injection_phase = injection_phase * (1.0f - injection_phase_lpf) + injection_phase_table * injection_phase_lpf;
      }
    }
  } else {
    injection_phase = injection_phase_start;
  }

  warmup_mix_corr = math_interpolate_1d(ipEngineTemp, table->warmup_mix_corrs);
  warmup_mix_koff = math_interpolate_1d(ipEngineTemp, table->warmup_mix_koffs);
  if(warmup_mix_koff > 0.003f) {
    warmup_mixture = math_interpolate_1d(ipEngineTemp, table->warmup_mixtures);
    if(warmup_mixture < wish_fuel_ratio) {
      wish_fuel_ratio = warmup_mixture * warmup_mix_koff + wish_fuel_ratio * (1.0f - warmup_mix_koff);
    }
  }

  injector_lag = math_interpolate_1d(ipVoltages, table->injector_lag);
  injector_lag_mult = injector_lag * 1000.0f;

  ignition_time = math_interpolate_1d(ipVoltages, table->ignition_time);
  ignition_time *= math_interpolate_1d(ipRpm, table->ignition_time_rpm_mult);

  if(gForceParameters.Enable.IgnitionPulse)
    ignition_time = gForceParameters.IgnitionPulse;

  if(shift_processing && !cutoff_processing) {
    ignition_advance = gEcuParams.shiftAdvance;
    wish_fuel_ratio = gEcuParams.shiftMixture;
  }

  if(cutoff_processing) {
    ignition_advance = gEcuParams.cutoffAdvance;
    wish_fuel_ratio = gEcuParams.cutoffMixture;
  }

  if(gForceParameters.Enable.WishFuelRatio)
    wish_fuel_ratio = gForceParameters.WishFuelRatio;

  math_pid_set_target(&gPidShortTermCorr, wish_fuel_ratio);

  if(!gEcuParams.useLambdaSensor || !o2_valid) {
    fuel_ratio = wish_fuel_ratio;
    lambda_value = wish_fuel_ratio / table->fuel_afr;
  }

  if(gForceParameters.Enable.WishIdleMassAirFlow)
    idle_wish_massair = gForceParameters.WishIdleMassAirFlow;

  if(!running || !gEcuParams.useLambdaSensor || !o2_valid) {
    short_term_correction = 0.0f;
    math_pid_reset(&gPidShortTermCorr);
  } else if(!econ_flag && !cutoff_processing && !shift_processing) {
    if(gEcuParams.useShortTermCorr && !calibration && !gForceParameters.Enable.InjectionPulse) {
      if(halfturns_performed) {
        for(int i = 0; i < halfturns_performed; i++) {
          short_term_last += 1000;
          short_term_last &= DelayMask;
          short_term_correction_pid = math_pid_update(&gPidShortTermCorr, fuel_ratio, short_term_last);
        }
        short_term_correction = -short_term_correction_pid;
      }
    } else {
      math_pid_reset(&gPidShortTermCorr);
      short_term_correction = 0;
    }

  }

  start_large_count = table->start_large_count;
  injection_start_mult = math_interpolate_1d(ipThr, table->start_tps_corrs);
  econ_flag = econ_flag && gEcuParams.isEconEnabled && (idle_econ_time > idle_econ_delay) && (running_time > start_econ_delay);

  if(econ_flag) {
    idle_accelerate_post_cycles = 0;
  }

  for(int i = 0; i < halfturns_performed; i++) {
    start_halfturns++;
  }
  if(!rotates)
    start_halfturns = 0;

  if(start_halfturns < start_large_count)
    cycle_air_flow_injection_startup = start_large_filling;
  else cycle_air_flow_injection_startup = start_small_filling;

  if(running) {
      cycle_air_flow_injection = cycle_air_flow;
  } else {
    cycle_air_flow_injection = cycle_air_flow_injection_startup;
  }

  fuel_amount_per_cycle = cycle_air_flow_injection * 0.001f / wish_fuel_ratio;
  async_flow_per_cycle = start_async_filling * 0.001f / wish_fuel_ratio;

  start_async_time = async_flow_per_cycle / fuel_flow_per_us;
  start_async_time *= injection_start_mult;
  if(found && !was_start_async) {
    if(start_async_time > 0) {
      start_async_time += injector_lag_mult;
      ecu_async_injection_push(start_async_time);
    }
    was_start_async = 1;
  }

  if(running_time < cold_start_idle_time) {
    cold_start_idle_mult = cold_start_idle_corr * (1.0f - (running_time / cold_start_idle_time));
  } else {
    cold_start_idle_mult = 0.0f;
  }

  if(running_time < start_filling_time && cycle_air_flow_injection_startup > cycle_air_flow_injection) {
    start_filling_mult = ((cycle_air_flow_injection_startup / cycle_air_flow_injection) - 1.0f) * (1.0f - (running_time / start_filling_time));
  } else {
    start_filling_mult = 0.0f;
  }

  injection_time = fuel_amount_per_cycle / fuel_flow_per_us;
  injection_time *= warmup_mix_corr + 1.0f;
  injection_time *= start_filling_mult + 1.0f;
  injection_time *= air_temp_mix_corr + 1.0f;
  injection_time *= engine_temp_mix_corr + 1.0f;
  injection_time *= cold_start_idle_mult + 1.0f;

  enrichment_load_type = table->enrichment_load_type;
  enrichment_load_dead_band = table->enrichment_load_dead_band;
  enrichment_accel_dead_band = table->enrichment_accel_dead_band;
  enrichment_ign_corr_decay_time = table->enrichment_ign_corr_decay_time * 1000.0f;
  enrichment_detect_duration = table->enrichment_detect_duration;
  enrichment_injection_phase_decay_time = table->enrichment_injection_phase_decay_time * 1000.0f;
  enrichment_async_pulses_divider = table->enrichment_async_pulses_divider;
  enrichment_async_pulses_divider = MAX(enrichment_async_pulses_divider, 1);

  enrichment_load_values_divider = 1;
  enrichment_load_values_count = enrichment_detect_duration;
  while(enrichment_load_values_count > ENRICHMENT_LOAD_STATES_COUNT) {
    enrichment_load_values_count /= 2;
    enrichment_load_values_divider *= 2;
  }

  enrichment_detect_duration *= 1000.0f;

  if(enrichment_load_type == 0 && gStatus.Sensors.Struct.ThrottlePos == HAL_OK) enrichment_load_value = throttle;
  else if(enrichment_load_type == 1 && gStatus.Sensors.Struct.Map == HAL_OK) enrichment_load_value = pressure;
  else enrichment_load_type = -1;

  if(enrichment_load_values_count > 4 && enrichment_load_type >= 0) {

    if(++enrichment_load_values_counter >= enrichment_load_values_divider) {
      for(int i = enrichment_load_values_count - 2; i >= 0; i--) {
        enrichmentLoadStates[i + 1] = enrichmentLoadStates[i];
      }
      enrichmentLoadStates[0] = enrichment_load_value;
      enrichment_load_values_counter = 0;
    }

    min = enrichmentLoadStates[enrichment_load_values_count - 1];
    max = enrichmentLoadStates[0];

    enrichment_load_value_start = min;
    enrichment_load_diff = max - min;

    enrichment_lpf = diff / enrichment_detect_duration;
    enrichment_load_derivative = enrichment_load_diff / enrichment_lpf;

    if(enrichment_phase_state > 0 && enrichment_injection_phase_decay_time >= 1000.0f)
      enrichment_phase_state -= diff / enrichment_injection_phase_decay_time;
    else enrichment_phase_state = 0;

    if(enrichment_ignition_state > 0 && enrichment_ign_corr_decay_time >= 1000.0f)
      enrichment_ignition_state -= diff / enrichment_ign_corr_decay_time;
    else enrichment_ignition_state = 0;

    if((!enrichment_triggered && enrichment_load_derivative >= enrichment_load_dead_band) ||
        (enrichment_triggered && (enrichment_load_derivative > enrichment_load_derivative_accept))) {
      enrichment_load_derivative_accept = enrichment_load_derivative;
      enrichment_load_derivative_final = enrichment_load_derivative;
      enrichment_load_value_start_accept = enrichment_load_value_start;
      enrichment_time_pass = 0;
      enrichment_triggered = 1;
      enrichment_ignition_state = 1;
      enrichment_phase_state = 1;
      enrichment_triggered_async = 1;
      enrichment_triggered_once = 1;
      enrichment_async_last = now;
    } else if(enrichment_triggered) {
      enrichment_time_pass += diff_sec;
      enrichment_load_derivative_final = enrichment_load_derivative_accept - enrichment_accel_dead_band * enrichment_time_pass;
      if(enrichment_load_derivative_final <= 0) {
        enrichment_load_derivative_final = 0;
        enrichment_triggered = 0;
      }
    }

    if(enrichment_triggered || !running) {
      enrichment_post_cycles = 0;
    } else {
      for(int ht = 0; enrichment_post_cycles < UINT_MAX && ht < halfturns_performed; ht++) {
        enrichment_post_cycles++;
      }
    }

    ipEnrLoadStart = math_interpolate_input_limit(enrichment_load_value_start_accept, table->enrichment_rate_start_load, table->enrichment_rate_start_load_count);
    ipEnrLoadDeriv = math_interpolate_input_limit(enrichment_load_derivative_final, table->enrichment_rate_load_derivative, table->enrichment_rate_load_derivative_count);

    enrichment_rate = math_interpolate_2d(ipEnrLoadDeriv, ipEnrLoadStart, TABLE_ENRICHMENT_PERCENTS_MAX, table->enrichment_rate);
    enrichment_rate *= enrichment_temp_mult + 1.0f;

    enrichment_ign_corr = math_interpolate_2d(ipEnrLoadStart, ipRpm, TABLE_ENRICHMENT_PERCENTS_MAX, table->enrichment_ign_corr);

    if(enrichment_sync_enabled) {
      enrichment_amount_sync = math_interpolate_1d(ipRpm, table->enrichment_sync_amount);
      enrichment_amount_sync *= enrichment_rate;
    }

    if(enrichment_async_enabled) {
      enrichment_amount_async = math_interpolate_1d(ipRpm, table->enrichment_async_amount);
      enrichment_amount_async *= enrichment_rate;
    }
  } else {
    enrichment_amount_sync = 0;
    enrichment_load_values_counter = 0;
    enrichment_phase_state = 0;
    enrichment_ignition_state = 0;
    enrichment_ign_corr = 0;
    enrichment_triggered_async = 0;
    enrichment_async_last = now;
  }

  enrichment_result = enrichment_amount_sync + enrichment_amount_async;

  if(running && enrichment_async_enabled) {
    enrichment_async_period = period_half / enrichment_async_pulses_divider;
    if(enrichment_amount_async >= 0.001f) {
      enrichment_async_time = 0;
      if(DelayDiff(now, enrichment_async_last) >= enrichment_async_period || enrichment_triggered_async) {
        enrichment_async_time = injection_time;
        if(!enrichment_triggered_async) {
          enrichment_async_time *= ((float)DelayDiff(now, enrichment_async_last) / (float)enrichment_async_period);
        }
        enrichment_triggered_async = 0;
        enrichment_async_last = now;
      }
      if(enrichment_async_time > 0) {
        enrichment_async_time *= enrichment_amount_async;
        enrichment_async_time /= ECU_CYLINDERS_COUNT;
        enrichment_async_time /= enrichment_async_pulses_divider;
        enrichment_async_time += injector_lag_mult;
        ecu_async_injection_push(enrichment_async_time);
      }
    }
  } else {
    enrichment_amount_async = 0;
    enrichment_triggered_async = 0;
    enrichment_async_last = now;
  }

  if(enrichment_phase_state > 0.001f && enrichment_injection_phase > injection_phase) {
    injection_phase = injection_phase * (1.0f - enrichment_phase_state) + enrichment_injection_phase * enrichment_phase_state;
  }
  if(enrichment_ignition_state > 0.001f) {
    ignition_advance += enrichment_ign_corr * enrichment_ignition_state;
  }

  if(gForceParameters.Enable.InjectionPhase)
    injection_phase = gForceParameters.InjectionPhase;

  injection_time *= enrichment_amount_sync + 1.0f;

  if(!running)
    injection_time *= injection_start_mult;

  if(idle_flag)
    injection_time *= gEcuCorrections.idle_correction + 1.0f;
  else
    injection_time *= gEcuCorrections.long_term_correction + 1.0f;
  injection_time *= short_term_correction + 1.0f;

  min_injection_time = gLocalParams.PhasedInjection ? 400 : 800;
  if(injection_time < min_injection_time) {
    injection_time = min_injection_time;
  }

  if(injection_time > 100.0f)
    injection_time += injector_lag_mult;
  else injection_time = 0;

  if(gForceParameters.Enable.InjectionPulse) {
    injection_time = gForceParameters.InjectionPulse;
    if(injection_time > injector_lag_mult) {
      fuel_amount_per_cycle = (injection_time - injector_lag_mult) * fuel_flow_per_us;
      wish_fuel_ratio = cycle_air_flow * 0.001f / fuel_amount_per_cycle;
    } else {
      fuel_amount_per_cycle = 0;
      wish_fuel_ratio = 200.0f;
    }
  }

  if(Cutoff.FuelCutoff)
    injection_time = 0;

  if(gStatus.OilPressure.is_error && gStatus.OilPressure.error_time > 500 && rpm >= gEcuParams.oilPressureCutoffRPM) {
    injection_time = 0;
  } else if(gStatus.OilPressure.is_error && gStatus.OilPressure.error_time > 10000) {
    injection_time = 0;
  }

  math_pid_set_target(&gPidIdleValveAirFlow, idle_wish_massair);
  math_pid_set_target(&gPidIdleValveRpm, idle_wish_rpm);
  math_pid_set_target(&gPidIdleIgnition, idle_wish_rpm);

  idle_valve_econ_position = math_interpolate_1d(ipRpm, table->idle_valve_econ_position);

  if(running) {
    idle_table_valve_pos = math_interpolate_1d(ipEngineTemp, table->idle_valve_position);
    idle_table_valve_pos *= idle_valve_pos_adaptation + 1.0f;
    math_pid_set_clamp(&gPidIdleValveAirFlow, -idle_table_valve_pos, IDLE_VALVE_POS_MAX);
    math_pid_set_clamp(&gPidIdleValveRpm, -idle_table_valve_pos, IDLE_VALVE_POS_MAX);
    idle_advance_correction = math_pid_update(&gPidIdleIgnition, rpm, now);
    idle_valve_pos_correction = math_pid_update(&gPidIdleValveAirFlow, mass_air_flow, now);
    idle_valve_pos_correction += math_pid_update(&gPidIdleValveRpm, rpm, now);
  } else {
    idle_table_valve_pos = math_interpolate_1d(ipEngineTemp, table->start_idle_valve_pos);
  }

  idle_wish_valve_pos = idle_table_valve_pos;

  ecu_pid_update(idle_corr_flag, idle_wish_to_rpm_relation);

  if(!idle_corr_flag) {
    idle_valve_pos_correction = 0;
    idle_advance_correction = 0;
  }

  idle_wish_valve_pos += idle_valve_pos_correction;

  if(throttle_idle_flag && !idle_rpm_flag) {
    idle_wish_valve_pos = idle_valve_econ_position;
  }

  ignition_advance += idle_advance_correction;

  if(gForceParameters.Enable.IgnitionAdvance)
    ignition_advance = gForceParameters.IgnitionAdvance;
  if(gForceParameters.Enable.IgnitionOctane)
    ignition_advance += gForceParameters.IgnitionOctane;

  if(idle_flag && running) {
    if(gForceParameters.Enable.WishIdleIgnitionAdvance) {
      math_pid_reset(&gPidIdleIgnition);
      ignition_advance = gForceParameters.WishIdleIgnitionAdvance;
      idle_advance_correction = 0;
    }
  }

  if(gForceParameters.Enable.WishIdleValvePosition) {
    math_pid_reset(&gPidIdleValveAirFlow);
    math_pid_reset(&gPidIdleValveRpm);
    idle_valve_pos_correction = 0;
    idle_wish_valve_pos = gForceParameters.WishIdleValvePosition;
  }

  injection_dutycycle = injection_time / (period * 2.0f);

  if(injection_dutycycle > 1.0f) {
    injection_time = period * 2.0f;
  }

  fuel_amount_per_cycle = injection_time * fuel_flow_per_us;

  if(idle_wish_valve_pos > IDLE_VALVE_POS_MAX)
    idle_wish_valve_pos = IDLE_VALVE_POS_MAX;
  else if(idle_wish_valve_pos < 0.0f)
    idle_wish_valve_pos = 0.0f;

  if(!gIgnCanShutdown)
	  out_set_idle_valve(roundf(idle_wish_valve_pos));

  gStatus.Knock.Voltage = 0;
  gStatus.Knock.Filtered = 0;
  gStatus.Knock.Detonate = 0;

  if(running) {
    for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
      knock_cy_level_multiplier[i] = math_interpolate_1d(ipRpm, table->knock_cy_level_multiplier[i]);
      knock_cy_level_multiplier_correction[i] = math_interpolate_1d(ipRpm, gEcuCorrections.knock_cy_level_multiplier[i]);
      knock_cy_level_multiplier[i] *= knock_cy_level_multiplier_correction[i] + 1.0f;
      knock_cy_level_multiplier[i] = CLAMP(knock_cy_level_multiplier[i], 0.1f, 5.0f);
      if(gStatus.Knock.Updated[i]) {
        gStatus.Knock.Denoised[i] = (gStatus.Knock.Voltages[i] * knock_cy_level_multiplier[i]) - knock_noise_level;
        gStatus.Knock.Detonates[i] = (gStatus.Knock.Denoised[i] / knock_cy_level_multiplier[i]) - knock_threshold;
        if(gStatus.Knock.Denoised[i] < 0.0f)
          gStatus.Knock.Denoised[i] = 0.0f;
        if(gStatus.Knock.Detonates[i] < 0.0f)
          gStatus.Knock.Detonates[i] = 0.0f;
        gStatus.Knock.Updated[i] = 0;
        gStatus.Knock.UpdatedInternally[i] = 1;
        gStatus.Knock.UpdatedAdaptation[i] = 1;
      }

      if(gStatus.Knock.Voltage < gStatus.Knock.Voltages[i])
        gStatus.Knock.Voltage = gStatus.Knock.Voltages[i];
      if(gStatus.Knock.Filtered < gStatus.Knock.Denoised[i])
        gStatus.Knock.Filtered = gStatus.Knock.Denoised[i];
      if(gStatus.Knock.Detonate < gStatus.Knock.Detonates[i])
        gStatus.Knock.Detonate = gStatus.Knock.Detonates[i];
      if(gStatus.Knock.AdaptationDetonate < gStatus.Knock.Detonate)
        gStatus.Knock.AdaptationDetonate = gStatus.Knock.Detonate;
    }

    if(gStatus.Knock.StatusVoltage < gStatus.Knock.Voltage)
      gStatus.Knock.StatusVoltage = gStatus.Knock.Voltage;
    if(gStatus.Knock.StatusFiltered < gStatus.Knock.Filtered)
      gStatus.Knock.StatusFiltered = gStatus.Knock.Filtered;
    if(gStatus.Knock.StatusDetonate < gStatus.Knock.Detonate)
      gStatus.Knock.StatusDetonate = gStatus.Knock.Detonate;
  }

  if(request_fill_last == 0 || DelayDiff(now, request_fill_last) > 100000) {
    gStatus.Knock.StatusVoltage = gStatus.Knock.Voltage;
    gStatus.Knock.StatusFiltered = gStatus.Knock.Filtered;
    gStatus.Knock.StatusDetonate = gStatus.Knock.Detonate;
    gLocalParams.RequestFillLast = now;
  }

  if(gStatus.Sensors.Struct.Map != HAL_OK && gStatus.Sensors.Struct.ThrottlePos != HAL_OK) {
    running = 0;
    rotates = 0;
  }

  km_drive = speed * 2.77777778e-10f * diff; // speed / (60 * 60 * 1.000.000) * diff
  km_driven += km_drive;

  if(!rotates) {
    running = 0;
    injection_time = 0;
    cycle_air_flow = 0;
    mass_air_flow = 0;
    injection_dutycycle = 0;
    effective_volume = 0;
    fuel_consumption_per_distance = 0;
    fuel_consumption_hourly = 0;
  } else {
    fuel_consumption = fuel_amount_per_cycle / table->fuel_mass_per_cc * (diff / period) * 0.001f * 2.0f;
    fuel_consumed += fuel_consumption;

    fuel_consumption_per_distance = fuel_consumption / km_drive * 100.0f;
    fuel_consumption_hourly = fuel_consumption * 3600000.0f;
  }

  if(!running) {
    ignition_advance = start_ignition_advance;
    if(gStatus.Sensors.Struct.ThrottlePos == HAL_OK) {
    	if(!rotates && throttle >= 95.0f)
    		ventilation_flag = 1;
    	else if(throttle < 90.0f)
    		ventilation_flag = 0;
    }
    if(ventilation_flag)
      injection_time = 0;
  }

  if(uspa > 0.0f && injection_time > 0.0f) {
    injection_phase_duration = injection_time / uspa;
  } else {
    injection_phase_duration = 0;
  }

  if(injection_dutycycle > 0.90f) {
    if(!gStatus.InjectionUnderflow.is_error) {
      gStatus.InjectionUnderflow.error_time = 0;
    } else {
      gStatus.InjectionUnderflow.error_time += HAL_DelayDiff(hal_now, gStatus.InjectionUnderflow.error_last);
    }
    gStatus.InjectionUnderflow.is_error = 1;
  } else {
    gStatus.InjectionUnderflow.is_error = 0;
  }
  gStatus.InjectionUnderflow.error_last = hal_now;

  gStatus.Knock.DetonationCountPerSecond -= diff * 0.000001f;
  if(gStatus.Knock.DetonationCountPerSecond < 0)
    gStatus.Knock.DetonationCountPerSecond = 0;

  if(running && gEcuParams.useKnockSensor && gStatus.Sensors.Struct.Knock == HAL_OK) {

    if(sens_get_ign(NULL) != GPIO_PIN_SET) {
      knock_running = 0;
      knock_running_time = 0;
    }

    if(knock_running) {
      knock_zone = math_interpolate_2d_clamp(ipRpm, ipFilling, TABLE_ROTATES_MAX, table->knock_zone, 0.0f, 1.0f);

      for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
        if(gStatus.Knock.UpdatedInternally[i]) {
          if(gStatus.Knock.Detonates[i] > 0.01f) {
            gStatus.Knock.DetonationCount++;
            gStatus.Knock.DetonationCountPerSecond += 1.0f;
            gStatus.Knock.DetonationCountCy[i]++;
            gStatus.Knock.GeneralStatus |= KnockStatusDedonation;
            gStatus.Knock.DetonationLast = hal_now;
            if(gStatus.Knock.Detonates[i] > 1.0f) {
              gStatus.Knock.GeneralStatus |= KnockStatusStrongDedonation;
              gStatus.Knock.StrongDetonationLast = hal_now;
            }
            gStatus.Knock.Advances[i] += (gStatus.Knock.Period[i] * 0.000001f * 3.3f) * gStatus.Knock.Detonates[i];  //0.3 seconds to advance
          } else {
            if(gStatus.Knock.Advances[i] > 0.0f) {
              gStatus.Knock.Advances[i] -= gStatus.Knock.Period[i] * 0.000001f * 0.1f; //10 seconds to restore
            }
            knock_low_noise_lpf = gStatus.Knock.Period[i] * 0.000001f * 0.33f; // 3.0 seconds
            knock_low_noise_lpf = CLAMP(knock_low_noise_lpf, 0.0f, 0.2f);
            if(gStatus.Knock.Voltages[i] < knock_noise_level * 0.5f) {
              knock_low_noise_state = knock_low_noise_state * (1.0f - knock_low_noise_lpf) + 1.0f * knock_low_noise_lpf;
            } else {
              knock_low_noise_state *= 1.0f - knock_low_noise_lpf;
            }
          }
          gStatus.Knock.Advances[i] = CLAMP(gStatus.Knock.Advances[i], 0.0f, 1.0f);
          gStatus.Knock.UpdatedInternally[i] = 0;
        }
      }
      if(knock_low_noise_state >= 0.5f && engine_temp >= KNOCK_LOW_NOISE_ON_ENGINE_TEMP_THRESHOLD) {
        gStatus.Knock.GeneralStatus |= KnockStatusLowNoise;
        gStatus.Knock.LowNoiseLast = hal_now;
      }
      if((gStatus.Knock.GeneralStatus & KnockStatusLowNoise) == KnockStatusLowNoise) {
        for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
          gStatus.Knock.Advances[i] = 1.0f;
        }
      }
    } else {
      knock_zone = 0.0f;
      knock_low_noise_state = 0;
      memset(gStatus.Knock.Advances, 0, sizeof(gStatus.Knock.Advances));
      memset(gStatus.Knock.Detonates, 0, sizeof(gStatus.Knock.Detonates));
      memset(gStatus.Knock.Voltages, 0, sizeof(gStatus.Knock.Voltages));
      memset(gStatus.Knock.Updated, 0, sizeof(gStatus.Knock.Updated));
      memset(gStatus.Knock.UpdatedInternally, 0, sizeof(gStatus.Knock.UpdatedInternally));
      memset(gStatus.Knock.UpdatedAdaptation, 0, sizeof(gStatus.Knock.UpdatedAdaptation));

      knock_running_time += diff;
      if(knock_running_time >= 1000000) {
        knock_running = 1;
        knock_running_time = 0;
      }
    }
  } else {
    knock_zone = 0.0f;
    knock_running = 0;
    knock_running_time = 0;
    knock_low_noise_state = 0;
    memset(gStatus.Knock.Advances, 0, sizeof(gStatus.Knock.Advances));
    memset(gStatus.Knock.Detonates, 0, sizeof(gStatus.Knock.Detonates));
    memset(gStatus.Knock.Voltages, 0, sizeof(gStatus.Knock.Voltages));
    memset(gStatus.Knock.Updated, 0, sizeof(gStatus.Knock.Updated));
    memset(gStatus.Knock.UpdatedInternally, 0, sizeof(gStatus.Knock.UpdatedInternally));
    memset(gStatus.Knock.UpdatedAdaptation, 0, sizeof(gStatus.Knock.UpdatedAdaptation));
    gStatus.Knock.AdaptationDetonate = 0;
    gStatus.Knock.GeneralStatus = KnockStatusOk;
  }

  gStatus.Knock.Advance = 0;
  for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
    if(gStatus.Knock.Advances[i] > gStatus.Knock.Advance)
      gStatus.Knock.Advance = gStatus.Knock.Advances[i];
    if(!calibration && !gForceParameters.Enable.IgnitionAdvance && !gForceParameters.Enable.InjectionPulse) {
      gEcuCorrections.knock_ignition_correctives[i] = table->knock_ign_corr_max * knock_zone * gStatus.Knock.Advances[i];
      gEcuCorrections.knock_injection_correctives[i] = table->knock_inj_corr_max * knock_zone * gStatus.Knock.Advances[i];
    } else {
      gEcuCorrections.knock_ignition_correctives[i] = 0.0f;
      gEcuCorrections.knock_injection_correctives[i] = 0.0f;
    }
  }

  if(gStatus.Knock.StrongDetonationLast && HAL_DelayDiff(hal_now, gStatus.Knock.StrongDetonationLast) > 5000) {
    gStatus.Knock.GeneralStatus &= ~KnockStatusStrongDedonation;
    gStatus.Knock.StrongDetonationLast = 0;
  }
  if(gStatus.Knock.DetonationLast && HAL_DelayDiff(hal_now, gStatus.Knock.DetonationLast) > 5000) {
    gStatus.Knock.GeneralStatus &= ~KnockStatusDedonation;
    gStatus.Knock.DetonationLast = 0;
  }
  if(gStatus.Knock.LowNoiseLast && HAL_DelayDiff(hal_now, gStatus.Knock.LowNoiseLast) > 5000) {
    gStatus.Knock.GeneralStatus &= ~KnockStatusLowNoise;
    gStatus.Knock.LowNoiseLast = 0;
  }

  if(running) {
    gStatus.Knock.DetonationRelation = gStatus.Knock.DetonationCountPerSecond / rpm * 30.0f;
  } else {
    gStatus.Knock.DetonationRelation = 0;
  }

  if(gStatus.Knock.DetonationRelation > 0.1f) {
    gStatus.Knock.GeneralStatus &= ~KnockStatusContinuousDedonation;
  } else {
    gStatus.Knock.GeneralStatus &= ~KnockStatusContinuousDedonation;
  }

  fuel_ratio_diff = fuel_ratio / wish_fuel_ratio;

  if(adapt_diff >= period_half) {
    adaptation_last = now;
    if(running) {
      calibration_permitted_to_perform = enrichment_post_cycles > LEARN_ENRICHMENT_POST_CYCLES_DELAY && idle_accelerate_post_cycles >= IDLE_ACCELERATE_POST_CYCLES_DELAY;

      lpf_calculation = adapt_diff * 0.000001f;
      if(lpf_calculation > 0.1f)
        lpf_calculation = 0.1f;

      if(calibration && corr_math_interpolate_2d_set_func) {
        if(gEcuParams.useLambdaSensor && gStatus.Sensors.Struct.Lambda == HAL_OK && injector_status == HAL_OK && o2_valid &&
            (idle_calibration || !idle_flag) && calibration_permitted_to_perform) {
          gEcuCorrections.long_term_correction = 0.0f;
          gEcuCorrections.idle_correction = 0.0f;
          short_term_correction = 0.0f;
          short_term_correction_pid = 0.0f;

          filling_diff = (fuel_ratio_diff) - 1.0f;
          if(gStatus.Sensors.Struct.Map == HAL_OK && !econ_flag) {
            fill_correction_density += filling_diff * lpf_calculation;
            idle_fill_correction_density += filling_diff * lpf_calculation;

            if(use_idle_filling && idle_corr_flag) {
              lpf_calculation *= 0.2f; // 5 sec

              //ipIdleFillingDensity = math_interpolate_input(density_when_injected, table->idle_filling_densities, table->idle_filling_densities_count);
              corr_math_interpolate_2d_set_func(ipIdleFillingRpm, ipIdleFillingDensity, TABLE_ROTATES_MAX, gEcuCorrections.idle_filling_by_density, idle_fill_correction_density, -1.0f, 1.0f);

              percentage = (filling_diff + 1.0f);
              if(percentage > 1.0f) percentage = 1.0f / percentage;
              calib_cur_progress = corr_math_interpolate_2d_func(ipIdleFillingRpm, ipIdleFillingDensity, TABLE_ROTATES_MAX, gEcuCorrectionsProgress.progress_idle_filling_by_density);
              calib_cur_progress = (percentage * lpf_calculation) + (calib_cur_progress * (1.0f - lpf_calculation));
              corr_math_interpolate_2d_set_func(ipIdleFillingRpm, ipIdleFillingDensity, TABLE_ROTATES_MAX, gEcuCorrectionsProgress.progress_idle_filling_by_density, calib_cur_progress, 0.0f, 1.0f);

            } else {
              //ipDensity = math_interpolate_input(density_when_injected, table->densities, table->densities_count);
              corr_math_interpolate_2d_set_func(ipRpm, ipDensity, TABLE_ROTATES_MAX, gEcuCorrections.fill_by_density, fill_correction_density, -1.0f, 1.0f);

              percentage = (filling_diff + 1.0f);
              if(percentage > 1.0f) percentage = 1.0f / percentage;
              calib_cur_progress = corr_math_interpolate_2d_func(ipRpm, ipDensity, TABLE_ROTATES_MAX, gEcuCorrectionsProgress.progress_fill_by_density);
              calib_cur_progress = (percentage * lpf_calculation) + (calib_cur_progress * (1.0f - lpf_calculation));
              corr_math_interpolate_2d_set_func(ipRpm, ipDensity, TABLE_ROTATES_MAX, gEcuCorrectionsProgress.progress_fill_by_density, calib_cur_progress, 0.0f, 1.0f);
            }
          }
        }

        if(gStatus.Sensors.Struct.ThrottlePos == HAL_OK && gStatus.Sensors.Struct.Map == HAL_OK) {
          map_diff_thr = (pressure / pressure_from_throttle) - 1.0f;
          map_correction_thr += map_diff_thr * lpf_calculation;
          corr_math_interpolate_2d_set_func(ipRpm, ipThr, TABLE_ROTATES_MAX, gEcuCorrections.map_by_thr, map_correction_thr, -1.0f, 1.0f);

          percentage = (map_diff_thr + 1.0f);
          if(percentage > 1.0f) percentage = 1.0f / percentage;
          calib_cur_progress = corr_math_interpolate_2d_func(ipRpm, ipThr, TABLE_ROTATES_MAX, gEcuCorrectionsProgress.progress_map_by_thr);
          calib_cur_progress = (percentage * lpf_calculation) + (calib_cur_progress * (1.0f - lpf_calculation));
          corr_math_interpolate_2d_set_func(ipRpm, ipThr, TABLE_ROTATES_MAX, gEcuCorrectionsProgress.progress_map_by_thr, calib_cur_progress, 0.0f, 1.0f);
        }

        if(idle_flag && gStatus.Sensors.Struct.Map == HAL_OK && gStatus.Sensors.Struct.ThrottlePos == HAL_OK && !gForceParameters.Enable.WishIdleValvePosition) {
          idle_valve_pos_dif = (idle_wish_valve_pos / idle_table_valve_pos) - 1.0f;
          idle_valve_pos_adaptation += idle_valve_pos_dif * lpf_calculation;

          math_interpolate_1d_set(ipEngineTemp, gEcuCorrections.idle_valve_position, idle_valve_pos_adaptation, -1.0f, 1.0f);

          percentage = (idle_valve_pos_dif + 1.0f);
          if(percentage > 1.0f) percentage = 1.0f / percentage;
          calib_cur_progress = math_interpolate_1d(ipEngineTemp, gEcuCorrectionsProgress.progress_idle_valve_position);
          calib_cur_progress = (percentage * lpf_calculation) + (calib_cur_progress * (1.0f - lpf_calculation));
          math_interpolate_1d_set(ipEngineTemp, gEcuCorrectionsProgress.progress_idle_valve_position, calib_cur_progress, 0.0f, 1.0f);
        }

        if(gEcuParams.useKnockSensor && gStatus.Sensors.Struct.Knock == HAL_OK) {
          calib_cur_progress = corr_math_interpolate_2d_func(ipRpm, ipFilling, TABLE_ROTATES_MAX, gEcuCorrectionsProgress.progress_ignitions);

          if(knock_zone > 0.05f && !idle_flag) {
            for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
              if(gStatus.Knock.UpdatedAdaptation[i]) {
                memset(gStatus.Knock.UpdatedAdaptation, 0, sizeof(gStatus.Knock.UpdatedAdaptation));

                detonation_count_table = math_interpolate_2d(ipRpm, ipFilling, TABLE_ROTATES_MAX, gEcuCorrections.knock_detonation_counter);

                if(gStatus.Knock.AdaptationDetonate > 0.05f) {
                  gStatus.Knock.AdaptationDetonate = 0;
                  detonation_count_table += 1.0f;

                  calib_cur_progress = 0.0f;

                  ignition_correction = table->knock_ign_corr_max * knock_zone * lpf_calculation + ignition_correction * (1.0f - lpf_calculation);
                  corr_math_interpolate_2d_set_func(ipRpm, ipFilling, TABLE_ROTATES_MAX, gEcuCorrections.ignitions, ignition_correction, -25.0f, 25.0f);

                  math_interpolate_2d_set(ipRpm, ipFilling, TABLE_ROTATES_MAX, gEcuCorrections.knock_detonation_counter, detonation_count_table, 0.0f, 99.0f);
                } else {
#if defined(KNOCK_DETONATION_INCREASING_ADVANCE) && KNOCK_DETONATION_INCREASING_ADVANCE > 0
                  if(detonation_count_table < 3.0f) {
                    lpf_calculation *= 0.2f; //5 sec
                    ignition_correction = -table->knock_ign_corr_max * knock_zone * lpf_calculation + ignition_correction * (1.0f - lpf_calculation);
                    corr_math_interpolate_2d_set_func(ipRpm, ipFilling, TABLE_ROTATES_MAX, gEcuCorrections.ignitions, ignition_correction, -25.0f, 25.0f);
                  }
#endif /* KNOCK_DETONATION_INCREASING_ADVANCE */
                  calib_cur_progress = (1.0f * lpf_calculation) + (calib_cur_progress * (1.0f - lpf_calculation));
                }
                corr_math_interpolate_2d_set_func(ipRpm, ipFilling, TABLE_ROTATES_MAX, gEcuCorrectionsProgress.progress_ignitions, calib_cur_progress, 0.0f, 1.0f);
                break;
              }
            }
          } else if(idle_flag) {
            if(engine_temp >= KNOCK_LOW_NOISE_ON_ENGINE_TEMP_THRESHOLD) {
              for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
                if(gStatus.Knock.UpdatedAdaptation[i]) {
                  lpf_calculation *= 0.3f; //3 sec

                  knock_cy_level_diff = knock_noise_level;
                  knock_cy_level_diff /= gStatus.Knock.Voltages[i] * knock_cy_level_multiplier[i];
                  knock_cy_level_diff -= 1.0f;

                  knock_cy_level_multiplier_correction[i] += knock_cy_level_diff * lpf_calculation;
                  math_interpolate_1d_set(ipRpm, gEcuCorrections.knock_cy_level_multiplier[i], knock_cy_level_multiplier_correction[i], -1.0f, 1.0f);

                  calib_cur_progress = math_interpolate_1d(ipRpm, gEcuCorrectionsProgress.progress_knock_cy_level_multiplier[i]);
                  calib_cur_progress = (1.0f * lpf_calculation) + (calib_cur_progress * (1.0f - lpf_calculation));
                  math_interpolate_1d_set(ipRpm, gEcuCorrectionsProgress.progress_knock_cy_level_multiplier[i], calib_cur_progress, 0.0f, 1.0f);
                  gStatus.Knock.UpdatedAdaptation[i] = 0;
                }
              }
            } else {
              memset(gStatus.Knock.UpdatedAdaptation, 0, sizeof(gStatus.Knock.UpdatedAdaptation));
            }
          }

        }
      } else {
        gStatus.Knock.AdaptationDetonate = 0;

        if(gEcuParams.useLambdaSensor && gStatus.Sensors.Struct.Lambda == HAL_OK && injector_status == HAL_OK && o2_valid && !econ_flag) {
          filling_diff = (fuel_ratio_diff) - 1.0f;
          if(gEcuParams.useLongTermCorr) {
            if(calibration_permitted_to_perform) {
              if(!idle_flag) {
                lpf_calculation *= 0.0333f; //30 sec
                gEcuCorrections.long_term_correction += (filling_diff + short_term_correction) * lpf_calculation;
              }
              else if(idle_corr_flag) {
                lpf_calculation *= 0.3333f; //3 sec
                gEcuCorrections.idle_correction += (filling_diff + short_term_correction) * lpf_calculation;
              }
            }
          } else {
            gEcuCorrections.long_term_correction = 0;
            gEcuCorrections.idle_correction = 0;
          }
        }
      }
    }
  }

  gEcuCorrections.long_term_correction = CLAMP(gEcuCorrections.long_term_correction, -20.0f, 20.0f);
  gEcuCorrections.idle_correction = CLAMP(gEcuCorrections.idle_correction, -20.0f, 20.0f);

  gEcuCriticalBackup.idle_valve_position = idle_valve_position;

  if(fuel_consumed > 0.1f) {
    gEcuCriticalBackup.fuel_consumed += fuel_consumed;
    fuel_consumed = 0;
  }

  if(km_driven > 0.1f) {
    gEcuCriticalBackup.km_driven += km_driven;
    km_driven = 0;
  }

  if(gStatus.Sensors.Struct.Map == HAL_OK && gStatus.Sensors.Struct.ThrottlePos == HAL_OK) {
    if(running &&
        ((!idle_flag || (rpm > idle_reg_rpm_2 && engine_temp > 80.0f)))) {
      float map_tps_relation = pressure / pressure_from_throttle;
      if(map_tps_relation > 1.10f || map_tps_relation < 0.80f) {
        if(gStatus.MapTpsRelation.is_error) {
          gStatus.MapTpsRelation.error_time += HAL_DelayDiff(hal_now, gStatus.MapTpsRelation.error_last);
        } else {
          gStatus.MapTpsRelation.error_time = 0;
        }
        gStatus.MapTpsRelation.is_error = 1;
      } else {
        gStatus.MapTpsRelation.is_error = 0;
      }
    } else {
      gStatus.MapTpsRelation.error_time = 0;
    }
  } else {
    gStatus.MapTpsRelation.is_error = 0;
    gStatus.MapTpsRelation.error_time = 0;
  }

  gStatus.MapTpsRelation.error_last = hal_now;

  min = 0.93f;
  max = 1.07f;

  if(gEcuParams.useKnockSensor && gStatus.Sensors.Struct.Knock == HAL_OK) {
    max *= (table->knock_inj_corr_max * knock_zone) + 1.0f;
  }

  if(gStatus.Sensors.Struct.Lambda == HAL_OK && o2_valid && running) {
    if(idle_flag) {
      if(fuel_ratio_diff < min) {
        if(gStatus.RichIdleMixture.is_error) {
          gStatus.RichIdleMixture.error_time += HAL_DelayDiff(hal_now, gStatus.RichIdleMixture.error_last);
        } else {
          gStatus.RichIdleMixture.error_time = 0;
        }
        gStatus.RichIdleMixture.is_error = 1;
      } else {
        gStatus.RichIdleMixture.is_error = 0;
      }
      if(fuel_ratio_diff > max) {
        if(gStatus.LeanIdleMixture.is_error) {
          gStatus.LeanIdleMixture.error_time += HAL_DelayDiff(hal_now, gStatus.LeanIdleMixture.error_last);
        } else {
          gStatus.LeanIdleMixture.error_time = 0;
        }
        gStatus.LeanIdleMixture.is_error = 1;
      } else {
        gStatus.LeanIdleMixture.is_error = 0;
      }
    } else {
      if(fuel_ratio_diff < min) {
        if(gStatus.RichMixture.is_error) {
          gStatus.RichMixture.error_time += HAL_DelayDiff(hal_now, gStatus.RichMixture.error_last);
        } else {
          gStatus.RichMixture.error_time = 0;
        }
        gStatus.RichMixture.is_error = 1;
      } else {
        gStatus.RichMixture.is_error = 0;
      }
      if(fuel_ratio_diff > max) {
        if(gStatus.LeanMixture.is_error) {
          gStatus.LeanMixture.error_time += HAL_DelayDiff(hal_now, gStatus.LeanMixture.error_last);
        } else {
          gStatus.LeanMixture.error_time = 0;
        }
        gStatus.LeanMixture.is_error = 1;
      } else {
        gStatus.LeanMixture.is_error = 0;
      }
    }
  } else {
    gStatus.RichMixture.error_time = 0;
    gStatus.LeanMixture.error_time = 0;
    gStatus.RichIdleMixture.error_time = 0;
    gStatus.LeanIdleMixture.error_time = 0;
  }
  gStatus.RichMixture.error_last = hal_now;
  gStatus.LeanMixture.error_last = hal_now;
  gStatus.RichIdleMixture.error_last = hal_now;
  gStatus.LeanIdleMixture.error_last = hal_now;

  ecu_update_shared_parameters();

  gParameters.AdcKnockVoltage = gStatus.Knock.Voltage;
  gParameters.AdcAirTemp = adc_get_voltage(AdcChAirTemperature);
  gParameters.AdcEngineTemp = adc_get_voltage(AdcChEngineTemperature);
  gParameters.AdcManifoldAirPressure = adc_get_voltage(AdcChManifoldAbsolutePressure);
  gParameters.AdcThrottlePosition = adc_get_voltage(AdcChThrottlePosition);
  gParameters.AdcPowerVoltage = adc_get_voltage(AdcChPowerVoltage);
  gParameters.AdcReferenceVoltage = adc_get_voltage(AdcMcuChReferenceVoltage);
  gParameters.AdcLambdaUR = adc_get_voltage(AdcChO2UR);
  gParameters.AdcLambdaUA = adc_get_voltage(AdcChO2UA);

  gParameters.KnockSensor = gStatus.Knock.StatusVoltage;
  gParameters.KnockSensorFiltered = gStatus.Knock.StatusFiltered;
  gParameters.KnockSensorDetonate = gStatus.Knock.StatusDetonate;
  gParameters.KnockZone = knock_zone;
  gParameters.KnockAdvance = gStatus.Knock.Advance;
  gParameters.KnockCount = gStatus.Knock.DetonationCount;
  for(int i = 0; i < ECU_CYLINDERS_COUNT; i++)
    gParameters.KnockCountCy[i] = gStatus.Knock.DetonationCountCy[i];
  gParameters.AirTemp = air_temp;
  gParameters.EngineTemp = engine_temp;
  gParameters.ManifoldAirPressure = pressure;
  gParameters.ThrottlePosition = throttle;
  gParameters.ReferenceVoltage = reference_voltage;
  gParameters.PowerVoltage = power_voltage;
  gParameters.FuelRatio = fuel_ratio;
  gParameters.FuelRatioDiff = fuel_ratio_diff;
  gParameters.LambdaValue = lambda_value;
  gParameters.LambdaTemperature = lambda_temperature;
  gParameters.LambdaTemperatureVoltage = lambda_temperaturevoltage;
  gParameters.LambdaHeaterVoltage = lambda_heatervoltage;
  gParameters.ShortTermCorrection = short_term_correction;
  gParameters.LongTermCorrection = gEcuCorrections.long_term_correction;
  gParameters.IdleCorrection = gEcuCorrections.idle_correction;

  gParameters.IdleFlag = idle_flag;
  gParameters.IdleCorrFlag = idle_corr_flag;
  gParameters.IdleEconFlag = econ_flag;
  gParameters.RPM = rpm;
  gParameters.Acceleration = acceleration;
  gParameters.Speed = speed;
  gParameters.MassAirFlow = mass_air_flow;
  gParameters.CyclicAirFlow = cycle_air_flow;
  gParameters.EffectiveVolume = effective_volume;
  gParameters.AirDensity = air_density;
  gParameters.RelativeFilling = filling_relative;
  gParameters.WishFuelRatio = wish_fuel_ratio;
  gParameters.IdleValvePosition = idle_valve_position;
  gParameters.IdleRegThrRPM = idle_reg_rpm_1;
  gParameters.WishIdleRPM = idle_wish_rpm;
  gParameters.WishIdleMassAirFlow = idle_wish_massair;
  gParameters.WishIdleValvePosition = idle_wish_valve_pos;
  gParameters.WishIdleIgnitionAdvance = idle_wish_ignition;
  gParameters.IgnitionAdvance = ignition_advance;
  gParameters.InjectionPhase = injection_phase;
  gParameters.InjectionPhaseDuration = injection_phase_duration;

  gParameters.InjectionPulse = injection_time;
  if(enrichment_post_injection_enabled && enrichment_triggered_once)
    gLocalParams.EnrichmentTriggered = 1;

  gParameters.InjectionDutyCycle = injection_dutycycle;
  gParameters.InjectionEnrichment = enrichment_result;
  gParameters.InjectionLag = injector_lag;
  gParameters.IgnitionPulse = ignition_time;
  gParameters.IdleSpeedShift = idle_rpm_shift;

  gParameters.EnrichmentSyncAmount = enrichment_amount_sync;
  gParameters.EnrichmentAsyncAmount = enrichment_amount_async;
  gParameters.EnrichmentStartLoad = enrichment_load_value_start_accept;
  gParameters.EnrichmentLoadDerivative = enrichment_load_derivative_accept;

  gParameters.DrivenKilometers = gEcuCriticalBackup.km_driven + km_driven;
  gParameters.FuelConsumed = gEcuCriticalBackup.fuel_consumed + fuel_consumed;
  gParameters.FuelConsumption = fuel_consumption_per_distance;
  gParameters.FuelHourly = fuel_consumption_hourly;
  gParameters.TspsRelativePosition = tsps_rel_pos;
  gParameters.IdleWishToRpmRelation = idle_wish_to_rpm_relation;

  gParameters.LambdaValid = o2_valid > 0;

  gParameters.OilSensor = sens_get_oil_pressure(NULL) != GPIO_PIN_RESET;
  gParameters.FanForceSwitch = sens_get_fan_force_switch(NULL) != GPIO_PIN_RESET;
  gParameters.HandbrakeSensor = sens_get_handbrake(NULL) != GPIO_PIN_RESET;
  gParameters.ChargeSensor = sens_get_charge(NULL);
  gParameters.ClutchSensor = sens_get_clutch(NULL) != GPIO_PIN_RESET;
  gParameters.IgnSensor = sens_get_ign(NULL) != GPIO_PIN_RESET;

  gParameters.FuelPumpRelay = out_get_fuelpump(NULL) != GPIO_PIN_RESET;
  gParameters.FanRelay = out_get_fan(NULL) != GPIO_PIN_RESET;
  gParameters.CheckEngine = out_get_checkengine(NULL) != GPIO_PIN_RESET;
  gParameters.StarterRelay = out_get_starter(NULL) != GPIO_PIN_RESET;
  gParameters.FanSwitch = out_get_fan_switch(NULL) != GPIO_PIN_RESET;
  gParameters.IgnOutput = out_get_ign(NULL) != GPIO_PIN_RESET;

  gParameters.IsRunning = running;
  gParameters.IsCheckEngine = gEcuIsError;
  gParameters.InjectorChannel = table->inj_channel;

  strcpy(gParameters.CurrentTableName, table->name);

  gDiagWorkingMode.Bits.is_enrichment = enrichment_triggered;
  gDiagWorkingMode.Bits.is_idle = idle_flag && running;
  gDiagWorkingMode.Bits.is_idle_last = idle_flag && running;
  gDiagWorkingMode.Bits.is_idle_lock_last = 0;
  gDiagWorkingMode.Bits.is_knock = (gStatus.Knock.GeneralStatus & KnockStatusDedonation) > 0;
  gDiagWorkingMode.Bits.is_knock_zone = !idle_flag && running;
  gDiagWorkingMode.Bits.is_knock_zone_last = !idle_flag && running;

  gDiagWorkingMode.Bits.is_lambda_last = o2_valid;
  gDiagWorkingMode.Bits.is_lambda_now = o2_valid;
  gDiagWorkingMode.Bits.is_lambda_teach = o2_valid && calibration;
  gDiagWorkingMode.Bits.is_not_running = !running;
  gDiagWorkingMode.Bits.is_use_lambda = o2_valid;

  updated_last = now;
}

static void ecu_init_post_init(void)
{
  out_enable_idle_valve(DEFAULT_IDLE_VALVE_POSITION);
  gEcuCriticalBackup.idle_valve_position = 0;
}

static void ecu_backup_save_process(void)
{
  static uint8_t state = 0;
  static uint32_t save_correction_last = 0;
  int8_t status;
  uint32_t now = Delay_Tick;

  if(!CRC16_IsBusy()) {
    if(!Mem.critical_lock) {
      Mem.critical_lock = 1;
      status = config_save_critical_backup(&gEcuCriticalBackup);
      Mem.critical_lock = 0;
      gCriticalStatus = status < 0 ? status : 0;
    }
  }

  switch(state) {
    case 0:
      if(DelayDiff(now, save_correction_last) > 500000) {
        save_correction_last = now;
        state++;
      }
      break;
    case 1:
      if(!CRC16_IsBusy()) {
        status = config_save_corrections(&gEcuCorrections);
        if(status) {
          state = 0;
        }
        gBackupStatus = status < 0 ? status : 0;
      }
      break;

    default:
      state = 0;
      break;
  }

  if(gBackupStatus < 0) {
      gStatus.Bkpsram.Struct.CorrsSave = HAL_ERROR;
  }
  if(gCriticalStatus < 0) {
      gStatus.Bkpsram.Struct.CriticalSave = HAL_ERROR;
  }
}

STATIC_INLINE uint8_t ecu_cutoff_ign_act(uint8_t cy_count, uint8_t cylinder, float rpm)
{
  static uint8_t cutoff_processing_prev = 0;
  static int8_t cutoffcnt0 = -1;
  static int8_t cutoffcnt1 = -1;
  static int8_t cutoffcnt2 = -1;
  static int8_t cutoffcnt3 = -1;
  static int8_t cutoffcnt4 = -1;
  static int8_t cutoffcnt5 = -1;
  static int8_t cutoffcnt6 = -1;
  static int8_t cy_prev = -1;
  int32_t mode = gEcuParams.cutoffMode;
  float cutoffrpm = gEcuParams.cutoffRPM;

  //Cutoff always enabled
  if(1)
  {
    if(rpm >= cutoffrpm)
    {
      if(mode == 0)
      {
        Cutoff.Processing = 1;
        cutoff_processing_prev = 32;
        Cutoff.Cutting = 1;
        return 0;
      }
      else if(mode == 1)
      {
        if(cutoffcnt0 == -1)
          cutoffcnt0 = 0;
      }
      else if(mode == 2)
      {
        if(cutoffcnt1 == -1)
          cutoffcnt1 = 0;
      }
      else if(mode == 3)
      {
        if(cutoffcnt2 == -1)
          cutoffcnt2 = 0;
      }
      else if(mode == 4)
      {
        if(cutoffcnt3 == -1)
          cutoffcnt3 = 0;
      }
      else if(mode == 5)
      {
        if(cutoffcnt4 == -1)
          cutoffcnt4 = 0;
      }
      else if(mode == 6)
      {
        if(cutoffcnt5 == -1)
          cutoffcnt5 = 0;
      }
      else if(mode == 7)
      {
        if(cutoffcnt6 == -1)
          cutoffcnt6 = 0;
      }
    }
  }

  if(cy_prev != cylinder)
  {
    cy_prev = cylinder;

    if(cutoff_processing_prev)
    {
      cutoff_processing_prev--;
      Cutoff.Processing = 1;
    }
    else Cutoff.Processing = 0;

    if(cutoffcnt0 >= 0)
    {
      Cutoff.Processing = 1;
      cutoff_processing_prev = 36+4+8;
      if(++cutoffcnt0 > 36)
        cutoffcnt0 = -1;
      else if(cutoffcnt0 <= 36-4) {
        Cutoff.Cutting = 1;
        return 0;
      }
    }

    if(cutoffcnt1 >= 0)
    {
      Cutoff.Processing = 1;
      cutoff_processing_prev = 24+4+8;
      if(++cutoffcnt1 > 24)
        cutoffcnt1 = -1;
      else if(cutoffcnt1 <= 24-4) {
        Cutoff.Cutting = 1;
        return 0;
      }
    }

    if(cutoffcnt2 >= 0)
    {
      Cutoff.Processing = 1;
      cutoff_processing_prev = 16+4+8;
      if(++cutoffcnt2 > 16)
        cutoffcnt2 = -1;
      else if(cutoffcnt2 <= 16-4) {
        Cutoff.Cutting = 1;
        return 0;
      }
    }

    if(cutoffcnt3 >= 0)
    {
      Cutoff.Processing = 1;
      cutoff_processing_prev = 8+4+8;
      if(++cutoffcnt3 > 8)
        cutoffcnt3 = -1;
      else if(cutoffcnt3 <= 8-4) {
        Cutoff.Cutting = 1;
        return 0;
      }
    }

    if(cutoffcnt4 >= 0)
    {
      Cutoff.Processing = 1;
      cutoff_processing_prev = 20+4+8;
      if(++cutoffcnt4 > 20)
        cutoffcnt4 = -1;
      else if((cutoffcnt4 % 5) != 0) {
        Cutoff.Cutting = 1;
        return 0;
      }
    }

    if(cutoffcnt5 >= 0)
    {
      Cutoff.Processing = 1;
      cutoff_processing_prev = 12+4+8;
      if(++cutoffcnt5 > 12)
        cutoffcnt5 = -1;
      else if((cutoffcnt5 % 3) != 0) {
        Cutoff.Cutting = 1;
        return 0;
      }
    }

    if(cutoffcnt6 >= 0)
    {
      Cutoff.Processing = 1;
      cutoff_processing_prev = 12+4+8;
      if(++cutoffcnt6 > 12)
        cutoffcnt6 = -1;
      else if((cutoffcnt6 % 3) == 0) {
        Cutoff.Cutting = 1;
        return 0;
      }
    }

  }
  Cutoff.Cutting = 0;
  return 1;
}

STATIC_INLINE uint8_t ecu_cutoff_inj_act(uint8_t cy_count, uint8_t cylinder, float rpm)
{
  float cutoffrpm = gEcuParams.cutoffRPM;

  //Just for safety purpose...
  gDiagWorkingMode.Bits.is_fuel_cutoff = 0;
  if(rpm >= cutoffrpm + 500 ||
      (gStatus.OilPressure.is_error && rpm >= gEcuParams.oilPressureCutoffRPM) ||
      (gStatus.OilPressure.is_error && gStatus.OilPressure.error_time > 10000)) {
    gDiagWorkingMode.Bits.is_fuel_cutoff = 1;
    Cutoff.FuelCutoff = 1;
    return 0;
  }

  Cutoff.FuelCutoff = 0;
  return 1;
}

STATIC_INLINE uint8_t ecu_shift_process(uint8_t cy_count, uint8_t cylinder, uint8_t mode, GPIO_PinState clutch, uint8_t reset)
{
  static int8_t shift_cnt1 = -1;
  static uint32_t shift_time_last = 0;
  uint32_t tick_count = 0;
  uint32_t now = Delay_Tick;
  uint8_t retval = 1;

  if(mode > 0 && !reset) {
    if(mode == 1) {
      if(clutch == GPIO_PIN_SET) {
        shift_cnt1 = 5;
        retval = 0;
      } else {
        if(shift_cnt1 >= 0) {
          shift_cnt1--;
          retval = 1;
        } else {
          retval = 2;
        }
      }
    } else if(mode > 1) {
      switch(mode) {
        case 2 : tick_count = 20000; break;
        case 3 : tick_count = 50000; break;
        case 4 : tick_count = 100000; break;
        default: break;
      }
      if(clutch == GPIO_PIN_SET) {
        if(DelayDiff(now, shift_time_last) < tick_count) {
          retval = 0;
        } else {
          retval = 1;
        }
      } else {
        retval = 2;
      }
    }
  } else {
    shift_cnt1 = -1;
    retval = 2;
  }

  if(retval > 0) {
    if(tick_count > 0) {
      shift_time_last += tick_count;
    } else {
      shift_time_last = now;
    }
  }

  return retval;
}

STATIC_INLINE uint8_t ecu_shift_ign_act(uint8_t cy_count, uint8_t cylinder, GPIO_PinState clutch, float rpm, float throttle)
{
  uint8_t mode = gEcuParams.shiftMode;
  float thrthr = gEcuParams.shiftThrThr;
  float rpmthr = gEcuParams.shiftRpmThr;
  float rpmtill = gEcuParams.shiftRpmTill;
  uint8_t shift_result = 1;

  if(mode > 0 && throttle >= thrthr) {
    if(clutch == GPIO_PIN_RESET) {
      if(!Shift.Shifting) {
        Shift.Tilled = 0;
        Shift.Shifting = 0;
        if(rpm >= rpmthr && throttle >= thrthr) {
          Shift.Thresholded = 1;
        }
      }
    } else {
      if(Shift.Thresholded) {
        if(rpm >= rpmtill && !Shift.Tilled) {
          Shift.Shifting = 1;
        } else {
          Shift.Tilled = 1;
          Shift.Shifting = 0;
          Shift.Thresholded = 0;
        }
      }
    }
  } else {
    Shift.Shifting = 0;
    Shift.Thresholded = 0;
    Shift.Tilled = 0;
  }

  if(Shift.Thresholded && Shift.Shifting && !Shift.Tilled) {
    shift_result = ecu_shift_process(cy_count, cylinder, mode, clutch, 0);
    if(shift_result > 1) {
      Shift.Shifting = 0;
      Shift.Thresholded = 0;
      Shift.Tilled = 0;
    }
  } else {
    shift_result = ecu_shift_process(cy_count, cylinder, mode, clutch, 1);
  }

  return shift_result > 0;
}

STATIC_INLINE uint8_t ecu_shift_inj_act(uint8_t cy_count, uint8_t cylinder, GPIO_PinState clutch, float rpm, float throttle)
{
  //TODO: currently not affecting injection directly
  return 1;
}

STATIC_INLINE void ecu_coil_saturate(uint8_t cy_count, uint8_t cylinder)
{
  if(cy_count == ECU_CYLINDERS_COUNT && cylinder < ECU_CYLINDERS_COUNT) {
    gIgnPorts[cylinder]->BSRR = gIgnPins[cylinder];
  } else if(cy_count == ECU_CYLINDERS_COUNT_HALF && cylinder < ECU_CYLINDERS_COUNT_HALF) {
    gIgnPorts[cylinder]->BSRR = gIgnPins[cylinder];
    gIgnPorts[ECU_CYLINDERS_COUNT - 1 - cylinder]->BSRR = gIgnPins[ECU_CYLINDERS_COUNT - 1 - cylinder];
  } else if(cy_count == 1) {
    for(int i = 0; i < ECU_CYLINDERS_COUNT; i++)
      gIgnPorts[i]->BSRR = gIgnPins[i];
  }
}

STATIC_INLINE void ecu_coil_ignite(uint8_t cy_count, uint8_t cylinder)
{
  if(cy_count == ECU_CYLINDERS_COUNT && cylinder < ECU_CYLINDERS_COUNT) {
    gIgnPorts[cylinder]->BSRR = gIgnPins[cylinder] << 16;
  } else if(cy_count == ECU_CYLINDERS_COUNT_HALF && cylinder < ECU_CYLINDERS_COUNT_HALF) {
    gIgnPorts[cylinder]->BSRR = gIgnPins[cylinder] << 16;
    gIgnPorts[ECU_CYLINDERS_COUNT - 1 - cylinder]->BSRR = gIgnPins[ECU_CYLINDERS_COUNT - 1 - cylinder] << 16;
  } else if(cy_count == 1) {
    for(int i = 0; i < ECU_CYLINDERS_COUNT; i++)
      gIgnPorts[i]->BSRR = gIgnPins[i] << 16;
  }
}

STATIC_INLINE void ecu_inject(uint8_t cy_count, uint8_t cylinder, uint32_t time)
{
#if defined(INJECTORS_ON_INJ_CH1_ONLY) && INJECTORS_ON_INJ_CH1_ONLY == 1
  if(gParameters.InjectorChannel != InjectorChannel1) {
    return;
  }
#endif
  if(cy_count == ECU_CYLINDERS_COUNT && cylinder < ECU_CYLINDERS_COUNT) {
    injector_enable(cylinder, time);
  } else if(cy_count == ECU_CYLINDERS_COUNT_HALF) {
    injector_enable(cylinder, time);
    injector_enable(ECU_CYLINDERS_COUNT - 1 - cylinder, time);
  } else if(cy_count == 1) {
    for(int i = 0; i < ECU_CYLINDERS_COUNT; i++)
      injector_enable(i, time);
  }
}

STATIC_INLINE void ecu_inject_async(uint32_t time)
{
  for(int i = 0; i < ECU_CYLINDERS_COUNT; i++)
    injector_enable(i, time);
}

ITCM_FUNC void ecu_process(void)
{
  sEcuTable *table = &gEcuTable[ecu_get_table()];
  uint8_t found = csps_isfound();
  uint8_t running = csps_isrunning();
  sCspsData csps = csps_data();
  uint32_t now = Delay_Tick;
  static uint32_t last = 0;
  uint32_t diff = DelayDiff(now, last);
  last = now;

  static GPIO_PinState clutch = GPIO_PIN_RESET;
  static uint8_t was_found = 0;
  static uint8_t last_start_triggered = 0;
  static uint32_t last_start = 0;
  static uint32_t turns_count_last = 0;
  static float oldanglesbeforeignite[ECU_CYLINDERS_COUNT] = {0,0,0,0};
  static float oldanglesbeforeinject[ECU_CYLINDERS_COUNT] = {0,0,0,0};
  static float injection_time_prev[ECU_CYLINDERS_COUNT] = {0,0,0,0};
#if defined(IGNITION_ACCEPTION_FEATURE) && IGNITION_ACCEPTION_FEATURE > 0
  static float ignition_accepted_angle[ECU_CYLINDERS_COUNT] = {0,0,0,0};
#endif
  static float enrichment_time_saturation[ECU_CYLINDERS_COUNT] = {0,0,0,0};
  static uint8_t enrichment_triggered[ECU_CYLINDERS_COUNT] = { 0,0,0,0 };
  static uint8_t saturated[ECU_CYLINDERS_COUNT] = { 1,1,1,1 };
  static uint8_t ignited[ECU_CYLINDERS_COUNT] = { 1,1,1,1 };
  static uint8_t injected[ECU_CYLINDERS_COUNT] = { 1,1,1,1 };
  //static uint8_t ignition_ready[ECU_CYLINDERS_COUNT] = { 0,0,0,0 };
  static uint8_t injection[ECU_CYLINDERS_COUNT] = { 1,1,1,1 };
  static uint8_t ign_prepare[ECU_CYLINDERS_COUNT] = { 0,0,0,0 };
  static uint8_t knock_process[ECU_CYLINDERS_COUNT] = { 0,0,0,0 };
  static uint8_t knock_busy = 0;
  static int8_t knock_cylinder = -1;
  static uint8_t inj_was_phased = 0;
  static uint8_t ign_was_phased = 0;
  static uint8_t knock_was_phased = 0;
  float angle_injection[ECU_CYLINDERS_COUNT];
  float angle_ignition[ECU_CYLINDERS_COUNT];
  float angle_knock[ECU_CYLINDERS_COUNT];
  static float ignition_saturate[ECU_CYLINDERS_COUNT];
  static float ignition_ignite[ECU_CYLINDERS_COUNT];
  static uint32_t ignition_saturate_time[ECU_CYLINDERS_COUNT];
  static uint32_t ignition_ignite_time[ECU_CYLINDERS_COUNT];
  static float anglesbeforeignite[ECU_CYLINDERS_COUNT];
  float anglesbeforeinject[ECU_CYLINDERS_COUNT];
  float injection_time_diff;
  float enrichment_end_injection_final_phase = table->enrichment_end_injection_final_phase;
  float enrichment_end_injection_final_amount = table->enrichment_end_injection_final_amount;

  HAL_StatusTypeDef map_status = HAL_OK;
  float pressure = 0;
  float density = 0;
#if defined(PRESSURE_ACCEPTION_FEATURE) && PRESSURE_ACCEPTION_FEATURE > 0
  static float oldanglesbeforepressure[ECU_CYLINDERS_COUNT_HALF] = {0,0};
  static uint8_t pressure_measurement[ECU_CYLINDERS_COUNT_HALF] = { 1,1 };
  static float pressure_filtered = 0;
  static uint32_t pressure_count = 0;
  float anglesbeforepressure[ECU_CYLINDERS_COUNT_HALF];
  float pressure_measurement_time = 20;
  float pressure_measurement_angle = 90 + pressure_measurement_time;
#endif

  static const uint8_t phjase_detect_cycles_wait = 32;
  static const uint8_t phjase_detect_cycles_threshold = ACCELERATION_POINTS_COUNT * ECU_CYLINDERS_COUNT_HALF;
  static const float phjase_detect_fueling_multiplier = 1.85f;
  static uint8_t phase_detect_fueling = 0;
  static uint8_t phase_detect_running_cycles = 0;
  static uint8_t phase_detect_cycles = 0;
  uint8_t phase_need_clean = 0;
  uint8_t phase_detect_enabled = gPhaseDetectMethod;

  float knock_lpf;
  float throttle;
  float angle = csps_getphasedangle(csps);
  float rpm = csps_getrpm(csps);
  float uspa_koff = 0.8f;
  static float uspa = 1000.0f;
  float r_uspa;
  float uspa_raw = csps_getuspa(csps);
  float period = csps_getperiod(csps);
  float period_half = period * 0.5f;
  float time_sat;
  float time_pulse;
  float angle_ignite_koff = 0.1f;
  float inj_phase_koff = 0.0f;
  float angle_ignite_param;
  float inj_phase_param;
  float inj_phase_temp;
  static float angle_ignite = 10.0f;
  static float inj_phase = 200.0f;
  float saturate;
  float angles_injection_per_turn;
  float angles_ignition_per_turn;
  float inj_pulse;
  float inj_angle;
  float cur_cy_ignition;
  static float cy_ignition[ECU_CYLINDERS_COUNT] = {10,10,10,10};
  float cy_injection[ECU_CYLINDERS_COUNT];
  float var, var2, knock;
  uint8_t injection_phase_by_end;
  uint8_t phased_knock;
  uint8_t phased_ignition;
  uint8_t phased_injection;
  uint8_t cy_count_ignition;
  uint8_t cy_count_knock;
  uint8_t cy_count_injection;
  uint8_t individual_coils;
  uint8_t single_coil;
  uint8_t phased_mode;
  uint8_t injector_channel;
  uint8_t start_allowed = gParameters.StartAllowed;
  uint8_t cutoff_inj_act, cutoff_ign_act;
  uint8_t shift_inj_act, shift_ign_act;
  uint8_t shiftEnabled = gEcuParams.shiftMode > 0;
  uint8_t is_phased = csps_isphased(csps);
  uint8_t use_phased;
  uint8_t econ_flag = gParameters.IdleEconFlag;
  HAL_StatusTypeDef throttle_status = HAL_OK;
  GPIO_PinState clutch_pin;
  uint32_t clutch_time;
  uint32_t turns_count = csps_getturns();

  static uint32_t async_inject_time = 0;
  static uint32_t async_inject_last = 0;
  float inj_lag = gParameters.InjectionLag * 1000.0f;

  float phased_angles[ECU_CYLINDERS_COUNT];
  float non_phased_angles[ECU_CYLINDERS_COUNT_HALF];

  float knock_injection_correctives[ECU_CYLINDERS_COUNT];
  float knock_ignition_correctives[ECU_CYLINDERS_COUNT];
  float cy_corr_injection[ECU_CYLINDERS_COUNT];
  float cy_corr_ignition[ECU_CYLINDERS_COUNT];

  memcpy(knock_injection_correctives, gEcuCorrections.knock_injection_correctives, sizeof(knock_injection_correctives));
  memcpy(knock_ignition_correctives, gEcuCorrections.knock_ignition_correctives, sizeof(knock_ignition_correctives));
  memcpy(cy_corr_injection, table->cy_corr_injection, sizeof(cy_corr_injection));
  memcpy(cy_corr_ignition, table->cy_corr_ignition, sizeof(cy_corr_ignition));

#ifndef SIMULATION
  throttle_status = sens_get_throttle_position(&throttle);
#else

  pressure = gDebugMap;
  throttle = gDebugThrottle;
#endif

  map_status = sens_get_map_urgent(&pressure);
  if(map_status != HAL_OK)
    pressure = gParameters.ManifoldAirPressure;

#if defined(PRESSURE_ACCEPTION_FEATURE) && PRESSURE_ACCEPTION_FEATURE > 0
  if (found) {
    pressure_filtered += pressure;
    pressure_count++;
  } else {
    pressure_filtered = pressure;
    pressure_count = 1;
  }
#endif

  if(shiftEnabled) {
    clutch_pin = sens_get_clutch(&clutch_time);
    if(clutch_pin != clutch)
    {
      if(clutch_time > 500) {
        clutch = clutch_pin;
      }
    }
  }
  if(throttle_status != HAL_OK)
    shiftEnabled = 0;
  if(!shiftEnabled)
    clutch = GPIO_PIN_RESET;

  injector_channel = table->inj_channel;
  single_coil = gEcuParams.isSingleCoil;
  individual_coils = gEcuParams.isIndividualCoils;
  phased_mode = gEcuParams.phasedMode;
  use_phased = phased_mode != PhasedModeDisabled && is_phased;
  phased_injection = use_phased;
  phased_ignition = use_phased && individual_coils && !single_coil;
  phased_knock = use_phased;
  cy_count_injection = phased_injection ? ECU_CYLINDERS_COUNT : ECU_CYLINDERS_COUNT_HALF;
  cy_count_ignition = phased_ignition ? ECU_CYLINDERS_COUNT : ECU_CYLINDERS_COUNT_HALF;
  cy_count_knock = phased_knock ? ECU_CYLINDERS_COUNT : ECU_CYLINDERS_COUNT_HALF;
  angle_ignite_param = gParameters.IgnitionAdvance;
  injection_phase_by_end = table->is_fuel_phase_by_end;
  inj_phase_param = gParameters.InjectionPhase;
  inj_pulse = gParameters.InjectionPulse;

  gLocalParams.PhasedInjection = phased_injection;

  if(found != was_found) {
    was_found = found;
    last_start_triggered = 1;
    last_start = now;
    turns_count_last = turns_count;
  }

  if(last_start_triggered) {
    if(found && phased_mode == PhasedModeWithSensor && !is_phased) {
      if(DelayDiff(now, last_start) < 1000000 && turns_count - turns_count_last < 3) {
        start_allowed = 0;
      } else {
        last_start_triggered = 0;
      }
    } else {
      last_start_triggered = 0;
    }
  }

  if(single_coil) {
    angle_ignite_koff = 0.005f;
  } else if(!phased_ignition) {
    angle_ignite_koff = 0.01f;
  }

  //TODO: remove this filter when fixed the bug of incorrect injection pulses
  inj_phase_koff = diff * 0.005f;
  if(inj_phase_koff > 0.8f)
    inj_phase_koff = 0.8f;
  if(inj_phase_koff < 0.0001f)
    inj_phase_koff = 0.0001f;

  uspa_koff = diff * 0.002f;

  uspa = uspa_raw * uspa_koff + uspa * (1.0f - uspa_koff);
  r_uspa = 1.0f / uspa;
  angle_ignite = angle_ignite_param * angle_ignite_koff + angle_ignite * (1.0f - angle_ignite_koff);
  inj_phase = inj_phase_param * inj_phase_koff + inj_phase * (1.0f - inj_phase_koff);
  inj_phase_temp = inj_phase;

  if(single_coil) {
    time_sat = period_half * 0.65f;
    time_pulse = period_half * 0.35f;
  } else {
    time_sat = gParameters.IgnitionPulse;
    time_pulse = 2500;
  }

  if(phased_ignition) {
    angles_ignition_per_turn = 720.0f;
    for(int i = 0; i < cy_count_ignition; i++) {
      var = angle_ignite;
      var += cy_corr_ignition[i];
      var += knock_ignition_correctives[i];
      if(!saturated[i] && !ignited[i]) {
        var2 = anglesbeforeignite[i] - var + cy_ignition[i];
        if(var2 > 0) {
          cy_ignition[i] = var;
        }
      }
    }
  } else {
    angles_ignition_per_turn = 360.0f;
    for(int i = 0; i < ECU_CYLINDERS_COUNT_HALF; i++) {
      saturated[ECU_CYLINDERS_COUNT - 1 - i] = 1;
      ignited[ECU_CYLINDERS_COUNT - 1 - i] = 1;
      ignition_saturate_time[ECU_CYLINDERS_COUNT - 1 - i] = 0;
    }
    for(int i = 0; i < ECU_CYLINDERS_COUNT_HALF; i++) {
      var = angle_ignite;
      var += (cy_corr_ignition[i] + cy_corr_ignition[ECU_CYLINDERS_COUNT - 1 - i]) * 0.5f;
      var += (knock_ignition_correctives[i] + knock_ignition_correctives[ECU_CYLINDERS_COUNT - 1 - i]) * 0.5f;
      if(!saturated[i] && !ignited[i]) {
        var2 = anglesbeforeignite[i] - var + cy_ignition[i];
        if(var2 > 0) {
          cy_ignition[i] = var;
        }
      }
    }
  }

  if(phased_injection) {
    angles_injection_per_turn = 720.0f;
    for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
      cy_injection[i] = (cy_corr_injection[i] + 1.0f) * inj_pulse;
      cy_injection[i] *= knock_injection_correctives[i] + 1.0f;
    }
  } else {
    angles_injection_per_turn = 360.0f;
    if(inj_pulse < inj_lag) {
      inj_pulse = 0;
    } else {
      inj_pulse = (inj_pulse - inj_lag) * 0.5f;
    }
    for(int i = 0; i < ECU_CYLINDERS_COUNT_HALF; i++) {
      injection[ECU_CYLINDERS_COUNT - 1 - i] = 1;
      injected[ECU_CYLINDERS_COUNT - 1 - i] = 1;
      if(cy_corr_injection[i] != 0.0f || cy_corr_injection[ECU_CYLINDERS_COUNT - 1 - i] != 0.0f)
        cy_injection[i] = ((cy_corr_injection[i] + cy_corr_injection[ECU_CYLINDERS_COUNT - 1 - i]) * 0.5f + 1.0f) * inj_pulse;
      else cy_injection[i] = inj_pulse;
      if(fabsf(knock_injection_correctives[i]) > 0.0005f || fabsf(knock_injection_correctives[ECU_CYLINDERS_COUNT - 1 - i]) > 0.0005f)
        cy_injection[i] *= (knock_injection_correctives[i] + knock_injection_correctives[ECU_CYLINDERS_COUNT - 1 - i]) * 0.5f + 1.0f;
      cy_injection[i] += inj_lag;

      cy_injection[ECU_CYLINDERS_COUNT - 1 - i] = cy_injection[i];
    }
  }

  if(inj_was_phased != phased_injection) {
    inj_was_phased = phased_injection;
    for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
      oldanglesbeforeinject[i] = 0.0f;
      injection_time_prev[i] = 0.0f;
      enrichment_time_saturation[i] = 0.0f;
      enrichment_triggered[i] = 0;
    }
  }

  if(ign_was_phased != phased_ignition) {
    ign_was_phased = phased_ignition;
    for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
      oldanglesbeforeignite[i] = 0.0f;
      ignition_saturate_time[i] = 0;
      ignition_ignite_time[i] = 0;
      //ignition_ready[i] = 0;
    }
  }

  if(knock_was_phased != phased_knock) {
    knock_was_phased = phased_knock;
    knock_cylinder = -1;
    knock_busy = 0;
    for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
      knock_process[i] = 0;
    }
  }

  for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
    phased_angles[i] = csps_getphasedangle_cy(csps_isphased(csps), i, angle);
  }

  non_phased_angles[0] = csps_getangle14(csps);
  non_phased_angles[1] = csps_getangle23from14(non_phased_angles[0]);

  if(phased_ignition) {
    for(int i = 0; i < cy_count_ignition; i++) {
      angle_ignition[i] = phased_angles[i];
    }
  } else {
    angle_ignition[0] = non_phased_angles[0];
    angle_ignition[1] = non_phased_angles[1];
  }

  if(phased_knock) {
    for(int i = 0; i < cy_count_knock; i++) {
      angle_knock[i] = phased_angles[i];
    }
  } else {
    angle_knock[0] = non_phased_angles[0];
    angle_knock[1] = non_phased_angles[1];
  }

  if(phased_injection) {
    for(int i = 0; i < cy_count_injection; i++) {
      angle_injection[i] = phased_angles[i];
    }
  } else {
    angle_injection[0] = non_phased_angles[0];
    angle_injection[1] = non_phased_angles[1];
  }

  if(gLocalParams.EnrichmentTriggered) {
    for(int i = 0; i < cy_count_injection; i++)
      enrichment_triggered[i] = 1;
    gLocalParams.EnrichmentTriggered = 0;
  }

  if((found || ecu_async_injection_check() || async_inject_time || gIITest.StartedTime) && start_allowed && !gIgnCanShutdown)
  {
    IGN_NALLOW_GPIO_Port->BSRR = IGN_NALLOW_Pin << 16;
    gInjChPorts[injector_channel]->BSRR = gInjChPins[injector_channel];
    gInjChPorts[injector_channel ^ 1]->BSRR = gInjChPins[injector_channel ^ 1] << 16;

    //Ignition/Injection Test
    if(gIITest.StartedTime) {
      Knock_SetState(KnockStateHold);
      if(DelayDiff(now, gIITest.StartedTime) >= gIITest.Period) {
        gIITest.InjectionTriggered = 0;
        gIITest.CompletedCount++;
        if(gIITest.CompletedCount >= gIITest.Count) {
          memset(knock_process, 0, sizeof(knock_process));
          knock_busy = 0;
          Knock_SetState(KnockStateIntegrate);
          gIITest.StartedTime = 0;
          gIITest.IgnitionEnabled = 0;
          gIITest.InjectionEnabled = 0;
          return;
        } else {
          gIITest.StartedTime += gIITest.Period;
          if(!gIITest.StartedTime)
            gIITest.StartedTime++;
        }
      }

      if(DelayDiff(now, gIITest.StartedTime) < gIITest.IgnitionPulse) {
        if(single_coil) {
          if(gIITest.IgnitionEnabled) {
            ecu_coil_saturate(1, 0);
          }
        } else if(individual_coils) {
          for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
            if(((gIITest.IgnitionEnabled >> i) & 1)) {
              ecu_coil_saturate(ECU_CYLINDERS_COUNT, i);
            } else {
              ecu_coil_ignite(ECU_CYLINDERS_COUNT, i);
            }
          }
        } else {
          for(int i = 0; i < ECU_CYLINDERS_COUNT_HALF; i++) {
            if(((gIITest.IgnitionEnabled >> (ECU_CYLINDERS_COUNT - 1 - i)) & 1)) {
              ecu_coil_saturate(ECU_CYLINDERS_COUNT_HALF, i);
            } else {
              ecu_coil_ignite(ECU_CYLINDERS_COUNT_HALF, i);
            }
          }
        }
      } else {
        for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
          ecu_coil_ignite(ECU_CYLINDERS_COUNT, i);
        }
      }

      if(!gIITest.InjectionTriggered) {
        gIITest.InjectionTriggered = 1;
        for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
          if(((gIITest.InjectionEnabled >> i) & 1)) {
            ecu_inject(ECU_CYLINDERS_COUNT, i, gIITest.InjectionPulse);
          }
        }
      }
    } else {
      if(!async_inject_time || DelayDiff(now, async_inject_last) > async_inject_time) {
        async_inject_time = 0;
        if(ecu_async_injection_pull(&async_inject_time)) {
          if(async_inject_time > 0) {
            ecu_inject_async(async_inject_time);
            async_inject_time += inj_lag;
            async_inject_last = now;
          }
        }
      }
      if(found) {

        if(period < time_sat + time_pulse) {
          time_sat = period * ((float)time_sat / (float)(time_sat + time_pulse));
        }

        saturate = time_sat * r_uspa;
        inj_angle = inj_pulse * r_uspa;

        if(inj_angle < diff * r_uspa * 1.5f)
          inj_angle = diff * r_uspa * 1.5f;

        if(!injection_phase_by_end) {
          inj_phase_temp += inj_angle;
        }

        while(inj_phase_temp > angles_injection_per_turn * 0.5f) {
          inj_phase_temp -= angles_injection_per_turn;
        }

#if defined(PRESSURE_ACCEPTION_FEATURE) && PRESSURE_ACCEPTION_FEATURE > 0
        //Pressure detection part

        if(map_status == HAL_OK && throttle_status == HAL_OK) {
          throttle_abs_diff = fabsf(throttle - gLocalParams.TpsAcceptValue);
          throttle_rel_diff = (1.0f + throttle) / (1.0f + gLocalParams.TpsAcceptValue);
          if(throttle_rel_diff < 1.0f)
            throttle_rel_diff = 1.0f / throttle_rel_diff;

          if(throttle_abs_diff > 2.0f && throttle_rel_diff > 1.33f) {
            gLocalParams.TpsAcceptValue = throttle;
            gLocalParams.MapAcceptValue = pressure;
            gLocalParams.MapAcceptLast = now;
          }
        }

        for(int i = 0; i < ECU_CYLINDERS_COUNT_HALF; i++)
        {
          if(non_phased_angles[i] < pressure_measurement_angle)
            anglesbeforepressure[i] = -non_phased_angles[i] + pressure_measurement_angle;
          else
            anglesbeforepressure[i] = 360.0f - non_phased_angles[i] + pressure_measurement_angle;

          if(oldanglesbeforepressure[i] - anglesbeforepressure[i] > 0.0f && oldanglesbeforepressure[i] - anglesbeforepressure[i] > 180.0f)
            anglesbeforepressure[i] = oldanglesbeforepressure[i];

          if(anglesbeforepressure[i] - pressure_measurement_time < 0.0f)
          {
            if(!pressure_measurement[i])
            {
              pressure_measurement[i] = 1;

              if(map_status == HAL_OK) {
                if(!pressure_count) {
                  pressure_filtered = pressure;
                  pressure_count = 1;
                }
                gLocalParams.TpsAcceptValue = throttle;
                gLocalParams.MapAcceptValue = pressure_filtered / (float)pressure_count;
                gLocalParams.MapAcceptLast = now;
                pressure_filtered = 0;
                pressure_count = 0;
              }
            }
          }

          if(oldanglesbeforepressure[i] - anglesbeforepressure[i] < -90.0f)
          {
            pressure_measurement[i] = 0;
          }

          oldanglesbeforepressure[i] = anglesbeforepressure[i];
        }
#endif

        //Knock part
        if(running) {
          if(knock_busy && knock_process[knock_cylinder]) {
            if(angle_knock[knock_cylinder] < gKnockDetectStartAngle || angle_knock[knock_cylinder] >= gKnockDetectEndAngle) {
              Knock_SetState(KnockStateHold);
              knock_process[knock_cylinder] = 0;
              knock_busy = 0;
            }
          }
          if(!knock_busy) {
            for(int i = 0; i < cy_count_knock; i++) {
              if(angle_knock[i] >= gKnockDetectStartAngle && angle_knock[i] < gKnockDetectEndAngle) {
                if(knock_cylinder >= 0) {
                  knock = adc_get_voltage(AdcChKnock);
                  if(cy_count_knock == ECU_CYLINDERS_COUNT) {
                    gStatus.Knock.Voltages[knock_cylinder] = knock;
                    gStatus.Knock.Updated[knock_cylinder] = 1;
                    gStatus.Knock.Period[knock_cylinder] = DelayDiff(now, gStatus.Knock.LastTime[knock_cylinder]);
                    gStatus.Knock.LastTime[knock_cylinder] = now;

                    knock_lpf = (float)gStatus.Knock.Period[knock_cylinder] * 0.000001f;
                    gStatus.Knock.VoltagesLpf[knock_cylinder] = knock * knock_lpf + gStatus.Knock.VoltagesLpf[knock_cylinder] * (1.0f - knock_lpf);

                  } else if(cy_count_knock == ECU_CYLINDERS_COUNT_HALF) {
                    gStatus.Knock.Voltages[knock_cylinder] = knock;
                    gStatus.Knock.Voltages[ECU_CYLINDERS_COUNT - 1 - knock_cylinder] = knock;
                    gStatus.Knock.Updated[knock_cylinder] = 1;
                    gStatus.Knock.Updated[ECU_CYLINDERS_COUNT - 1 - knock_cylinder] = 1;
                    gStatus.Knock.Period[knock_cylinder] = DelayDiff(now, gStatus.Knock.LastTime[knock_cylinder]);
                    gStatus.Knock.Period[ECU_CYLINDERS_COUNT - 1 - knock_cylinder] = gStatus.Knock.Period[knock_cylinder];
                    gStatus.Knock.LastTime[knock_cylinder] = now;
                    gStatus.Knock.LastTime[ECU_CYLINDERS_COUNT - 1 - knock_cylinder] = now;

                    knock_lpf = (float)gStatus.Knock.Period[knock_cylinder] * 0.000001f;
                    gStatus.Knock.VoltagesLpf[knock_cylinder] = knock * knock_lpf + gStatus.Knock.VoltagesLpf[knock_cylinder] * (1.0f - knock_lpf);
                    gStatus.Knock.VoltagesLpf[ECU_CYLINDERS_COUNT - 1 - knock_cylinder] = gStatus.Knock.VoltagesLpf[knock_cylinder];
                  }
                }
                knock_busy = 1;
                knock_cylinder = i;
                knock_process[knock_cylinder] = 1;
                Knock_SetState(KnockStateIntegrate);
              }
            }
          }
        }

        //Ignition part
        for(int i = 0; i < cy_count_ignition; i++)
        {
#if defined(IGNITION_ACCEPTION_FEATURE) && IGNITION_ACCEPTION_FEATURE > 0
          cur_cy_ignition = ignition_accepted_angle[i];
#else
          cur_cy_ignition = cy_ignition[i];
#endif
          if(angle_ignition[i] < -cur_cy_ignition)
            anglesbeforeignite[i] = -angle_ignition[i] - cur_cy_ignition;
          else
            anglesbeforeignite[i] = angles_ignition_per_turn - angle_ignition[i] - cur_cy_ignition;

          if(anglesbeforeignite[i] - oldanglesbeforeignite[i] > 0.0f && anglesbeforeignite[i] - oldanglesbeforeignite[i] < 180.0f)
            anglesbeforeignite[i] = oldanglesbeforeignite[i];

          if(anglesbeforeignite[i] - saturate < 0.0f)
          {
            if(!saturated[i] && !ignited[i] && (ignition_ignite_time[i] == 0 || DelayDiff(now, ignition_ignite_time[i]) >= ignition_ignite[i]))
            {
              //if(cy_count_ignition == ECU_CYLINDERS_COUNT) {
              //  ignition_ready[i] = 1;
              //} else if(cy_count_ignition == ECU_CYLINDERS_COUNT_HALF) {
              //  ignition_ready[i] = 1;
              //  ignition_ready[ECU_CYLINDERS_COUNT - 1 - i] = 1;
              //}
              ignition_ignite_time[i] = 0;
              ignition_ignite[i] = 0;
              saturated[i] = 1;

              if(single_coil) {
                ecu_coil_saturate(1, 0);
              } else {
                shift_ign_act = !shiftEnabled || ecu_shift_ign_act(cy_count_ignition, i, clutch, rpm, throttle);
                cutoff_ign_act = ecu_cutoff_ign_act(cy_count_ignition, i, rpm);
                if(shift_ign_act && cutoff_ign_act)
                  ecu_coil_saturate(cy_count_ignition, i);
              }

              ignition_saturate_time[i] = now;
              ignition_saturate[i] = time_sat;
            }
          }

          if(ign_prepare[i] || oldanglesbeforeignite[i] - anglesbeforeignite[i] < -90.0f)
          {
            if((!ignited[i] && saturated[i]) || ign_prepare[i])
            {
              ign_prepare[i] = 1;
              if(ignition_saturate_time[i] == 0 || DelayDiff(now, ignition_saturate_time[i]) >= ignition_saturate[i]) {
                ign_prepare[i] = 0;
                ignited[i] = 1;
                saturated[i] = 0;
                ignition_saturate_time[i] = 0;
                ignition_saturate[i] = 0;

                if(single_coil) {
                  shift_ign_act = !shiftEnabled || ecu_shift_ign_act(cy_count_ignition, i, clutch, rpm, throttle);
                  cutoff_ign_act = ecu_cutoff_ign_act(cy_count_ignition, i, rpm);
                  if(shift_ign_act && cutoff_ign_act)
                    ecu_coil_ignite(1, 0);
                } else {
                  ecu_coil_ignite(cy_count_ignition, i);
                }
                gParameters.CylinderIgnitionBitmask ^= 1 << i;
                ignition_ignite[i] = time_pulse;
                ignition_ignite_time[i] = now;
              }
            }
          } else {
            ignited[i] = 0;
          }

          oldanglesbeforeignite[i] = anglesbeforeignite[i];
        }

        //Injection part
        for(int i = 0, i_inv; i < cy_count_injection; i++)
        {
          if(angle_injection[i] < inj_phase_temp)
            anglesbeforeinject[i] = -angle_injection[i] + inj_phase_temp;
          else
            anglesbeforeinject[i] = angles_injection_per_turn - angle_injection[i] + inj_phase_temp;

          if(oldanglesbeforeinject[i] - anglesbeforeinject[i] > 0.0f && oldanglesbeforeinject[i] - anglesbeforeinject[i] > 180.0f)
            anglesbeforeinject[i] = oldanglesbeforeinject[i];

          if(anglesbeforeinject[i] - inj_angle < 0.0f)
          {
            if(!injection[i])
            {
              injection[i] = 1;
              shift_inj_act = !shiftEnabled || ecu_shift_inj_act(cy_count_ignition, i, clutch, rpm, throttle);
              cutoff_inj_act = ecu_cutoff_inj_act(cy_count_ignition, i, rpm);
              if(/*ignition_ready[i] && */cutoff_inj_act && shift_inj_act && cy_injection[i] > 0.0f && !econ_flag) {
                if(cy_injection[i] > inj_lag * 1.1f) {
                  if(phased_injection) {
                    ecu_inject(ECU_CYLINDERS_COUNT, i, cy_injection[i]);
                  } else {
                    i_inv = ECU_CYLINDERS_COUNT - i - 1;

                    if(phased_mode == PhasedModeWithoutSensor) {
                      if(phase_detect_enabled != PhaseDetectDisabled && gParameters.IdleFlag && running) {
                        if(phase_detect_running_cycles < phjase_detect_cycles_wait) {
                          phase_detect_running_cycles++;
                        } else {
                          if(phase_detect_cycles < phjase_detect_cycles_threshold) {

                            if(phase_detect_enabled && (gPhaseDetectActive || i != 0)) {
                              gPhaseDetectActive = 1;
                              if(phase_detect_enabled == PhaseDetectByDisabling) {
                                cy_injection[3 - 1] = 0;
                                cy_injection[4 - 1] = 0;
                              }
                              else if(phase_detect_enabled == PhaseDetectByFueling) {
                                if((phase_detect_fueling & (1 << i))) {
                                  cy_injection[i] *= phjase_detect_fueling_multiplier;
                                  cy_injection[i_inv] /= phjase_detect_fueling_multiplier;
                                }
                                else {
                                  cy_injection[i] /= phjase_detect_fueling_multiplier;
                                  cy_injection[i_inv] *= phjase_detect_fueling_multiplier;
                                }
                                phase_detect_fueling ^= (1 << i);
                              }

                              phase_detect_cycles++;
                            }
                          } else if(phase_detect_cycles == phjase_detect_cycles_threshold) {
                            gPhaseDetectCompleted = 1;
                            phase_detect_cycles++;
                          }
                        }
                      }
                    }

                    ecu_inject(ECU_CYLINDERS_COUNT, i, cy_injection[i]);
                    ecu_inject(ECU_CYLINDERS_COUNT, i_inv, cy_injection[i_inv]);
                  }
                }
                injection_time_prev[i] = cy_injection[i] - inj_lag;
              }
              gLocalParams.ParametersAcceptLast = 0;
              density = ecu_get_air_density(pressure, gParameters.AirTemp);
              gLocalParams.InjectionDensityPressure = density;
              gParameters.CylinderInjectionBitmask ^= 1 << i;
              enrichment_triggered[i] = 0;

#if defined(IGNITION_ACCEPTION_FEATURE) && IGNITION_ACCEPTION_FEATURE > 0
              ignition_accepted_angle[i] = cy_ignition[i];
#endif
            }
          }

          if(oldanglesbeforeinject[i] - anglesbeforeinject[i] < -90.0f)
          {
            if(!injected[i] && injection[i])
            {
              //TODO: this is probably a bug, but works well, without extra pulses on duty cycle >= 1.0
              injection[i] = 0;
              injector_isenabled(i, &injected[i]);
              injected[i] ^= 1;
            }
          }
          else injected[i] = 0;

          if((angle_injection[i] >= 0 && angle_injection[i] >= inj_phase && angle_injection[i] < enrichment_end_injection_final_phase) ||
              (angle_injection[i] < 0 && angle_injection[i] >= inj_phase - 720.0f && angle_injection[i] < enrichment_end_injection_final_phase - 720.0f)) {
            if(enrichment_triggered[i]) {
              injection_time_diff = (cy_injection[i] - inj_lag) - injection_time_prev[i];
              injection_time_prev[i] += injection_time_diff;
              enrichment_time_saturation[i] += injection_time_diff;
            }
          } else {
            if(enrichment_time_saturation[i] > inj_lag && enrichment_end_injection_final_amount > 0.0f) {
              enrichment_time_saturation[i] *= enrichment_end_injection_final_amount;
              enrichment_time_saturation[i] += inj_lag;
              ecu_inject(cy_count_injection, i, enrichment_time_saturation[i]);
              enrichment_time_saturation[i] = 0;
              gLocalParams.InjectionDensityPressure = pressure;
              gLocalParams.ParametersAcceptLast = 0;

#if defined(IGNITION_ACCEPTION_FEATURE) && IGNITION_ACCEPTION_FEATURE > 0
              ignition_accepted_angle[i] = cy_ignition[i];
#endif
            }
          }
          enrichment_triggered[i] = 0;

          oldanglesbeforeinject[i] = anglesbeforeinject[i];
        }
      }
    }
  } else {
    for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {

      oldanglesbeforeignite[i] = 0.0f;
      ignition_saturate_time[i] = 0;
      ignition_ignite_time[i] = 0;
      injection_time_prev[i] = 0;
      enrichment_time_saturation[i] = 0;
      enrichment_triggered[i] = 0;
      saturated[i] = 0;
      ignited[i] = 1;
      injection[i] = 0;
      injected[i] = 0;
      //ignition_ready[i] = 0;
    }


#if defined(PRESSURE_ACCEPTION_FEATURE) && PRESSURE_ACCEPTION_FEATURE > 0
    for(int i = 0; i < ECU_CYLINDERS_COUNT_HALF; i++) {
      pressure_measurement[i] = 0;
      oldanglesbeforepressure[i] = 0.0f;
    }
#endif

#if defined(IGNITION_ACCEPTION_FEATURE) && IGNITION_ACCEPTION_FEATURE > 0
    for(int i = 0; i < ECU_CYLINDERS_COUNT_HALF; i++) {
      ignition_accepted_angle[i] = angle_ignite_param;
    }
#endif

    IGN_NALLOW_GPIO_Port->BSRR = IGN_NALLOW_Pin;
    for(int i = 0; i < ITEMSOF(gInjChPins); i++)
      gInjChPorts[i]->BSRR = gInjChPins[i] << 16;
    for(int i = 0; i < ITEMSOF(gIgnPorts); i++)
      gIgnPorts[i]->BSRR = gIgnPins[i] << 16;
    for(int i = 0; i < ITEMSOF(gInjPorts); i++)
      gInjPorts[i]->BSRR = gInjPins[i];

    gParameters.CylinderIgnitionBitmask = 0;
    gParameters.CylinderInjectionBitmask = 0;

    phase_need_clean = 1;

  }

  if(phase_detect_running_cycles && phased_mode != PhasedModeWithoutSensor) {
    phase_need_clean = 1;
  }


  if(phase_need_clean) {
    memset(gPhaseDetectAccelerations, 0, sizeof(gPhaseDetectAccelerations));
    memset(gPhaseDetectAccelerationsCount, 0, sizeof(gPhaseDetectAccelerationsCount));
    gPhaseDetectActive = 0;
    gPhaseDetectCompleted = 0;
    phase_detect_cycles = 0;
    phase_detect_running_cycles = 0;
    phase_detect_fueling = 0;
  }

  if(!running || !found) {
    Knock_SetState(KnockStateHold);
    knock_busy = 0;

    for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
      knock_process[i] = 0;
    }
  }
}

static void ecu_fuelpump_process(void)
{
  static uint8_t ftime = 1;
  static uint32_t active_last = 0;
  static uint8_t active = 0;
  static uint8_t was_rotating = 1;
  uint32_t now = Delay_Tick;
  uint8_t rotates = csps_isrotates();
  uint8_t can_shutdown = gIgnCanShutdown;
  uint8_t time_to_last;

  if(ftime) {
    active_last = now;
    ftime = 0;
  }

  time_to_last = DelayDiff(now, active_last) < FUEL_PUMP_TIMEOUT;

  if(gForceParameters.Enable.FuelPumpRelay) {
    active = gForceParameters.FuelPumpRelay > 0;
    active_last = now;
  } else {
    if(((was_rotating && time_to_last) || rotates) && !can_shutdown) {
      if(rotates) {
        active_last = now;
        was_rotating = 1;
      }
      active = 1;
    } else {
      active = 0;
      was_rotating = 0;
    }
#if defined(FUEL_PUMP_ON_INJ_CH1_ONLY) && FUEL_PUMP_ON_INJ_CH1_ONLY == 1
    if(gParameters.InjectorChannel != InjectorChannel1) {
      active = 0;
      was_rotating = 0;
    }
#endif
  }

  if(active) {
    out_set_fuelpump(GPIO_PIN_SET);
  } else {
    out_set_fuelpump(GPIO_PIN_RESET);
  }
}

static void ecu_fan_process(void)
{
  static uint32_t running_last = 0;
  static uint32_t high_start_time = 0;

  uint32_t table_number = gParameters.CurrentTable;
  sEcuTable *table = &gEcuTable[table_number];
  HAL_StatusTypeDef status;
  uint8_t fan_state;
  uint8_t force_enabled = gForceParameters.Enable.FanRelay || gForceParameters.Enable.FanSwitch;
  GPIO_PinState force = sens_get_fan_force_switch(NULL);
  GPIO_PinState fan_pin_state = out_get_fan(NULL);
  GPIO_PinState switch_state = out_get_fan_switch(NULL);
  GPIO_PinState starter_state = GPIO_PIN_RESET;
  uint8_t can_shutdown = gIgnCanShutdown;

  GPIO_PinState out_fan_state = fan_pin_state;
  GPIO_PinState out_fan_sw_state = switch_state;
  GPIO_PinState out_fan_sw_state_temp = switch_state;

  sMathInterpolateInput ipEngineTemp = {0};
  sMathInterpolateInput ipSpeed = {0};

#ifdef SIMULATION
  force = GPIO_PIN_RESET;
#endif

  float fan_advance_control;
  float fan_advance_control_low = table->fan_advance_control_low;
  float fan_advance_control_mid = table->fan_advance_control_mid;
  float fan_advance_control_high = table->fan_advance_control_high;

  status = gStatus.Sensors.Struct.EngineTemp;

  ipEngineTemp = math_interpolate_input(gParameters.EngineTemp, table->engine_temps, table->engine_temp_count);
  ipSpeed = math_interpolate_input(gParameters.Speed, table->speeds, table->speeds_count);
  fan_advance_control = math_interpolate_2d(ipSpeed, ipEngineTemp, TABLE_SPEEDS_MAX, table->fan_advance_control);

  uint8_t running = csps_isrunning();
  uint8_t rotates = csps_isrotates();
  uint32_t now = Delay_Tick;

  if(!now)
    now += 1;

  if(running) {
    running_last = now;
  }

  if(running_last > 0 && DelayDiff(now, running_last) > FAN_TIMEOUT) {
    running_last = 0;
  }

  //fan_advance_control = gParameters.EngineTemp;
  //fan_advance_control_low = gEcuParams.fanLowTemperature;
  //fan_advance_control_mid = gEcuParams.fanMidTemperature;
  //fan_advance_control_high = gEcuParams.fanHighTemperature;

  if(!force_enabled) {
    if(/* fan_pin_state && */switch_state) {
      fan_state = 2;
    } else if(fan_pin_state && !switch_state) {
      fan_state = 1;
    } else if(!fan_pin_state && !switch_state) {
      fan_state = 0;
    } else {
      out_fan_state = GPIO_PIN_SET;
      out_fan_sw_state = GPIO_PIN_SET;
      out_fan_sw_state_temp = GPIO_PIN_SET;
      fan_state = 2;
      return;
    }
  }

  if(force_enabled) {
    if(gForceParameters.Enable.FanRelay)
      out_fan_state = gForceParameters.FanRelay > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET;
    if(gForceParameters.Enable.FanSwitch)
      out_fan_sw_state_temp = gForceParameters.FanSwitch > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET;
    else out_fan_sw_state_temp = out_fan_sw_state;
  } else if((!running && rotates && !can_shutdown) || !running_last || starter_state == GPIO_PIN_SET) {
    out_fan_state = GPIO_PIN_RESET;
    out_fan_sw_state_temp = GPIO_PIN_RESET;
  } else if(status != HAL_OK || force == GPIO_PIN_SET) {
    out_fan_state  = GPIO_PIN_SET;
    out_fan_sw_state_temp = GPIO_PIN_SET;
    fan_state = 2;
  } else if(fan_state == 0) {
    if(fan_advance_control > fan_advance_control_mid) {
      out_fan_state = GPIO_PIN_SET;
      out_fan_sw_state_temp = GPIO_PIN_RESET;
    } else {
      out_fan_state = GPIO_PIN_RESET;
      out_fan_sw_state_temp = GPIO_PIN_RESET;
    }
  } else if(fan_state == 1) {
    if(fan_advance_control > fan_advance_control_high) {
      out_fan_state = GPIO_PIN_SET;
      out_fan_sw_state_temp = GPIO_PIN_SET;
    } else if(fan_advance_control < fan_advance_control_low) {
      out_fan_state = GPIO_PIN_RESET;
      out_fan_sw_state_temp = GPIO_PIN_RESET;
    }
  } else if(fan_state == 2) {
    if(fan_advance_control < fan_advance_control_mid) {
      out_fan_state = GPIO_PIN_SET;
      out_fan_sw_state_temp = GPIO_PIN_RESET;
    }
  }

  if(out_fan_state == GPIO_PIN_SET && out_fan_sw_state == GPIO_PIN_RESET) {
    if(high_start_time == 0 || DelayDiff(now, high_start_time) > FAN_HIGH_SWITCH_TIME) {
      high_start_time = 0;
      out_fan_sw_state = out_fan_sw_state_temp;
    }
  } else {
    high_start_time = now;
    out_fan_sw_state = out_fan_sw_state_temp;
  }

  out_set_fan(out_fan_state);
  out_set_fan_switch(out_fan_sw_state);
}

#define CHECK_STATUS(iserror, cod, link) \
  if((link)) { \
    gCheckBitmap[cod >> 3] |= 1 << (cod & 7); \
    iserror |= 1; \
  }

static void ecu_checkengine_loop(void)
{
  static uint32_t hal_error_last = 0;
  static uint8_t was_error = 0;
  uint32_t hal_now = HAL_GetTick();
  uint8_t running = csps_isrunning();
  uint8_t iserror = 0;
  uint8_t status_reset = gStatusReset;
  GPIO_PinState ign_status = sens_get_ign(NULL);

  memset(gCheckBitmap, 0, sizeof(gCheckBitmap));

  CHECK_STATUS(iserror, CheckFlashLoadFailure, gStatus.Flash.Struct.Load != HAL_OK);
  CHECK_STATUS(iserror, CheckFlashSaveFailure, gStatus.Flash.Struct.Save != HAL_OK);
  CHECK_STATUS(iserror, CheckFlashInitFailure, gStatus.Flash.Struct.Init != HAL_OK);
  CHECK_STATUS(iserror, CheckBkpsramSaveFailure, gStatus.Bkpsram.Struct.CriticalSave != HAL_OK || gStatus.Bkpsram.Struct.CorrsSave != HAL_OK)
#ifdef DEBUG
  CHECK_STATUS(iserror, CheckBkpsramLoadFailure, gStatus.Bkpsram.Struct.CriticalLoad != HAL_OK || gStatus.Bkpsram.Struct.CorrsLoad != HAL_OK);
#endif

  CHECK_STATUS(iserror, CheckSensorMapFailure, gStatus.Sensors.Struct.Map != HAL_OK);
  CHECK_STATUS(iserror, CheckSensorKnockFailure, gStatus.Sensors.Struct.Knock != HAL_OK);
  CHECK_STATUS(iserror, CheckSensorCspsFailure, gStatus.Sensors.Struct.Csps != HAL_OK);
  CHECK_STATUS(iserror, CheckSensorTspsFailure, gStatus.Sensors.Struct.Tsps != HAL_OK);
  CHECK_STATUS(iserror, CheckSensorAirTempFailure, gStatus.Sensors.Struct.AirTemp != HAL_OK);
  CHECK_STATUS(iserror, CheckSensorEngineTempFailure, gStatus.Sensors.Struct.EngineTemp != HAL_OK);
  CHECK_STATUS(iserror, CheckSensorTPSFailure, gStatus.Sensors.Struct.ThrottlePos != HAL_OK);
  CHECK_STATUS(iserror, CheckSensorRefVoltageFailure, gStatus.Sensors.Struct.ReferenceVoltage != HAL_OK);
  CHECK_STATUS(iserror, CheckSensorPwrVoltageFailure, gStatus.Sensors.Struct.PowerVoltage != HAL_OK);
  CHECK_STATUS(iserror, CheckSensorLambdaFailure, gStatus.Sensors.Struct.Lambda != HAL_OK);
  CHECK_STATUS(iserror, CheckOutputDriverFailure, gStatus.OutputStatus != HAL_OK);

  CHECK_STATUS(iserror, CheckCanInitFailure, gStatus.CanInitStatus != HAL_OK);
  CHECK_STATUS(iserror, CheckCanTestFailure, gStatus.CanTestStatus != HAL_OK);
  CHECK_STATUS(iserror, CheckKlineProtocolFailure, gStatus.KlineProtocolStatus != HAL_OK);
  CHECK_STATUS(iserror, CheckKlineLoopbackFailure, gStatus.KlineLoopbackStatus != HAL_OK);

#if defined(INJECTORS_ON_INJ_CH1_ONLY) && INJECTORS_ON_INJ_CH1_ONLY == 1
  if(gParameters.InjectorChannel == InjectorChannel1) {
    CHECK_STATUS(iserror, CheckInjector4OpenCircuit, gStatus.OutputDiagnostic.Injectors.Diagnostic.Data.InjCy4 == OutputDiagOpenCircuit);
    CHECK_STATUS(iserror, CheckInjector3OpenCircuit, gStatus.OutputDiagnostic.Injectors.Diagnostic.Data.InjCy3 == OutputDiagOpenCircuit);
    CHECK_STATUS(iserror, CheckInjector2OpenCircuit, gStatus.OutputDiagnostic.Injectors.Diagnostic.Data.InjCy2 == OutputDiagOpenCircuit);
    CHECK_STATUS(iserror, CheckInjector1OpenCircuit, gStatus.OutputDiagnostic.Injectors.Diagnostic.Data.InjCy1 == OutputDiagOpenCircuit);
  }
#endif

  CHECK_STATUS(iserror, CheckInjector4ShortToBatOrOverheat, gStatus.OutputDiagnostic.Injectors.Diagnostic.Data.InjCy4 == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(iserror, CheckInjector4ShortToGND, gStatus.OutputDiagnostic.Injectors.Diagnostic.Data.InjCy4 == OutputDiagShortToGnd);
  CHECK_STATUS(iserror, CheckInjector3ShortToBatOrOverheat, gStatus.OutputDiagnostic.Injectors.Diagnostic.Data.InjCy3 == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(iserror, CheckInjector3ShortToGND, gStatus.OutputDiagnostic.Injectors.Diagnostic.Data.InjCy3 == OutputDiagShortToGnd);
  CHECK_STATUS(iserror, CheckInjector2ShortToBatOrOverheat, gStatus.OutputDiagnostic.Injectors.Diagnostic.Data.InjCy2 == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(iserror, CheckInjector2ShortToGND, gStatus.OutputDiagnostic.Injectors.Diagnostic.Data.InjCy2 == OutputDiagShortToGnd);
  CHECK_STATUS(iserror, CheckInjector1ShortToBatOrOverheat, gStatus.OutputDiagnostic.Injectors.Diagnostic.Data.InjCy1 == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(iserror, CheckInjector1ShortToGND, gStatus.OutputDiagnostic.Injectors.Diagnostic.Data.InjCy1 == OutputDiagShortToGnd);
  CHECK_STATUS(iserror, CheckInjectorCommunicationFailure, gStatus.OutputDiagnostic.Injectors.Availability != HAL_OK);

  //CHECK_STATUS(iserror, CheckCheckEngineOpenCirtuit, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.CheckEngine == OutputDiagOpenCircuit);
  CHECK_STATUS(iserror, CheckCheckEngineShortToBatOrOverheat, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.CheckEngine == OutputDiagShortToBatOrOvertemp);
  //CHECK_STATUS(iserror, CheckCheckEngineShortToGND, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.CheckEngine == OutputDiagShortToGnd);
  //CHECK_STATUS(iserror, CheckSpeedMeterOpenCirtuit, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.Speedmeeter == OutputDiagOpenCircuit);
  CHECK_STATUS(iserror, CheckSpeedMeterShortToBatOrOverheat, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.Speedmeeter == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(iserror, CheckSpeedMeterShortToGND, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.Speedmeeter == OutputDiagShortToGnd);
  //CHECK_STATUS(iserror, CheckTachometerOpenCirtuit, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.Tachometer == OutputDiagOpenCircuit);
  CHECK_STATUS(iserror, CheckTachometerShortToBatOrOverheat, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.Tachometer == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(iserror, CheckTachometerShortToGND, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.Tachometer == OutputDiagShortToGnd);
  CHECK_STATUS(iserror, CheckFuelPumpOpenCirtuit, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.FuelPumpRelay == OutputDiagOpenCircuit);
  CHECK_STATUS(iserror, CheckFuelPumpShortToBatOrOverheat, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.FuelPumpRelay == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(iserror, CheckFuelPumpShortToGND, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.FuelPumpRelay == OutputDiagShortToGnd);
  CHECK_STATUS(iserror, CheckOutputs1CommunicationFailure, gStatus.OutputDiagnostic.Outs1.Availability != HAL_OK);

  CHECK_STATUS(iserror, CheckOutIgnOpenCirtuit, gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.OutIgn == OutputDiagOpenCircuit);
  CHECK_STATUS(iserror, CheckOutIgnShortToBatOrOverheat, gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.OutIgn == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(iserror, CheckOutIgnShortToGND, gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.OutIgn == OutputDiagShortToGnd);
  CHECK_STATUS(iserror, CheckFanSwitchOpenCirtuit, gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.FanSwitch == OutputDiagOpenCircuit);
  CHECK_STATUS(iserror, CheckFanSwitchShortToBatOrOverheat, gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.FanSwitch == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(iserror, CheckFanSwitchShortToGND, gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.FanSwitch == OutputDiagShortToGnd);
  //CHECK_STATUS(iserror, CheckStarterRelayOpenCirtuit, gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.StarterRelay == OutputDiagOpenCircuit);
  CHECK_STATUS(iserror, CheckStarterRelayShortToBatOrOverheat, gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.StarterRelay == OutputDiagShortToBatOrOvertemp);
  //CHECK_STATUS(iserror, CheckStarterRelayShortToGND, gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.StarterRelay == OutputDiagShortToGnd);
  CHECK_STATUS(iserror, CheckFanRelayOpenCirtuit, gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.FanRelay == OutputDiagOpenCircuit);
  CHECK_STATUS(iserror, CheckFanRelayShortToBatOrOverheat, gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.FanRelay == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(iserror, CheckFanRelayShortToGND, gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.FanRelay == OutputDiagShortToGnd);
  CHECK_STATUS(iserror, CheckOutputs2CommunicationFailure, gStatus.OutputDiagnostic.Outs2.Availability != HAL_OK);

  CHECK_STATUS(iserror, CheckIdleValveFailure, gStatus.OutputDiagnostic.IdleValvePosition.Status != HAL_OK);
  CHECK_STATUS(iserror, CheckIdleValveDriverFailure, gStatus.IdleValvePosition != HAL_OK);
  CHECK_STATUS(iserror, CheckInjectionUnderflow, gStatus.InjectionUnderflow.is_error && gStatus.InjectionUnderflow.error_time > 1000);
  CHECK_STATUS(iserror, CheckAdcFailure, gStatus.AdcStatus != HAL_OK);

  if(gEcuParams.useLambdaSensor) {
    CHECK_STATUS(iserror, CheckLambdaCommunicationFailure, gStatus.O2Status != HAL_OK);
    CHECK_STATUS(iserror, CheckLambdaVMShortToBat, gStatus.O2Diagnostic.VM == O2DiagShortToBat);
    //CHECK_STATUS(iserror, CheckLambdaVMLowBattery, gStatus.O2Diagnostic.VM == O2DiagNoPower);
    CHECK_STATUS(iserror, CheckLambdaVMShortToGND, gStatus.O2Diagnostic.VM == O2DiagShortToGnd);
    CHECK_STATUS(iserror, CheckLambdaUNShortToBat, gStatus.O2Diagnostic.UN == O2DiagShortToBat);
    //CHECK_STATUS(iserror, CheckLambdaUNLowBattery, gStatus.O2Diagnostic.UN == O2DiagNoPower);
    CHECK_STATUS(iserror, CheckLambdaUNShortToGND, gStatus.O2Diagnostic.UN == O2DiagShortToGnd);
    CHECK_STATUS(iserror, CheckLambdaIAIPShortToBat, gStatus.O2Diagnostic.IAIP == O2DiagShortToBat);
    //CHECK_STATUS(iserror, CheckLambdaIAIPLowBattery, gStatus.O2Diagnostic.IAIP == O2DiagNoPower);
    CHECK_STATUS(iserror, CheckLambdaIAIPShortToGND, gStatus.O2Diagnostic.IAIP == O2DiagShortToGnd);
    CHECK_STATUS(iserror, CheckLambdaDIAHGDShortToBat, gStatus.O2Diagnostic.DIAHGD == O2DiagShortToBat);
    CHECK_STATUS(iserror, CheckLambdaDIAHGDOpenCirtuit, gStatus.O2Diagnostic.DIAHGD == O2DiagNoPower);
    CHECK_STATUS(iserror, CheckLambdaDIAHGDShortToGND, gStatus.O2Diagnostic.DIAHGD == O2DiagShortToGnd);
    CHECK_STATUS(iserror, CheckLambdaTemperatureFailure, gStatus.O2TemperatureStatus != HAL_OK);
    CHECK_STATUS(iserror, CheckLambdaHeaterFailure, gStatus.O2HeaterStatus != HAL_OK);
    CHECK_STATUS(iserror, CheckLambdaDIAHGDShortToGND, gStatus.O2Diagnostic.DIAHGD == O2DiagShortToGnd);
    CHECK_STATUS(iserror, CheckEngineLeanMixture, gStatus.LeanMixture.is_error && gStatus.LeanMixture.error_time > 5000);
    CHECK_STATUS(iserror, CheckEngineRichMixture, gStatus.RichMixture.is_error && gStatus.RichMixture.error_time > 5000);
    CHECK_STATUS(iserror, CheckEngineLeanIdleMixture, gStatus.LeanIdleMixture.is_error && gStatus.LeanIdleMixture.error_time > 5000);
    CHECK_STATUS(iserror, CheckEngineRichIdleMixture, gStatus.RichIdleMixture.is_error && gStatus.RichIdleMixture.error_time > 5000);
  }
  if(gEcuParams.useKnockSensor) {
    CHECK_STATUS(iserror, CheckKnockDetonationFound, (gStatus.Knock.GeneralStatus & KnockStatusStrongDedonation) > 0);
    CHECK_STATUS(iserror, CheckKnockLowNoiseLevel, (gStatus.Knock.GeneralStatus & KnockStatusLowNoise) > 0);
  }
  if(gEcuParams.phasedMode == PhasedModeWithSensor) {
    CHECK_STATUS(iserror, CheckTspsDesynchronized, gStatus.Tsps.sync_error && gStatus.Tsps.sync_error_time > 100);
  }
  CHECK_STATUS(iserror, CheckSensorMapTpsMismatch, gStatus.MapTpsRelation.is_error && gStatus.MapTpsRelation.error_time > 5000);

  CHECK_STATUS(iserror, CheckNoOilPressure, gStatus.OilPressure.is_error);
  CHECK_STATUS(iserror, CheckNoBatteryCharge, gStatus.BatteryCharge.is_error);

  gDiagErrors.Bits.csps_error = gStatus.Sensors.Struct.Csps != HAL_OK;

  gDiagErrors.Bits.csps_error = gStatus.Sensors.Struct.Csps != HAL_OK;
  gDiagErrors.Bits.not_used1 = 0;
  gDiagErrors.Bits.eeprom_error = gStatus.Flash.Byte > 0;
  gDiagErrors.Bits.lambda_heater = gStatus.O2Diagnostic.DIAHGD > 0;
  gDiagErrors.Bits.tsps_error = gStatus.Sensors.Struct.Tsps != HAL_OK;
  gDiagErrors.Bits.reset_error = 0;
  gDiagErrors.Bits.ram_error = 0;
  gDiagErrors.Bits.flash_error = 0;

  gDiagErrors.Bits.low_voltage = gSharedParameters.PowerVoltage < 7.0f;
  gDiagErrors.Bits.not_used2 = 0;
  gDiagErrors.Bits.not_used3 = 0;
  gDiagErrors.Bits.engine_temp_low = gStatus.Sensors.Struct.EngineTemp != HAL_OK;
  gDiagErrors.Bits.lambda_low = gStatus.Sensors.Struct.Lambda != HAL_OK;
  gDiagErrors.Bits.tps_low = gStatus.Sensors.Struct.ThrottlePos != HAL_OK;
  gDiagErrors.Bits.maf_low = gStatus.Sensors.Struct.Map != HAL_OK;
  gDiagErrors.Bits.low_noise = (gStatus.Knock.GeneralStatus & KnockStatusLowNoise) > 0;

  gDiagErrors.Bits.high_voltage = gSharedParameters.PowerVoltage > 16.0f;
  gDiagErrors.Bits.not_used4 = 0;
  gDiagErrors.Bits.not_used5 = 0;
  gDiagErrors.Bits.engine_temp_high = gStatus.Sensors.Struct.EngineTemp != HAL_OK;
  gDiagErrors.Bits.lambda_high = gStatus.Sensors.Struct.Lambda != HAL_OK;
  gDiagErrors.Bits.tps_high = gStatus.Sensors.Struct.ThrottlePos != HAL_OK;
  gDiagErrors.Bits.maf_high = gStatus.Sensors.Struct.Map != HAL_OK;
  gDiagErrors.Bits.high_noise = (gStatus.Knock.GeneralStatus & KnockStatusDedonation) > 0;

  gDiagErrors.Bits.knock_error = gStatus.Sensors.Struct.Knock != HAL_OK;
  gDiagErrors.Bits.no_immo = 0;
  gDiagErrors.Bits.not_used6 = 0;
  gDiagErrors.Bits.lambda_no_activity = gStatus.Sensors.Struct.Lambda != HAL_OK;
  gDiagErrors.Bits.lambda_no_on_lean = gStatus.Sensors.Struct.Lambda != HAL_OK;
  gDiagErrors.Bits.lambda_no_on_rich = gStatus.Sensors.Struct.Lambda != HAL_OK;
  gDiagErrors.Bits.speed_error = 0;
  gDiagErrors.Bits.idle_error = gStatus.IdleValvePosition != HAL_OK || gStatus.OutputDiagnostic.IdleValvePosition.Status != HAL_OK;

  if(iserror) {
    was_error = 1;
    hal_error_last = hal_now;
  }

  if(was_error) {
    if(HAL_DelayDiff(hal_now, hal_error_last) < 60000) {
      iserror = 1;
    } else {
      was_error = 0;
    }
  }

  for(int i = 0; i < CHECK_BITMAP_SIZE; i++) {
    gEcuCriticalBackup.CheckBitmapRecorded[i] |= gCheckBitmap[i];
  }

  gEcuIsError = iserror;

  if(status_reset) {
    memset(&gStatus, 0, sizeof(gStatus));
    memset(&gCheckBitmap, 0, sizeof(gCheckBitmap));
    memset(&gEcuCriticalBackup.CheckBitmapRecorded, 0, sizeof(gEcuCriticalBackup.CheckBitmapRecorded));
    gEcuIsError = 0;
    was_error = 0;
    iserror = 0;
    gStatusReset = 0;
  }

  if(gForceParameters.Enable.CheckEngine) {
    out_set_checkengine(gForceParameters.CheckEngine > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    was_error = 0;
    iserror = 0;
  } else {

    if(ign_status == GPIO_PIN_SET && (!running || was_error)) {
      out_set_checkengine(GPIO_PIN_SET);
    } else {
      out_set_checkengine(GPIO_PIN_RESET);
    }
  }
}

static void ecu_drag_process(void)
{
  uint32_t now = Delay_Tick;
  sCspsData csps = csps_data();
  float rpm = csps_getrpm(csps);
  float speed = speed_getspeed();
  if(Drag.Ready)
  {
    if(Drag.Completed)
    {
      Drag.StopTime = 0;
      Drag.Ready = 0;
      Drag.Started = 0;
    }
    else
    {
      if(Drag.Started)
      {
        if(DelayDiff(now, Drag.TimeLast) >= DRAG_POINTS_DISTANCE)
        {
          Drag.TimeLast = now;
          if(Drag.PointsCount < DRAG_MAX_POINTS)
          {
            Drag.StopTime = now;
            Drag.Points[Drag.PointsCount].RPM = rpm;
            Drag.Points[Drag.PointsCount].Speed = speed;
            Drag.Points[Drag.PointsCount].Acceleration = gSharedParameters.Acceleration;
            Drag.Points[Drag.PointsCount].Pressure = gSharedParameters.ManifoldAirPressure;
            Drag.Points[Drag.PointsCount].Ignition = gSharedParameters.IgnitionAdvance;
            Drag.Points[Drag.PointsCount].Mixture = gSharedParameters.FuelRatio;
            Drag.Points[Drag.PointsCount].CycleAirFlow = gSharedParameters.CyclicAirFlow;
            Drag.Points[Drag.PointsCount].MassAirFlow = gSharedParameters.MassAirFlow;
            Drag.Points[Drag.PointsCount].Throttle = gSharedParameters.ThrottlePosition;
            Drag.Points[Drag.PointsCount].Time = DelayDiff(now, Drag.StartTime);
            Drag.PointsCount++;

            if(Drag.FromSpeed < Drag.ToSpeed)
            {
              if(speed >= Drag.ToSpeed)
              {
                Drag.Started = 0;
                Drag.Status = 0;
                Drag.Ready = 0;
                Drag.Completed = 1;
                Drag.TimeLast = 0;
              }
              else if(speed <= Drag.FromSpeed || speed <= 0.01f)
              {
                Drag.Started = 0;
                Drag.Status = 4;
                Drag.Ready = 0;
                Drag.Completed = 1;
                Drag.TimeLast = 0;
              }
            }
            else if(Drag.FromSpeed > Drag.ToSpeed)
            {
              if(speed <= Drag.ToSpeed)
              {
                Drag.Started = 0;
                Drag.Status = 0;
                Drag.Ready = 0;
                Drag.Completed = 1;
                Drag.TimeLast = 0;
              }
              else if(speed > Drag.FromSpeed + 3.0f)
              {
                Drag.Started = 0;
                Drag.Status = 4;
                Drag.Ready = 0;
                Drag.Completed = 1;
                Drag.TimeLast = 0;
              }
            }
          }
          else
          {
            Drag.Started = 0;
            Drag.Status = 5;
            Drag.Ready = 0;
            Drag.Completed = 1;
            Drag.TimeLast = 0;
          }
        }
      }
      else
      {
        if(Drag.FromSpeed < Drag.ToSpeed)
        {
          if(speed > Drag.FromSpeed)
          {
            if(Drag.TimeLast != 0)
            {
              Drag.TimeLast = now;
              Drag.Started = 1;
              Drag.Status = 0;
              Drag.PointsCount = 0;
              Drag.StartTime = now;
              Drag.StopTime = now;
              Drag.Points[Drag.PointsCount].RPM = rpm;
              Drag.Points[Drag.PointsCount].Speed = speed;
              Drag.Points[Drag.PointsCount].Acceleration = gSharedParameters.Acceleration;
              Drag.Points[Drag.PointsCount].Pressure = gSharedParameters.ManifoldAirPressure;
              Drag.Points[Drag.PointsCount].Ignition = gSharedParameters.IgnitionAdvance;
              Drag.Points[Drag.PointsCount].Mixture = gSharedParameters.FuelRatio;
              Drag.Points[Drag.PointsCount].CycleAirFlow = gSharedParameters.CyclicAirFlow;
              Drag.Points[Drag.PointsCount].MassAirFlow = gSharedParameters.MassAirFlow;
              Drag.Points[Drag.PointsCount].Throttle = gSharedParameters.ThrottlePosition;
              Drag.Points[Drag.PointsCount].Time = DelayDiff(now, Drag.StartTime);
              Drag.PointsCount++;
            }
          }
          else
          {
            Drag.TimeLast = now;
          }
        }
        else if(Drag.FromSpeed > Drag.ToSpeed)
        {
          if(speed <= Drag.FromSpeed)
          {
            if(Drag.TimeLast != 0)
            {
              Drag.TimeLast = now;
              Drag.Started = 1;
              Drag.PointsCount = 0;
              Drag.StartTime = now;
              Drag.StopTime = now;
              Drag.Points[Drag.PointsCount].RPM = rpm;
              Drag.Points[Drag.PointsCount].Speed = speed;
              Drag.Points[Drag.PointsCount].Acceleration = gSharedParameters.Acceleration;
              Drag.Points[Drag.PointsCount].Pressure = gSharedParameters.ManifoldAirPressure;
              Drag.Points[Drag.PointsCount].Ignition = gSharedParameters.IgnitionAdvance;
              Drag.Points[Drag.PointsCount].Mixture = gSharedParameters.FuelRatio;
              Drag.Points[Drag.PointsCount].CycleAirFlow = gSharedParameters.CyclicAirFlow;
              Drag.Points[Drag.PointsCount].MassAirFlow = gSharedParameters.MassAirFlow;
              Drag.Points[Drag.PointsCount].Throttle = gSharedParameters.ThrottlePosition;
              Drag.Points[Drag.PointsCount].Time = DelayDiff(now, Drag.StartTime);
              Drag.PointsCount++;
            }
          }
          else
          {
            Drag.TimeLast = now;
          }
        }
      }
    }
  }
}

static void ecu_specific_parameters_loop(void)
{
  static uint32_t counter = 0;
  uint32_t pointer = gSpecificParametersWritePointer;
  uint32_t pointer_new = pointer;
  uint32_t addr;
  uint8_t period = gSpecificParametersPeriod;

  if(gSpecificParametersConfigured) {
    if(counter >= period) {
      if(++pointer_new >= SPECIFIC_PARAMETERS_ARRAY_POINTS)
        pointer_new = 0;

      if(gSpecificParametersReadPointer == pointer_new) {
        gSpecificParametersOverflow++;
      } else {
        for(int i = 0; i < gSpecificParametersConfigured; i++) {
          addr = gSpecificParametersAddrs[i];
          if(addr && addr < sizeof(gParameters) / 4) {
            memcpy(&gSpecificParametersArray[i][pointer], &((uint32_t *)&gParameters)[addr], sizeof(uDword));
          }
        }

        gSpecificParametersWritePointer = pointer_new;
      }
      counter = 0;
    } else {
      counter++;
    }
  } else {
    gSpecificParametersOverflow = 0;
    gSpecificParametersReadPointer = 0;
    gSpecificParametersWritePointer = 0;
    counter = 0;
  }
}

int8_t positives[ECU_CYLINDERS_COUNT] = {0};
int8_t negatives[ECU_CYLINDERS_COUNT] = {0};

static void ecu_phase_detect_process(void)
{
  int8_t diffs[ECU_CYLINDERS_COUNT] = {0};
  int8_t msi = -1;

  if(gEcuParams.phasedMode == PhasedModeWithoutSensor)
  {
    if(gPhaseDetectCompleted) {
      memset(positives, 0, sizeof(positives));
      memset(negatives, 0, sizeof(negatives));
      for(int cy = 0; cy < ECU_CYLINDERS_COUNT; cy++) {
        if(gPhaseDetectAccelerationsCount[cy] > 2) {
          gPhaseDetectAccelerationsCount[cy] &= ~1;
          for(int i = 3; i < gPhaseDetectAccelerationsCount[cy]; i++) {
            if(fabsf(gPhaseDetectAccelerations[cy][i - 1] - gPhaseDetectAccelerations[cy][i]) > 0.007f) {
              if(gPhaseDetectAccelerations[cy][i - 1] - gPhaseDetectAccelerations[cy][i] > 0)
                if((i & 1))
                  positives[cy]++;
                else negatives[cy]++;
              else {
                if((i & 1))
                  negatives[cy]++;
                else positives[cy]++;
              }
            }
          }
        }
        diffs[cy] = abs(positives[cy] - negatives[cy]);
      }

      if(diffs[0] > diffs[1])
        msi = 0;
      else if(diffs[0] < diffs[1])
        msi = 1;

      if(msi >= 0) {
        if(positives[msi] > negatives[msi]) {
          csps_tsps_simulate(1);
        }
        else if(positives[msi] < negatives[msi]) {
          csps_tsps_simulate(0);
        }
      }

      gPhaseDetectCompleted = 0;
      gPhaseDetectActive = 0;
    }
  }
}

static void ecu_mem_process(void)
{
  if(!Mem.lock)
  {
    Mem.lock = 1;
    if(!Mem.issaving && !Mem.isloading) {
      if(Mem.savereq && !Mem.isloading) {
        Mem.issaving = 1;
      } else if(Mem.loadreq && !Mem.issaving) {
        Mem.isloading = 1;
      } else {
        Mem.lock = 0;
      }
    } else {
      Mem.lock = 0;
    }
  }
}

static void ecu_mem_loop(void)
{
  int8_t flashstatus = 0;

  if(Mem.issaving)
  {
    flashstatus = config_save_all(&gEcuParams, gEcuTable, TABLE_SETUPS_MAX);
    if(flashstatus)
    {
      PK_SaveConfigAcknowledge.ErrorCode = flashstatus > 0 ? 0 : 1;
      PK_SendCommand(Mem.savereqsrc, &PK_SaveConfigAcknowledge, sizeof(PK_SaveConfigAcknowledge));
      gStatus.Flash.Struct.Save = flashstatus > 0 ? HAL_OK : HAL_ERROR;
      Mem.issaving = 0;
      Mem.savereq = 0;
      Mem.lock = 0;
    }
  }
  else if(Mem.isloading)
  {
    flashstatus = config_load_all(&gEcuParams, gEcuTable, TABLE_SETUPS_MAX);
    if(flashstatus)
    {
      PK_RestoreConfigAcknowledge.ErrorCode = flashstatus > 0 ? 0 : 1;
      PK_SendCommand(Mem.loadreqsrc, &PK_RestoreConfigAcknowledge, sizeof(PK_RestoreConfigAcknowledge));
      gStatus.Flash.Struct.Load = flashstatus > 0 ? HAL_OK : HAL_ERROR;
      Mem.isloading = 0;
      Mem.loadreq = 0;
      Mem.lock = 0;
    }
  }
}

static void ecu_bluetooth_loop(void)
{
  static uint8_t bt_was_enabled = 0;
  uint8_t enabled = gEcuParams.isBluetoothEnabled;
  uint16_t pin = gEcuParams.bluetoothPin;
  const char *name = gEcuParams.bluetoothName;

  if(bt_was_enabled != enabled) {
    bt_was_enabled = enabled;
    if(enabled) {
      bluetooth_enable(name, pin);
    } else {
      bluetooth_disable();
    }
  }
}

static void ecu_diagnostic_loop(void)
{
  sOutputDiagnostic output_diagnostic;
  sO2Diagnostic o2_diagnostic;
  sO2Status o2_status = sens_get_o2_status();

  if(outputs_get_diagnostic(&output_diagnostic) == HAL_OK) {
    gStatus.OutputStatus = HAL_OK;
    gStatus.OutputDiagnostic = output_diagnostic;
  } else {
    gStatus.OutputStatus = HAL_ERROR;
  }

  if(sens_get_o2_diagnostic(&o2_status, &o2_diagnostic) == HAL_OK) {
    gStatus.O2Status = HAL_OK;
    gStatus.O2TemperatureStatus = o2_status.TemperatureStatus;
    gStatus.O2HeaterStatus = o2_status.HeaterStatus;
    gStatus.O2Diagnostic = o2_diagnostic;
  } else {
    gStatus.O2Status = HAL_ERROR;
  }

}

static void ecu_corrections_loop(void)
{
  static int8_t old_correction = -1;
  static uint32_t prev_conversion = 0;
  uint32_t now = Delay_Tick;
  uint8_t perform_correction = gEcuParams.performAdaptation;

  if(old_correction < 0)
    old_correction = perform_correction;

  if(perform_correction != old_correction) {
    old_correction = perform_correction;
    if(perform_correction) {
      memset(&gEcuCorrectionsProgress, 0, sizeof(gEcuCorrectionsProgress));
      memset(&gEcuCorrections, 0, sizeof(gEcuCorrections));
    }
    prev_conversion = now;
  }
  if(DelayDiff(now, prev_conversion) > 500000)
  {
    prev_conversion = now;
    for(int y = 0; y < TABLE_FILLING_MAX; y++) {
      for(int x = 0; x < TABLE_ROTATES_MAX; x++) {
        float value = gEcuCorrectionsProgress.progress_ignitions[y][x];
        if(value > 1.0f) value = 1.0f;
        gEcuCorrections.progress_ignitions[y][x] = value * 255.0f;
      }
    }
    for(int y = 0; y < TABLE_DENSITIES_MAX; y++) {
      for(int x = 0; x < TABLE_ROTATES_MAX; x++) {
        float value = gEcuCorrectionsProgress.progress_fill_by_density[y][x];
        if(value > 1.0f) value = 1.0f;
        gEcuCorrections.progress_fill_by_density[y][x] = value * 255.0f;
      }
    }
    for(int y = 0; y < TABLE_DENSITIES_MAX; y++) {
      for(int x = 0; x < TABLE_ROTATES_MAX; x++) {
        float value = gEcuCorrectionsProgress.progress_idle_filling_by_density[y][x];
        if(value > 1.0f) value = 1.0f;
        gEcuCorrections.progress_idle_filling_by_density[y][x] = value * 255.0f;
      }
    }
    for(int y = 0; y < TABLE_THROTTLES_MAX; y++) {
      for(int x = 0; x < TABLE_ROTATES_MAX; x++) {
        float value = gEcuCorrectionsProgress.progress_map_by_thr[y][x];
        if(value > 1.0f) value = 1.0f;
        gEcuCorrections.progress_map_by_thr[y][x] = value * 255.0f;
      }
    }
    for(int y = 0; y < TABLE_TEMPERATURES_MAX; y++) {
      float value = gEcuCorrectionsProgress.progress_idle_valve_position[y];
      if(value > 1.0f) value = 1.0f;
      gEcuCorrections.progress_idle_valve_position[y] = value * 255.0f;
    }
    for(int y = 0; y < ECU_CYLINDERS_COUNT; y++) {
      for(int x = 0; x < TABLE_ROTATES_MAX; x++) {
        float value = gEcuCorrectionsProgress.progress_knock_cy_level_multiplier[y][x];
        if(value > 1.0f) value = 1.0f;
        gEcuCorrections.progress_knock_cy_level_multiplier[y][x] = value * 255.0f;
      }
    }
  }

  speed_setinputcorrective(gEcuParams.speedInputCorrection);
  speed_setoutputcorrective(gEcuParams.speedOutputCorrection);
}

#ifndef SIMULATION
static void ecu_ign_process(void)
{
  static uint32_t tick_ready = 0;
  static uint8_t shutdown_process = 0;
  int8_t status;
  GPIO_PinState state;
  uint32_t now = Delay_Tick;
  uint32_t time;

  state = sens_get_ign(&time);
#ifdef DEBUG
  //state = GPIO_PIN_SET;
#endif

  if(state == GPIO_PIN_RESET) {
    out_set_checkengine(GPIO_PIN_RESET);
  }

  if(state == GPIO_PIN_SET) {
    gIgnShutdownReady = 0;
    gIgnCanShutdown = 0;
    gIgnState = GPIO_PIN_SET;
    out_set_ign(GPIO_PIN_SET);
  }
  else if(!gIgnCanShutdown && time >= 100000) {
    gIgnState = GPIO_PIN_RESET;
    gIgnCanShutdown = 1;

    tick_ready = now;
  }

  if(gIgnCanShutdown && !gIgnShutdownReady) {
    tick_ready = now;
  }
  else if(gIgnCanShutdown && gIgnShutdownReady) {
    if(DelayDiff(now, tick_ready) > 5000000) {
      out_set_ign(GPIO_PIN_RESET);
    }
  }

  if(shutdown_process || (gIgnCanShutdown && !gIgnShutdownReady)) {
    shutdown_process = 1;
    status = ecu_shutdown_process();
    if(status) {
      if(gIgnCanShutdown) {
        gIgnShutdownReady = 1;
      }
      shutdown_process = 0;
    }
  }

  if(shutdown_process || gIgnCanShutdown || gIgnShutdownReady) {
    gEcuIgnStartAllowed = 0;
  } else {
    gEcuIgnStartAllowed = 1;
  }
}

static int8_t ecu_shutdown_process(void)
{
  static int8_t stage = 0;
  static uint32_t last_idle_valve_moving = 0;
  uint32_t now = Delay_Tick;
  int8_t status = 0;
  int8_t retval = 0;
  uint8_t is_idle_valve_moving = out_is_idle_valve_moving();

  switch(stage) {
    case 0:
      last_idle_valve_moving = now;
      stage++;
      break;
    case 1:
      if(is_idle_valve_moving) {
        last_idle_valve_moving = now;
      } else if(DelayDiff(now, last_idle_valve_moving) > 100000) {
        stage++;
      }
      break;
    case 2:
      status = out_reset_idle_valve(DEFAULT_IDLE_VALVE_POSITION);
      if(status) {
    	  gEcuCriticalBackup.idle_valve_position = 1;
    	  stage++;
      }
      break;
    case 3:
        if(!Mem.critical_lock) {
          Mem.critical_lock = 1;
          config_save_critical_backup(&gEcuCriticalBackup);
          Mem.critical_lock = 0;
          stage++;
        }
        break;
    case 4:
      if(!Mem.lock) {
        Mem.lock = 1;
        stage++;
      }
      break;
    case 5:
      Mem.lock = 0;
      stage = 0;
      retval = 1;
      break;
    default:
      stage = 0;
      break;
  }

  return retval;
}
#endif

static void ecu_config_process(void)
{
  uint32_t table_number = gParameters.CurrentTable;
  sEcuTable *table = &gEcuTable[table_number];
  int32_t knock_frequency;
  int32_t knock_gain;
  int32_t integrator_time = gEcuParams.knockIntegratorTime;
  sCspsData data = csps_data();
  float rpm = csps_getrpm(data);
  sMathInterpolateInput ipRpm;

  O2_SetLambdaForceEnabled(gEcuParams.isLambdaForceEnabled);

  if(gEcuParams.useKnockSensor) {
    ipRpm =  math_interpolate_input(rpm, table->rotates, table->rotates_count);
    knock_frequency = roundf(math_interpolate_1d(ipRpm, table->knock_filter_frequency));
    knock_gain = roundf(math_interpolate_1d(ipRpm, table->knock_gain));

    Knock_SetBandpassFilterFrequency(knock_frequency);
    Knock_SetGainValue(knock_gain);
    Knock_SetIntegratorTimeConstant(integrator_time);
  }

  csps_tsps_enable(gEcuParams.phasedMode == PhasedModeWithSensor);
}

static void ecu_immo_init(void)
{
  gParameters.StartAllowed = 0;
}

static void ecu_immo_process(void)
{
  uint8_t start_allowed;

  start_allowed = 1;

  if(gEcuImmoStartAllowed != start_allowed) {
    gEcuImmoStartAllowed = start_allowed;
  }
}

static void ecu_starter_process(void)
{
  uint8_t start_allowed;
  uint8_t running = csps_isrunning();
  GPIO_PinState starter_state;

  starter_state = out_get_starter(NULL);

  start_allowed = gEcuImmoStartAllowed && gEcuIgnStartAllowed;

  if(start_allowed && !running) {
    starter_state = GPIO_PIN_SET;
  } else {
    starter_state = GPIO_PIN_RESET;
  }

  gParameters.StartAllowed = start_allowed;

  out_set_starter(starter_state);
}

static void ecu_can_init(void)
{
  gStatus.CanInitStatus = can_start(0x100, 0x7F0);
  if(gStatus.CanInitStatus == HAL_OK) {
    gCanTestStarted = 1;
  }
}

static void ecu_kline_init(void)
{
  kline_start();
}

static void ecu_can_loop(void)
{
  static sCanMessage message = {0};
  int8_t status;
  if(gStatus.CanInitStatus == HAL_OK) {
    if(gCanTestStarted) {
      status = can_test();
      if(status != 0) {
        gCanTestStarted = 0;
        gStatus.CanTestStatus = status > 0 ? HAL_OK : HAL_ERROR;
      }
    } else {
      status = can_receive(&message);
      if(status > 0) {
        ecu_can_process_message(&message);
      }
      can_loop();
    }
  }
}

static void kline_send_error(sKlineMessage *tx_message, uint8_t req_code, uint8_t err_code)
{
  tx_message->data[0] = 0x7F;
  tx_message->data[1] = req_code;
  tx_message->data[2] = err_code;
  tx_message->length = 3;
}

static void ecu_kline_loop(void)
{
  static uint32_t tester_last = 0;
  static uint8_t session_on = 0;
  const char *string = NULL;
  uint32_t now = Delay_Tick;
  sKlineStatus kline_status = kline_getstatus();
  sKlineMessage rx_message;
  sKlineMessage tx_message;
  int8_t status;

  gStatus.KlineProtocolStatus = kline_status.error_protocol;
  gStatus.KlineLoopbackStatus = kline_status.error_loopback;

  status = kline_receive(&rx_message);
  if(status > 0 && rx_message.length > 0) {
    //By the doc we need to expect 0x10, but ELM327 for some reason sends 0x33
    if(rx_message.dst == 0x10 || rx_message.dst == 0x33) {
      tx_message.addr_mode = rx_message.addr_mode;
      tx_message.dst = rx_message.src;
      tx_message.src = rx_message.dst;
      tx_message.length = 0;

      do {
        if(rx_message.data[0] == 0x81) { //startCommunication
          tx_message.data[tx_message.length++] = rx_message.data[0] | 0x40;
          tx_message.data[tx_message.length++] = 0x6B;
          tx_message.data[tx_message.length++] = 0x8F;
        } else if(rx_message.data[0] == 0x82) { //stopCommunication
          tx_message.data[tx_message.length++] = rx_message.data[0] | 0x40;
        } else if(rx_message.data[0] == 0x10) { //startDiagnosticSession
          if(rx_message.data[1] != 0x81) { kline_send_error(&tx_message, rx_message.data[0], 0x12); break; }
          tx_message.data[tx_message.length++] = rx_message.data[0] | 0x40;
          tx_message.data[tx_message.length++] = rx_message.data[1];
          switch(rx_message.data[2]) {
            case 0x0A:
              kline_setbaud(10400);
              kline_start();
              session_on = 1;
              break;
            case 0x26:
              kline_setbaud(38400);
              kline_start();
              session_on = 2;
              break;
            case 0x39:
              kline_setbaud(57600);
              kline_start();
              session_on = 2;
              break;
            case 0x73:
              kline_setbaud(115200);
              kline_start();
              session_on = 2;
              break;
            default:
              kline_send_error(&tx_message, rx_message.data[0], 0x12);
              break;
          }
        } else if(rx_message.data[0] == 0x20) { //stopDiagnosticSession
          tx_message.data[tx_message.length++] = rx_message.data[0] | 0x40;
          if(session_on > 1) {
            kline_setbaud(10400);
            kline_start();
          }
          session_on = 0;
        } else if(rx_message.data[0] == 0x3E) { //testerPresent
          if(rx_message.data[1] == 1) {
            tx_message.data[tx_message.length++] = rx_message.data[0] | 0x40;
          }
        } else if(rx_message.data[0] == 0x11) { //ecuReset
          if(rx_message.data[1] == 1) {
            tx_message.data[tx_message.length++] = rx_message.data[0] | 0x40;
            //TODO: Delay before reset?
            NVIC_SystemReset();
          } else {
            kline_send_error(&tx_message, rx_message.data[0], 0x12);
          }
        } else if(rx_message.data[0] == 0x1A) { //readEcuIdentification
          tx_message.data[tx_message.length++] = rx_message.data[0] | 0x40;
          tx_message.data[tx_message.length++] = rx_message.data[1];
          if(rx_message.data[1] == 0x80) { //ECUIdentificationDataTable
            string = "AutoECU";
          } else if(rx_message.data[1] == 0x90) { //VIN(Vehicle Identification Number)
            string = "VINNOTSET012345";
          } else if(rx_message.data[1] == 0x91) { //vehicleManufacturerECUHardwareNumber
            string = "AutoECU v0.1";
          } else if(rx_message.data[1] == 0x92) { //systemSupplierECUHardwareNumber
            string = "AutoECU v1.0";
          } else if(rx_message.data[1] == 0x94) { //systemSupplierECUSoftwareNumber
            string = "AutoECU v1.0";
          } else if(rx_message.data[1] == 0x97) { //systemNameOrEngineType
            string = "AutoECU v1.0";
          } else if(rx_message.data[1] == 0x98) { //repairShopCode
            string = "AutoECU v1.0";
          } else if(rx_message.data[1] == 0x99) { //ProgrammingDate
            string = "05.2022";
          } else if(rx_message.data[1] == 0x9A) { //vehicleManufacturerECUIdentifier
            string = "AutoECU v1.0";
          } else {
            kline_send_error(&tx_message, rx_message.data[0], 0x12);
            break;
          }

          if(string) {
            strcpy((char *)&tx_message.data[tx_message.length], string);
            tx_message.length += strlen(string);
          }
        } else if(rx_message.data[0] == 0x14) { //clearDiagnosticInformation
          tx_message.data[tx_message.length++] = rx_message.data[0] | 0x40;
          tx_message.data[tx_message.length++] = rx_message.data[1];
          tx_message.data[tx_message.length++] = rx_message.data[2];

          memset(&gStatus, 0, sizeof(gStatus));
          memset(&gCheckBitmap, 0, sizeof(gCheckBitmap));
          memset(&gEcuCriticalBackup.CheckBitmapRecorded, 0, sizeof(gEcuCriticalBackup.CheckBitmapRecorded));

        } else if(rx_message.data[0] == 0x18) { //readDTCByStatus
          tx_message.data[tx_message.length++] = rx_message.data[0] | 0x40;
          tx_message.data[tx_message.length++] = 0;
          for(int i = 0; i < CHECK_BITMAP_SIZE * 8 && tx_message.length < 200; i++) {
            if((gCheckBitmap[i >> 3] & (1 << (i & 7))) || (gEcuCriticalBackup.CheckBitmapRecorded[i >> 3] & (1 << (i & 7)))) {
              tx_message.data[1]++;
              tx_message.data[tx_message.length++] = (i >> 8) & 0xFF;
              tx_message.data[tx_message.length++] = i & 0xFF;
              tx_message.data[tx_message.length++] = 0xE0;
            }
          }

        } else if(rx_message.data[0] == 0x21) { //readDataByLocalIdentifier
          tx_message.data[tx_message.length++] = rx_message.data[0] | 0x40;
          tx_message.data[tx_message.length++] = rx_message.data[1];
          if(rx_message.data[1] == 0x01) { //afterSalesServiceRecordLocalIdentifier
            tx_message.data[tx_message.length++] = 0x39; //Слово комплектации 1
            tx_message.data[tx_message.length++] = 0x18; //Слово комплектации 2
            tx_message.data[tx_message.length++] = gDiagWorkingMode.Bytes.byte[0]; //Слово режима работы 1
            tx_message.data[tx_message.length++] = gDiagWorkingMode.Bytes.byte[1]; //Слово режима работы 2
            tx_message.data[tx_message.length++] = gDiagErrors.Bytes.byte[0]; //Слово флагов текущих неисправностей 1
            tx_message.data[tx_message.length++] = gDiagErrors.Bytes.byte[1]; //Слово флагов текущих неисправностей 2
            tx_message.data[tx_message.length++] = gDiagErrors.Bytes.byte[2]; //Слово флагов текущих неисправностей 3
            tx_message.data[tx_message.length++] = gDiagErrors.Bytes.byte[3]; //Слово флагов текущих неисправностей 4
            tx_message.data[tx_message.length++] = gSharedParameters.EngineTemp <= -40 ? 0 : gSharedParameters.EngineTemp + 40; //Температура охлаждающей жидкости
            tx_message.data[tx_message.length++] = (gSharedParameters.FuelRatio < 7.5f ? 7.5f : gSharedParameters.FuelRatio > 21 ? 21 : gSharedParameters.FuelRatio) / 14.7f * 256 - 128; //Соотношение воздух/топливо
            tx_message.data[tx_message.length++] = gSharedParameters.ThrottlePosition > 100 ? 100 : gSharedParameters.ThrottlePosition < 0 ? 0 : gSharedParameters.ThrottlePosition; //Положение дроссельной заслонки
            tx_message.data[tx_message.length++] = gSharedParameters.RPM / 40.0f; //Скорость вращения двигателя
            tx_message.data[tx_message.length++] = gSharedParameters.WishIdleRPM / 10.0f; //Скорость вращения двигателя на холостом ходу
            tx_message.data[tx_message.length++] = gSharedParameters.WishIdleValvePosition > 255 ? 255 : gSharedParameters.WishIdleValvePosition < 0 ? 0 : gSharedParameters.WishIdleValvePosition; //Желаемое положение регулятора холостого хода
            tx_message.data[tx_message.length++] = gSharedParameters.IdleValvePosition > 255 ? 255 : gSharedParameters.IdleValvePosition < 0 ? 0 : gSharedParameters.IdleValvePosition; //Текущее положение регулятора холостого хода
            tx_message.data[tx_message.length++] = gSharedParameters.LongTermCorrection + gSharedParameters.ShortTermCorrection * 256.0f + 128; //Коэффициент коррекции времени впрыска
            tx_message.data[tx_message.length++] = (int8_t)(gSharedParameters.IgnitionAdvance * 2.0f); //Угол опережения зажигания
            tx_message.data[tx_message.length++] = gSharedParameters.Speed; //Скорость автомобиля
            tx_message.data[tx_message.length++] = ((gSharedParameters.PowerVoltage > 17.5f ? 17.5f : gSharedParameters.PowerVoltage < 5.5f ? 5.5f : gSharedParameters.PowerVoltage) - 5.2f) * 20.0f; //Напряжение бортсети
            tx_message.data[tx_message.length++] = gSharedParameters.WishIdleRPM / 10.0f; //Желаемые обороты холостого хода
            tx_message.data[tx_message.length++] = (gSharedParameters.AdcLambdaUA > 4.9f ? 4.9f : gSharedParameters.AdcLambdaUA) / 5.0f * 256.0f; //Напряжение на датчике кислорода
            tx_message.data[tx_message.length++] = gDiagWorkingMode.Bits.is_use_lambda * 0xC0; //Флаги состояния датчика кислорода
            tx_message.data[tx_message.length++] = (uint32_t)(gSharedParameters.InjectionPulse * 125.0f) & 0xFF; //Длительность импульса впрыска (младший байт)
            tx_message.data[tx_message.length++] = ((uint32_t)(gSharedParameters.InjectionPulse * 125.0f) >> 8) & 0xFF; //Длительность импульса впрыска (старший байт)
            tx_message.data[tx_message.length++] = (uint32_t)(gSharedParameters.MassAirFlow * 10.0f) & 0xFF; //Массовый расход воздуха (младший байт)
            tx_message.data[tx_message.length++] = ((uint32_t)(gSharedParameters.MassAirFlow * 10.0f) >> 8) & 0xFF; //Массовый расход воздуха (старший байт)
            tx_message.data[tx_message.length++] = (uint32_t)(gSharedParameters.CyclicAirFlow * 6.0f) & 0xFF; //Цикловой расход воздуха (младший байт)
            tx_message.data[tx_message.length++] = ((uint32_t)(gSharedParameters.CyclicAirFlow * 6.0f) >> 8) & 0xFF; //Цикловой расход воздуха (старший байт)
            tx_message.data[tx_message.length++] = (uint32_t)(gSharedParameters.FuelHourly * 50.0f) & 0xFF; //Часовой расход топлива (младший байт)
            tx_message.data[tx_message.length++] = ((uint32_t)(gSharedParameters.FuelHourly * 50.0f) >> 8) & 0xFF; //Часовой расход топлива (старший байт)
            tx_message.data[tx_message.length++] = (uint32_t)(gSharedParameters.FuelConsumption * 128.0f) & 0xFF; //Путевой расход топлива(младший байт)
            tx_message.data[tx_message.length++] = ((uint32_t)(gSharedParameters.FuelConsumption * 128.0f) >> 8) & 0xFF; //Путевой расход топлива(старший байт)
            tx_message.data[tx_message.length++] = 0; //Контрольная сумма ПЗУ (младший байт)
            tx_message.data[tx_message.length++] = 0; //Контрольная сумма ПЗУ (старший байт)
          } else if(rx_message.data[1] == 0x02) { //endOfLineServiceRecordLocalIdentifier
            tx_message.data[tx_message.length++] = 0x39; //Слово комплектации 1
            tx_message.data[tx_message.length++] = 0x18; //Слово комплектации 2
            tx_message.data[tx_message.length++] = gDiagWorkingMode.Bytes.byte[0]; //Слово режима работы 1
            tx_message.data[tx_message.length++] = gDiagWorkingMode.Bytes.byte[1]; //Слово режима работы 2
            tx_message.data[tx_message.length++] = gSharedParameters.EngineTemp <= -40 ? 0 : gSharedParameters.EngineTemp + 40; //Температура охлаждающей жидкости
            tx_message.data[tx_message.length++] = (gSharedParameters.FuelRatio < 7.5f ? 7.5f : gSharedParameters.FuelRatio > 21 ? 21 : gSharedParameters.FuelRatio) / 14.7f * 256 - 128; //Соотношение воздух/топливо
            tx_message.data[tx_message.length++] = gSharedParameters.ThrottlePosition > 100 ? 100 : gSharedParameters.ThrottlePosition < 0 ? 0 : gSharedParameters.ThrottlePosition; //Положение дроссельной заслонки
            tx_message.data[tx_message.length++] = gSharedParameters.RPM / 40.0f; //Скорость вращения двигателя
            tx_message.data[tx_message.length++] = gSharedParameters.WishIdleRPM / 10.0f; //Скорость вращения двигателя на холостом ходу
            tx_message.data[tx_message.length++] = gSharedParameters.IdleValvePosition > 255 ? 255 : gSharedParameters.IdleValvePosition < 0 ? 0 : gSharedParameters.IdleValvePosition; //Текущее положение регулятора холостого хода
            tx_message.data[tx_message.length++] = (int8_t)(gSharedParameters.IgnitionAdvance * 2.0f); //Угол опережения зажигания
            tx_message.data[tx_message.length++] = gSharedParameters.Speed; //Скорость автомобиля
            tx_message.data[tx_message.length++] = ((gSharedParameters.PowerVoltage > 17.5f ? 17.5f : gSharedParameters.PowerVoltage < 5.5f ? 5.5f : gSharedParameters.PowerVoltage) - 5.2f) * 20.0f; //Напряжение бортсети
            tx_message.data[tx_message.length++] = gDiagWorkingMode.Bits.is_use_lambda * 0xC0; //Флаги состояния датчика кислорода
            tx_message.data[tx_message.length++] = (uint32_t)(gSharedParameters.InjectionPulse * 125.0f) & 0xFF; //Длительность импульса впрыска (младший байт)
            tx_message.data[tx_message.length++] = ((uint32_t)(gSharedParameters.InjectionPulse * 125.0f) >> 8) & 0xFF; //Длительность импульса впрыска (старший байт)
            tx_message.data[tx_message.length++] = (uint32_t)(gSharedParameters.MassAirFlow * 10.0f) & 0xFF; //Массовый расход воздуха (младший байт)
            tx_message.data[tx_message.length++] = ((uint32_t)(gSharedParameters.MassAirFlow * 10.0f) >> 8) & 0xFF; //Массовый расход воздуха (старший байт)
            tx_message.data[tx_message.length++] = (uint32_t)(gSharedParameters.CyclicAirFlow * 6.0f) & 0xFF; //Цикловой расход воздуха (младший байт)
            tx_message.data[tx_message.length++] = ((uint32_t)(gSharedParameters.CyclicAirFlow * 6.0f) >> 8) & 0xFF; //Цикловой расход воздуха (старший байт)
            tx_message.data[tx_message.length++] = (uint32_t)(gSharedParameters.FuelHourly * 50.0f) & 0xFF; //Часовой расход топлива (младший байт)
            tx_message.data[tx_message.length++] = ((uint32_t)(gSharedParameters.FuelHourly * 50.0f) >> 8) & 0xFF; //Часовой расход топлива (старший байт)
            tx_message.data[tx_message.length++] = (uint32_t)(gSharedParameters.FuelConsumption * 128.0f) & 0xFF; //Путевой расход топлива(младший байт)
            tx_message.data[tx_message.length++] = ((uint32_t)(gSharedParameters.FuelConsumption * 128.0f) >> 8) & 0xFF; //Путевой расход топлива(старший байт)
          } else if(rx_message.data[1] == 0x03) { //  factoryTestRecordLocalIdentifier
            tx_message.data[tx_message.length++] = (gSharedParameters.KnockSensor > 4.95f) ? 255 : (gSharedParameters.KnockSensor / 5.0f * 256);
            tx_message.data[tx_message.length++] = (gSharedParameters.AdcEngineTemp > 4.95f) ? 255 : (gSharedParameters.AdcEngineTemp / 5.0f * 256);
            tx_message.data[tx_message.length++] = (gSharedParameters.AdcManifoldAirPressure > 4.95f) ? 255 : (gSharedParameters.AdcManifoldAirPressure / 5.0f * 256);
            tx_message.data[tx_message.length++] = (gSharedParameters.PowerVoltage > 4.95f) ? 255 : (gSharedParameters.PowerVoltage / 5.0f * 256);
            tx_message.data[tx_message.length++] = (gSharedParameters.AdcLambdaUA > 4.95f) ? 255 : (gSharedParameters.AdcLambdaUA / 5.0f * 256);
            tx_message.data[tx_message.length++] = (gSharedParameters.AdcThrottlePosition > 4.95f) ? 255 : (gSharedParameters.AdcThrottlePosition / 5.0f * 256);
            tx_message.data[tx_message.length++] = 0;
            tx_message.data[tx_message.length++] = 0;
            tx_message.data[tx_message.length++] = 0;
            tx_message.data[tx_message.length++] = 0;
          } else {
            kline_send_error(&tx_message, rx_message.data[0], 0x12);
            break;
          }


        } else {
          kline_send_error(&tx_message, rx_message.data[0], 0x12);
        }

      } while(0);

      if(tx_message.length > 0) {
        kline_send(&tx_message);
        tester_last = now;
      }
    }
  }

  if(session_on && DelayDiff(now, tester_last) > 3000000) {
    if(session_on > 1) {
      kline_setbaud(10400);
      kline_start();
    }
    session_on = 0;
  }
}

static void ecu_rtc_loop(void)
{
  static uint32_t reset_last = 0;
  static uint32_t get_last = 0;
  uint8_t running = csps_isrunning();
  uint32_t now = Delay_Tick;
  uint32_t running_time = gLocalParams.RunningTime;
  HAL_StatusTypeDef status;
  uint32_t last_runned;

  if(running) {
    if(running_time > 3) {
      if(DelayDiff(now, reset_last) > 500000) {
        reset_last = now;
        ecu_rtc_reset();
      }
    }
  } else {
    reset_last = now;
  }

  if(DelayDiff(now, get_last) > 500000) {
    get_last = now;
    status = ecu_rtc_get_time(&last_runned);
    if(status == HAL_OK) {
      gLocalParams.LastRunned = last_runned;
    }
  }

}

static void ecu_oil_pressure_process(void)
{
  static GPIO_PinState prev = GPIO_PIN_RESET;
  static uint32_t pressure_last = 0;
  uint32_t now = Delay_Tick;
  uint32_t hal_now = HAL_GetTick();
  uint8_t is_running = csps_isrunning();

#ifdef SIMULATION
  GPIO_PinState pressure = is_running ? GPIO_PIN_SET : GPIO_PIN_RESET;
  GPIO_PinState ignition = GPIO_PIN_SET;
#else
  GPIO_PinState pressure = sens_get_oil_pressure(NULL);
  GPIO_PinState ignition = sens_get_ign(NULL);
#endif

  if(is_running && ignition != GPIO_PIN_RESET) {
    if(!gStatus.OilPressure.is_running) {
      if(!gStatus.OilPressure.run_time)
        gStatus.OilPressure.run_time = now;

      if(DelayDiff(now, gStatus.OilPressure.run_time) > 3000000) {
        gStatus.OilPressure.is_running = 1;
      }
    } else {
      if(prev != pressure) {
        if(pressure) {
          if(DelayDiff(now, pressure_last) > 500000) {
            prev = pressure;
            pressure_last = now;
          }
        } else {
          if(DelayDiff(now, pressure_last) > 100000) {
            prev = pressure;
            pressure_last = now;
          }
        }
      } else {
        pressure_last = now;
      }
      gStatus.OilPressure.is_error = prev != GPIO_PIN_SET;
    }
  } else {
    pressure_last = now;
    gStatus.OilPressure.is_running = 0;
    gStatus.OilPressure.run_time = 0;
    gStatus.OilPressure.is_error = 0;
  }

  if(gStatus.OilPressure.is_error) {
    gStatus.OilPressure.error_time = HAL_DelayDiff(hal_now, gStatus.OilPressure.error_started);
  } else {
    gStatus.OilPressure.error_started = hal_now;
    gStatus.OilPressure.error_time = 0;
  }
}

static void ecu_battery_charge_process(void)
{
  static GPIO_PinState prev = GPIO_PIN_RESET;
  static uint32_t charge_last = 0;
  GPIO_PinState charge = sens_get_charge(NULL);
  uint32_t now = Delay_Tick;
  uint8_t is_running = csps_isrunning();

  if(is_running) {
    if(!gStatus.BatteryCharge.is_running) {
      if(!gStatus.BatteryCharge.run_time)
        gStatus.BatteryCharge.run_time = now;

      if(DelayDiff(now, gStatus.BatteryCharge.run_time) > 500000) {
        gStatus.BatteryCharge.is_running = 1;
      }
    } else {
      if(prev != charge) {
        if(charge) {
          if(DelayDiff(now, charge_last) > 300000) {
            prev = charge;
            charge_last = now;
          }
        } else {
          if(DelayDiff(now, charge_last) > 75000) {
            prev = charge;
            charge_last = now;
          }
        }
      } else {
        charge_last = now;
      }
      gStatus.BatteryCharge.is_error = prev != GPIO_PIN_SET;
    }
  } else {
    charge_last = now;
    gStatus.BatteryCharge.is_running = 0;
    gStatus.BatteryCharge.run_time = 0;
    gStatus.BatteryCharge.is_error = 0;
  }
}

static void ecu_csps_acceleration_callback(uint8_t cylinder, float acceleration)
{
  if(gPhaseDetectActive && !gPhaseDetectCompleted) {
    if(gPhaseDetectAccelerationsCount[cylinder] < ACCELERATION_POINTS_COUNT) {
      gPhaseDetectAccelerations[cylinder][gPhaseDetectAccelerationsCount[cylinder]++] = acceleration;
    }
  }
}

void ecu_init(RTC_HandleTypeDef *_hrtc)
{
  hrtc = _hrtc;

  ecu_config_init();

  ecu_set_table(gEcuParams.startupTableNumber);

  ecu_pid_init();

  ecu_init_post_init();

  ecu_can_init();

  ecu_kline_init();

  ecu_immo_init();

  ecu_async_injection_init();

  memset(&gDiagWorkingMode, 0, sizeof(gDiagWorkingMode));
  memset(&gDiagErrors, 0, sizeof(gDiagErrors));
  memset(&gLocalParams, 0, sizeof(gLocalParams));

#if defined(PRESSURE_ACCEPTION_FEATURE) && PRESSURE_ACCEPTION_FEATURE > 0
  gLocalParams.MapAcceptValue = 103000.0f;
  gLocalParams.MapAcceptLast = 0;
#endif

  csps_register_acceleration_callback(ecu_csps_acceleration_callback);

  gEcuInitialized = 1;

}

void ecu_irq_fast_loop(void)
{
  if(!gEcuInitialized)
    return;

  ecu_process();
  ecu_phase_detect_process();

#ifdef DEBUG
  float pressure;
  float throttle;
  sens_get_throttle_position(&throttle);
  sens_get_map(&pressure);
#if defined(PRESSURE_ACCEPTION_FEATURE) && PRESSURE_ACCEPTION_FEATURE > 0
  pressure = gLocalParams.MapAcceptValue;
#endif
    if(pressure > 80000)
      HAL_GPIO_WritePin(MCU_RSVD_2_GPIO_Port, MCU_RSVD_2_Pin, GPIO_PIN_SET);
    else if(pressure < 60000)
      HAL_GPIO_WritePin(MCU_RSVD_2_GPIO_Port, MCU_RSVD_2_Pin, GPIO_PIN_RESET);

    if(throttle > 80) {
      HAL_GPIO_WritePin(MCU_RSVD_3_GPIO_Port, MCU_RSVD_3_Pin, GPIO_PIN_SET);
#ifdef SIMULATION
      gDebugMap = 103000;
#endif
    } else if(throttle < 50) {
      HAL_GPIO_WritePin(MCU_RSVD_3_GPIO_Port, MCU_RSVD_3_Pin, GPIO_PIN_RESET);
#ifdef SIMULATION
      gDebugMap = 50000;
#endif
    }
#endif
}

void ecu_irq_slow_loop(void)
{
  if(!gEcuInitialized)
    return;

  ecu_update_current_table();
  ecu_config_process();
  ecu_update();
  ecu_immo_process();
  ecu_backup_save_process();

  ecu_mem_process();
  ecu_fuelpump_process();
  ecu_fan_process();
  ecu_oil_pressure_process();
  ecu_battery_charge_process();
  ecu_starter_process();

  ecu_drag_process();
  ecu_specific_parameters_loop();

#ifndef SIMULATION
  ecu_ign_process();
#endif
}

void ecu_loop(void)
{
  ecu_mem_loop();
  ecu_bluetooth_loop();
  ecu_diagnostic_loop();
  ecu_checkengine_loop();
  ecu_corrections_loop();
  ecu_can_loop();
  ecu_kline_loop();
  ecu_rtc_loop();
}

void ecu_parse_command(eTransChannels xChaSrc, uint8_t * msgBuf, uint32_t length)
{
  static uint32_t pclastsent = 0;
  uint32_t now = Delay_Tick;
  uint32_t offset;
  uint32_t size;
  uint32_t table;
  uint32_t tablesize;
  uint32_t configsize;
  uint32_t dragpoint;
  uint32_t *addr;
  if(xChaSrc == etrPC)
  {
    if(DelayDiff(now, pclastsent) > 100000)
    {
      pclastsent = now;
      PK_SendCommand(xChaSrc, &PK_PcConnected, sizeof(PK_PcConnected));
    }
  }
  switch(msgBuf[0])
  {
    case PK_PingID :
      PK_Copy(&PK_Ping, msgBuf);
      PK_Pong.RandomPong = PK_Ping.RandomPing;
      PK_SendCommand(xChaSrc, &PK_Pong, sizeof(PK_Pong));
      break;

    case PK_PongID :
      PK_Copy(&PK_Pong, msgBuf);
      (void)PK_Pong.RandomPong;
      break;

    case PK_GeneralStatusRequestID :
      //PK_Copy(&PK_GeneralStatusRequest, msgBuf);
      PK_GeneralStatusResponse.RPM = gSharedParameters.RPM;
      PK_GeneralStatusResponse.Pressure = gSharedParameters.ManifoldAirPressure;
      PK_GeneralStatusResponse.Voltage = gSharedParameters.PowerVoltage;
      PK_GeneralStatusResponse.EngineTemp = gSharedParameters.EngineTemp;
      PK_GeneralStatusResponse.FuelUsage = gSharedParameters.FuelConsumption;
      PK_GeneralStatusResponse.check = gEcuIsError;
      PK_GeneralStatusResponse.tablenum = ecu_get_table();
      strcpy(PK_GeneralStatusResponse.tablename, gEcuTable[ecu_get_table()].name);
      PK_SendCommand(xChaSrc, &PK_GeneralStatusResponse, sizeof(PK_GeneralStatusResponse));
      break;

    case PK_TableMemoryRequestID :
      PK_Copy(&PK_TableMemoryRequest, msgBuf);
      offset = PK_TableMemoryData.offset = PK_TableMemoryRequest.offset;
      size = PK_TableMemoryData.size = PK_TableMemoryRequest.size;
      table = PK_TableMemoryData.table = PK_TableMemoryRequest.table;
      tablesize = PK_TableMemoryData.tablesize = PK_TableMemoryRequest.tablesize;
      PK_TableMemoryData.ErrorCode = 0;

      if(tablesize != sizeof(sEcuTable))
        PK_TableMemoryData.ErrorCode = 1;

      if(size + offset > sizeof(sEcuTable))
        PK_TableMemoryData.ErrorCode = 2;

      if(size > PACKET_TABLE_MAX_SIZE || size > sizeof(sEcuTable))
        PK_TableMemoryData.ErrorCode = 3;

      if(table >= TABLE_SETUPS_MAX)
        PK_TableMemoryData.ErrorCode = 4;

      if(PK_TableMemoryData.ErrorCode == 0)
      {
        memcpy(&PK_TableMemoryData.data[0], &((uint8_t*)&gEcuTable[table])[offset], size);
        PK_TableMemoryData.PacketLength = sizeof(PK_TableMemoryData) - sizeof(PK_TableMemoryData.data) + PK_TableMemoryData.size;
      }
      else
      {
        PK_TableMemoryData.PacketLength = sizeof(PK_TableMemoryData) - sizeof(PK_TableMemoryData.data);
      }
      PK_SendCommand(xChaSrc, &PK_TableMemoryData, PK_TableMemoryData.PacketLength);
      break;

    case PK_TableMemoryDataID :
      PK_Copy(&PK_TableMemoryData, msgBuf);
      offset = PK_TableMemoryAcknowledge.offset = PK_TableMemoryData.offset;
      size = PK_TableMemoryAcknowledge.size = PK_TableMemoryData.size;
      table = PK_TableMemoryAcknowledge.table = PK_TableMemoryData.table;
      tablesize = PK_TableMemoryAcknowledge.tablesize = PK_TableMemoryData.tablesize;
      PK_TableMemoryAcknowledge.ErrorCode = 0;

      if(tablesize != sizeof(sEcuTable))
        PK_TableMemoryAcknowledge.ErrorCode = 1;

      if(size + offset > sizeof(sEcuTable))
        PK_TableMemoryAcknowledge.ErrorCode = 2;

      if(size > PACKET_TABLE_MAX_SIZE || size > sizeof(sEcuTable))
        PK_TableMemoryAcknowledge.ErrorCode = 3;

      if(table >= TABLE_SETUPS_MAX)
        PK_TableMemoryAcknowledge.ErrorCode = 4;

      if(PK_TableMemoryAcknowledge.ErrorCode == 0)
      {
        memcpy(&((uint8_t*)&gEcuTable[table])[offset], &PK_TableMemoryData.data[0], size);
      }

      PK_SendCommand(xChaSrc, &PK_TableMemoryAcknowledge, sizeof(PK_TableMemoryAcknowledge));
      break;

    case PK_ConfigMemoryRequestID :
      PK_Copy(&PK_ConfigMemoryRequest, msgBuf);
      offset = PK_ConfigMemoryData.offset = PK_ConfigMemoryRequest.offset;
      size = PK_ConfigMemoryData.size = PK_ConfigMemoryRequest.size;
      configsize = PK_ConfigMemoryData.configsize = PK_ConfigMemoryRequest.configsize;
      PK_ConfigMemoryData.ErrorCode = 0;

      if(configsize != sizeof(gEcuParams))
        PK_ConfigMemoryData.ErrorCode = 1;

      if(size + offset > sizeof(gEcuParams))
        PK_ConfigMemoryData.ErrorCode = 2;

      if(size > sizeof(gEcuParams) || size > PACKET_CONFIG_MAX_SIZE)
        PK_ConfigMemoryData.ErrorCode = 3;

      if(PK_ConfigMemoryData.ErrorCode == 0)
      {
        memcpy(&PK_ConfigMemoryData.data[0], &((uint8_t*)&gEcuParams)[offset], size);
        PK_ConfigMemoryData.PacketLength = sizeof(PK_ConfigMemoryData) - sizeof(PK_ConfigMemoryData.data) + PK_ConfigMemoryData.size;
      }
      else
      {
        PK_ConfigMemoryData.PacketLength = sizeof(PK_ConfigMemoryData) - sizeof(PK_ConfigMemoryData.data);
      }
      PK_SendCommand(xChaSrc, &PK_ConfigMemoryData, PK_ConfigMemoryData.PacketLength);
      break;

    case PK_ConfigMemoryDataID :
      PK_Copy(&PK_ConfigMemoryData, msgBuf);
      offset = PK_ConfigMemoryAcknowledge.offset = PK_ConfigMemoryData.offset;
      size = PK_ConfigMemoryAcknowledge.size = PK_ConfigMemoryData.size;
      configsize = PK_ConfigMemoryAcknowledge.configsize = PK_ConfigMemoryData.configsize;
      PK_ConfigMemoryAcknowledge.ErrorCode = 0;

      if(configsize != sizeof(gEcuParams))
        PK_ConfigMemoryData.ErrorCode = 1;

      if(size + offset > sizeof(gEcuParams))
        PK_ConfigMemoryData.ErrorCode = 2;

      if(size > sizeof(gEcuParams) || size > PACKET_CONFIG_MAX_SIZE)
        PK_ConfigMemoryData.ErrorCode = 3;

      if(PK_ConfigMemoryAcknowledge.ErrorCode == 0)
      {
        memcpy(&((uint8_t*)&gEcuParams)[offset], &PK_ConfigMemoryData.data[0], size);
      }

      PK_SendCommand(xChaSrc, &PK_ConfigMemoryAcknowledge, sizeof(PK_ConfigMemoryAcknowledge));
      break;

    case PK_CriticalMemoryRequestID :
      PK_Copy(&PK_CriticalMemoryRequest, msgBuf);
      offset = PK_CriticalMemoryData.offset = PK_CriticalMemoryRequest.offset;
      size = PK_CriticalMemoryData.size = PK_CriticalMemoryRequest.size;
      configsize = PK_CriticalMemoryData.configsize = PK_CriticalMemoryRequest.configsize;
      PK_CriticalMemoryData.ErrorCode = 0;

      if(configsize != sizeof(gEcuCriticalBackup))
        PK_CriticalMemoryData.ErrorCode = 1;

      if(size + offset > sizeof(gEcuCriticalBackup))
        PK_CriticalMemoryData.ErrorCode = 2;

      if(size > sizeof(gEcuCriticalBackup) || size > PACKET_CONFIG_MAX_SIZE)
        PK_CriticalMemoryData.ErrorCode = 3;

      if(PK_CriticalMemoryData.ErrorCode == 0)
      {
        memcpy(&PK_CriticalMemoryData.data[0], &((uint8_t*)&gEcuCriticalBackup)[offset], size);
        PK_CriticalMemoryData.PacketLength = sizeof(PK_CriticalMemoryData) - sizeof(PK_CriticalMemoryData.data) + PK_CriticalMemoryData.size;
      }
      else
      {
        PK_CriticalMemoryData.PacketLength = sizeof(PK_CriticalMemoryData) - sizeof(PK_CriticalMemoryData.data);
      }
      PK_SendCommand(xChaSrc, &PK_CriticalMemoryData, PK_CriticalMemoryData.PacketLength);
      break;

    case PK_CriticalMemoryDataID :
      PK_Copy(&PK_CriticalMemoryData, msgBuf);
      offset = PK_CriticalMemoryAcknowledge.offset = PK_CriticalMemoryData.offset;
      size = PK_CriticalMemoryAcknowledge.size = PK_CriticalMemoryData.size;
      configsize = PK_CriticalMemoryAcknowledge.configsize = PK_CriticalMemoryData.configsize;
      PK_CriticalMemoryAcknowledge.ErrorCode = 0;

      if(configsize != sizeof(gEcuCriticalBackup))
        PK_CriticalMemoryData.ErrorCode = 1;

      if(size + offset > sizeof(gEcuCriticalBackup))
        PK_CriticalMemoryData.ErrorCode = 2;

      if(size > sizeof(gEcuCriticalBackup) || size > PACKET_CONFIG_MAX_SIZE)
        PK_CriticalMemoryData.ErrorCode = 3;

      if(PK_CriticalMemoryAcknowledge.ErrorCode == 0)
      {
        memcpy(&((uint8_t*)&gEcuCriticalBackup)[offset], &PK_CriticalMemoryData.data[0], size);
      }

      PK_SendCommand(xChaSrc, &PK_CriticalMemoryAcknowledge, sizeof(PK_CriticalMemoryAcknowledge));
      break;

    case PK_CorrectionsMemoryRequestID :
      PK_Copy(&PK_CorrectionsMemoryRequest, msgBuf);
      offset = PK_CorrectionsMemoryData.offset = PK_CorrectionsMemoryRequest.offset;
      size = PK_CorrectionsMemoryData.size = PK_CorrectionsMemoryRequest.size;
      configsize = PK_CorrectionsMemoryData.configsize = PK_CorrectionsMemoryRequest.configsize;
      PK_CorrectionsMemoryData.ErrorCode = 0;

      if(configsize != sizeof(gEcuCorrections))
        PK_CorrectionsMemoryData.ErrorCode = 1;

      if(size + offset > sizeof(gEcuCorrections))
        PK_CorrectionsMemoryData.ErrorCode = 2;

      if(size > sizeof(gEcuCorrections) || size > PACKET_CONFIG_MAX_SIZE)
        PK_CorrectionsMemoryData.ErrorCode = 3;

      if(PK_CorrectionsMemoryData.ErrorCode == 0)
      {
        memcpy(&PK_CorrectionsMemoryData.data[0], &((uint8_t*)&gEcuCorrections)[offset], size);
        PK_CorrectionsMemoryData.PacketLength = sizeof(PK_CorrectionsMemoryData) - sizeof(PK_CorrectionsMemoryData.data) + PK_CorrectionsMemoryData.size;
      }
      else
      {
        PK_CorrectionsMemoryData.PacketLength = sizeof(PK_CorrectionsMemoryData) - sizeof(PK_CorrectionsMemoryData.data);
      }
      PK_SendCommand(xChaSrc, &PK_CorrectionsMemoryData, PK_CorrectionsMemoryData.PacketLength);
      break;

    case PK_CorrectionsMemoryDataID :
      PK_Copy(&PK_CorrectionsMemoryData, msgBuf);
      offset = PK_CorrectionsMemoryAcknowledge.offset = PK_CorrectionsMemoryData.offset;
      size = PK_CorrectionsMemoryAcknowledge.size = PK_CorrectionsMemoryData.size;
      configsize = PK_CorrectionsMemoryAcknowledge.configsize = PK_CorrectionsMemoryData.configsize;
      PK_CorrectionsMemoryAcknowledge.ErrorCode = 0;

      if(configsize != sizeof(gEcuCorrections))
        PK_CorrectionsMemoryData.ErrorCode = 1;

      if(size + offset > sizeof(gEcuCorrections))
        PK_CorrectionsMemoryData.ErrorCode = 2;

      if(size > sizeof(gEcuCorrections) || size > PACKET_CONFIG_MAX_SIZE)
        PK_CorrectionsMemoryData.ErrorCode = 3;

      if(PK_CorrectionsMemoryAcknowledge.ErrorCode == 0)
      {
        memcpy(&((uint8_t*)&gEcuCorrections)[offset], &PK_CorrectionsMemoryData.data[0], size);
      }

      PK_SendCommand(xChaSrc, &PK_CorrectionsMemoryAcknowledge, sizeof(PK_CorrectionsMemoryAcknowledge));
      break;

    case PK_SaveConfigID :
      if(!Mem.savereq)
      {
        Mem.savereqsrc = xChaSrc;
        Mem.savereq = 1;
        //CanDeinit = 0;
      }
      else if(Mem.loadreq)
      {
        PK_SaveConfigAcknowledge.ErrorCode = 6;
        PK_SendCommand(xChaSrc, &PK_SaveConfigAcknowledge, sizeof(PK_SaveConfigAcknowledge));
      }
      else
      {
        PK_SaveConfigAcknowledge.ErrorCode = 7;
        PK_SendCommand(xChaSrc, &PK_SaveConfigAcknowledge, sizeof(PK_SaveConfigAcknowledge));
      }
      break;


    case PK_RestoreConfigID :
      if(!Mem.loadreq)
      {
        Mem.loadreqsrc = xChaSrc;
        Mem.loadreq = 1;
      }
      else if(Mem.savereq)
      {
        PK_RestoreConfigAcknowledge.ErrorCode = 6;
        PK_SendCommand(xChaSrc, &PK_RestoreConfigAcknowledge, sizeof(PK_RestoreConfigAcknowledge));
      }
      else
      {
        PK_RestoreConfigAcknowledge.ErrorCode = 7;
        PK_SendCommand(xChaSrc, &PK_RestoreConfigAcknowledge, sizeof(PK_RestoreConfigAcknowledge));
      }
      break;

    case PK_DragStartID :
      PK_Copy(&PK_DragStart, msgBuf);
      Drag.FromSpeed = PK_DragStartAcknowledge.FromSpeed = PK_DragStart.FromSpeed;
      Drag.ToSpeed = PK_DragStartAcknowledge.ToSpeed = PK_DragStart.ToSpeed;
      Drag.Started = 0;
      Drag.Ready = 0;
      Drag.Completed = 0;
      Drag.PointsCount = 0;
      Drag.StartTime = 0;
      Drag.TimeLast = 0;
      if(fabsf(Drag.FromSpeed - Drag.ToSpeed) > 5.0f)
      {
        Drag.Status = PK_DragStartAcknowledge.ErrorCode = 0;
        Drag.Ready = 1;
      }
      else Drag.Status = PK_DragStartAcknowledge.ErrorCode = 1;
      PK_SendCommand(xChaSrc, &PK_DragStartAcknowledge, sizeof(PK_DragStartAcknowledge));
      break;

    case PK_DragStopID :
      PK_Copy(&PK_DragStop, msgBuf);
      PK_DragStopAcknowledge.FromSpeed = PK_DragStop.FromSpeed;
      PK_DragStopAcknowledge.ToSpeed = PK_DragStop.ToSpeed;
      Drag.Status = PK_DragStopAcknowledge.ErrorCode = 0;
      Drag.Started = 0;
      Drag.Ready = 0;
      Drag.Completed = 0;
      Drag.PointsCount = 0;
      Drag.StartTime = 0;
      if(Drag.FromSpeed != PK_DragStop.FromSpeed || Drag.ToSpeed != PK_DragStop.ToSpeed)
      {
        PK_DragStopAcknowledge.ErrorCode = 2;
      }
      PK_SendCommand(xChaSrc, &PK_DragStopAcknowledge, sizeof(PK_DragStopAcknowledge));
      break;

    case PK_DragUpdateRequestID :
      PK_Copy(&PK_DragUpdateRequest, msgBuf);
      PK_DragUpdateResponse.ErrorCode = 0;
      PK_DragUpdateResponse.FromSpeed = Drag.FromSpeed;
      PK_DragUpdateResponse.ToSpeed = Drag.ToSpeed;
      PK_DragUpdateResponse.Data.Pressure = gSharedParameters.ManifoldAirPressure;
      PK_DragUpdateResponse.Data.RPM = gSharedParameters.RPM;
      PK_DragUpdateResponse.Data.Mixture = gSharedParameters.FuelRatio;
      PK_DragUpdateResponse.Data.Acceleration = gSharedParameters.Acceleration;
      PK_DragUpdateResponse.Data.Ignition = gSharedParameters.IgnitionAdvance;
      PK_DragUpdateResponse.Data.CycleAirFlow = gSharedParameters.CyclicAirFlow;
      PK_DragUpdateResponse.Data.MassAirFlow = gSharedParameters.MassAirFlow;
      PK_DragUpdateResponse.Data.Throttle = gSharedParameters.ThrottlePosition;
      PK_DragUpdateResponse.TotalPoints = Drag.PointsCount;
      PK_DragUpdateResponse.Started = Drag.Started;
      PK_DragUpdateResponse.Completed = Drag.Completed;
      PK_DragUpdateResponse.Data.Time = Drag.StartTime > 0 ? DelayDiff(Drag.StopTime, Drag.StartTime) : 0;
      if(Drag.Status > 0) PK_DragUpdateResponse.ErrorCode = Drag.Status + 10;
      PK_SendCommand(xChaSrc, &PK_DragUpdateResponse, sizeof(PK_DragUpdateResponse));
      break;

    case PK_DragPointRequestID :
      PK_Copy(&PK_DragPointRequest, msgBuf);
      PK_DragPointResponse.FromSpeed = PK_DragStop.FromSpeed;
      PK_DragPointResponse.ToSpeed = PK_DragStop.ToSpeed;
      PK_DragPointResponse.ErrorCode = 0;
      dragpoint = PK_DragPointResponse.PointNumber = PK_DragPointRequest.PointNumber;
      if(dragpoint >= Drag.PointsCount)
      {
        PK_DragPointResponse.Point.Pressure = 0;
        PK_DragPointResponse.Point.RPM = 0;
        PK_DragPointResponse.Point.Mixture = 0;
        PK_DragPointResponse.Point.Acceleration = 0;
        PK_DragPointResponse.Point.Ignition = 0;
        PK_DragPointResponse.Point.Time = 0;
        PK_DragPointResponse.ErrorCode = 3;
      }
      else
      {
        PK_DragPointResponse.Point = Drag.Points[dragpoint];
        if(Drag.FromSpeed != PK_DragPointRequest.FromSpeed || Drag.ToSpeed != PK_DragPointRequest.ToSpeed)
        {
          PK_DragPointResponse.ErrorCode = 2;
        }
      }

      PK_SendCommand(xChaSrc, &PK_DragPointResponse, sizeof(PK_DragPointResponse));
      break;

    case PK_ConfigMemoryAcknowledgeID :
      PK_Copy(&PK_ConfigMemoryAcknowledge, msgBuf);
      if(PK_ConfigMemoryAcknowledge.ErrorCode != 0)
      {

      }
      break;

    case PK_TableMemoryAcknowledgeID :
      PK_Copy(&PK_TableMemoryAcknowledge, msgBuf);
      if(PK_TableMemoryAcknowledge.ErrorCode != 0)
      {

      }
      break;

    case PK_CriticalMemoryAcknowledgeID :
      PK_Copy(&PK_CriticalMemoryAcknowledge, msgBuf);
      if(PK_CriticalMemoryAcknowledge.ErrorCode != 0)
      {

      }
      break;

    case PK_CorrectionsMemoryAcknowledgeID :
      PK_Copy(&PK_CorrectionsMemoryAcknowledge, msgBuf);
      if(PK_CorrectionsMemoryAcknowledge.ErrorCode != 0)
      {

      }
      break;

    case PK_FuelSwitchID :
      PK_Copy(&PK_FuelSwitch, msgBuf);
      if(PK_FuelSwitch.FuelSwitchPos < 3)
        ecu_set_table(PK_FuelSwitch.FuelSwitchPos);
      break;

    case PK_ParametersRequestID :
      //PK_Copy(&PK_ParametersRequest, msgBuf);
      PK_ParametersResponse.Parameters = gSharedParameters;
      gLocalParams.RequestFillLast = 0;
      PK_SendCommand(xChaSrc, &PK_ParametersResponse, sizeof(PK_ParametersResponse));
      break;

    case PK_SpecificParameterRequestID :
      PK_Copy(&PK_SpecificParameterRequest, msgBuf);
      PK_SpecificParameterResponse.addr = PK_SpecificParameterRequest.addr;
      memset(&PK_SpecificParameterResponse.Parameter, 0, sizeof(PK_SpecificParameterResponse.Parameter));
      if(PK_SpecificParameterRequest.addr < sizeof(gSharedParameters) / 4) {
        addr = &((uint32_t *)&gSharedParameters)[PK_SpecificParameterRequest.addr];
        memcpy(&PK_SpecificParameterResponse.Parameter, addr, sizeof(uint32_t));
      }
      PK_SendCommand(xChaSrc, &PK_SpecificParameterResponse, sizeof(PK_SpecificParameterResponse));
      break;

    case PK_SpecificParameterArrayConfigureRequestID :
      PK_Copy(&PK_SpecificParameterArrayConfigureRequest, msgBuf);
      PK_SpecificParameterArrayConfigureResponse.ErrorCode = 0;

      for(int i = 0; i < SPECIFIC_PARAMETERS_ARRAY_MAX_ITEMS; i++) {
        PK_SpecificParameterArrayConfigureResponse.Addrs[i] = PK_SpecificParameterArrayConfigureRequest.Addrs[i];
        if(PK_SpecificParameterArrayConfigureRequest.Addrs[i] >= sizeof(gParameters) / 4)
          PK_SpecificParameterArrayConfigureResponse.ErrorCode = 11;
      }
      PK_SpecificParameterArrayConfigureResponse.Period = PK_SpecificParameterArrayConfigureRequest.Period;
      PK_SpecificParameterArrayConfigureResponse.BufferSize = 0;

      if(PK_SpecificParameterArrayConfigureRequest.Period > 1000) {
        PK_SpecificParameterArrayConfigureResponse.ErrorCode = 12;
      }

      if(PK_SpecificParameterArrayConfigureResponse.ErrorCode == 0) {
        gSpecificParametersOverflow = 0;
        gSpecificParametersReadPointer = 0;
        gSpecificParametersWritePointer = 0;

        gSpecificParametersPeriod = PK_SpecificParameterArrayConfigureRequest.Period;
        gSpecificParametersConfigured = 0;
        for(int i = 0; i < SPECIFIC_PARAMETERS_ARRAY_MAX_ITEMS; i++) {
          if(PK_SpecificParameterArrayConfigureRequest.Addrs[i]) {
            gSpecificParametersAddrs[gSpecificParametersConfigured++] = PK_SpecificParameterArrayConfigureRequest.Addrs[i];
          }
        }
        PK_SpecificParameterArrayConfigureResponse.BufferSize = SPECIFIC_PARAMETERS_ARRAY_POINTS;
      }

      PK_SendCommand(xChaSrc, &PK_SpecificParameterArrayConfigureResponse, sizeof(PK_SpecificParameterArrayConfigureResponse));
      break;

    case PK_SpecificParameterArrayRequestID :
      PK_Copy(&PK_SpecificParameterArrayRequest, msgBuf);
      memset(&PK_SpecificParameterArrayResponse.Parameters, 0, sizeof(PK_SpecificParameterArrayResponse.Parameters));

      size = 0;
      while(size < PACKET_SPECIFIC_PARAMETERS_ARRAY_MAX_SIZE && gSpecificParametersWritePointer != gSpecificParametersReadPointer) {
        offset = gSpecificParametersReadPointer;

        for(int i = 0; i < gSpecificParametersConfigured && size < PACKET_SPECIFIC_PARAMETERS_ARRAY_MAX_SIZE; i++, size++) {
          PK_SpecificParameterArrayResponse.Parameters[size] = gSpecificParametersArray[i][offset];
        }

        if(++offset >= SPECIFIC_PARAMETERS_ARRAY_POINTS)
          offset = 0;
        gSpecificParametersReadPointer = offset;
      }
      PK_SpecificParameterArrayResponse.Length = size;
      PK_SpecificParameterArrayResponse.Underflow = gSpecificParametersOverflow;
      PK_SpecificParameterArrayResponse.Items = gSpecificParametersConfigured;
      PK_SpecificParameterArrayResponse.IsLeft = gSpecificParametersReadPointer != gSpecificParametersWritePointer;
      PK_SpecificParameterArrayResponse.ErrorCode = 0;
      gSpecificParametersOverflow = 0;

      PK_SendCommand(xChaSrc, &PK_SpecificParameterArrayResponse, sizeof(PK_SpecificParameterArrayResponse));
      break;

    case PK_ForceParametersDataID :
      PK_Copy(&PK_ForceParametersData, msgBuf);
      gForceParameters = PK_ForceParametersData.Parameters;
      PK_ForceParametersDataAcknowledge.ErrorCode = 0;
      PK_SendCommand(xChaSrc, &PK_ForceParametersDataAcknowledge, sizeof(PK_ForceParametersDataAcknowledge));
      break;

    case PK_StatusRequestID :
      //PK_Copy(&PK_StatusRequest, msgBuf);
      memcpy(PK_StatusResponse.CheckBitmap, gCheckBitmap, CHECK_BITMAP_SIZE);
      memcpy(PK_StatusResponse.CheckBitmapRecorded, gEcuCriticalBackup.CheckBitmapRecorded, CHECK_BITMAP_SIZE);
      PK_SendCommand(xChaSrc, &PK_StatusResponse, sizeof(PK_StatusResponse));
      break;

    case PK_ResetStatusRequestID :
      //PK_Copy(&PK_StatusRequest, msgBuf);
      gStatusReset = 1;
      memset(&gStatus, 0, sizeof(gStatus));
      memset(&gCheckBitmap, 0, sizeof(gCheckBitmap));
      memset(&gEcuCriticalBackup.CheckBitmapRecorded, 0, sizeof(gEcuCriticalBackup.CheckBitmapRecorded));
      PK_ResetStatusResponse.ErrorCode = 0;
      PK_SendCommand(xChaSrc, &PK_ResetStatusResponse, sizeof(PK_ResetStatusResponse));
      break;

    case PK_IgnitionInjectionTestRequestID :
      PK_Copy(&PK_IgnitionInjectionTestRequest, msgBuf);

      gIITest.IgnitionEnabled = 0;
      gIITest.InjectionEnabled = 0;
      gIITest.StartedTime = 0;

      gIITest.IgnitionEnabled = PK_IgnitionInjectionTestRequest.IgnitionEnabled;
      gIITest.InjectionEnabled = PK_IgnitionInjectionTestRequest.InjectionEnabled;
      gIITest.Count = PK_IgnitionInjectionTestRequest.Count;
      gIITest.Period = PK_IgnitionInjectionTestRequest.Period;
      gIITest.IgnitionPulse = PK_IgnitionInjectionTestRequest.IgnitionPulse;
      gIITest.InjectionPulse = PK_IgnitionInjectionTestRequest.InjectionPulse;

      if(gIITest.IgnitionEnabled || gIITest.InjectionEnabled) {
        gIITest.StartedTime = Delay_Tick;
        if(gIITest.StartedTime == 0)
          gIITest.StartedTime++;
      }
      gIITest.CompletedCount = 0;

      PK_IgnitionInjectionTestResponse.ErrorCode = 0;
      PK_SendCommand(xChaSrc, &PK_IgnitionInjectionTestResponse, sizeof(PK_IgnitionInjectionTestResponse));
      break;

    default:
      break;
  }
}

static int8_t ecu_can_process_message(const sCanMessage *message)
{
  sCanMessage transmit = {0};
  int8_t status = 0;

  if(message->id == 0x100) { //Loopback
    if(message->rtr == CAN_RTR_DATA) {
      transmit.id = message->id + 0x100;
      transmit.rtr = CAN_RTR_DATA;
      transmit.length = message->length;
      memcpy(transmit.data.bytes, message->data.bytes, message->length);
    }
  } else if(message->id == 0x102) { //Parameter request
    if(message->rtr == CAN_RTR_DATA && message->length == 4) {
      if(message->data.dwords[0] < sizeof(gSharedParameters) / sizeof(uint32_t)) {
        transmit.id = message->id + 0x100;
        transmit.rtr = CAN_RTR_DATA;
        transmit.length = 8;
        transmit.data.dwords[0] = message->data.dwords[0];
        transmit.data.dwords[1] = ((uint32_t *)&gSharedParameters)[message->data.dwords[0]];

        if(OFFSETOF(sParameters, KnockSensor) / 4 == message->data.dwords[0] ||
            OFFSETOF(sParameters, KnockSensorFiltered) / 4 == message->data.dwords[0]) {
          gLocalParams.RequestFillLast = 0;
        }
      }
    }
  } else if(message->id == 0x103) { //Table memory request
    if(message->rtr == CAN_RTR_DATA && message->length == 4) {
      if(message->data.dwords[0] < sizeof(gEcuTable) / sizeof(uint32_t)) {
        transmit.id = message->id + 0x100;
        transmit.rtr = CAN_RTR_DATA;
        transmit.length = 8;
        transmit.data.dwords[0] = message->data.dwords[0];
        transmit.data.dwords[1] = ((uint32_t *)&gEcuTable[0])[message->data.dwords[0]];
      }
    }
  } else if(message->id == 0x104) { //Config memory request
    if(message->rtr == CAN_RTR_DATA && message->length == 4) {
      if(message->data.dwords[0] < sizeof(gEcuParams) / sizeof(uint32_t)) {
        transmit.id = message->id + 0x100;
        transmit.rtr = CAN_RTR_DATA;
        transmit.length = 8;
        transmit.data.dwords[0] = message->data.dwords[0];
        transmit.data.dwords[1] = ((uint32_t *)&gEcuParams)[message->data.dwords[0]];
      }
    }
  }

  if(transmit.id > 0 || transmit.length > 0) {
    status = can_send(&transmit);
  }

  return status;
}

void ecu_hardfault_handle(void)
{
  gEcuCriticalBackup.CheckBitmapRecorded[CheckHardFaultException >> 3] |= 1 << (CheckHardFaultException & 7);
  config_save_critical_backup(&gEcuCriticalBackup);
}

