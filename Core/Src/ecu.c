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
#include "kalman.h"
#include "can_signals.h"
#include "can_signals_db.h"

#define DEFAULT_IDLE_VALVE_POSITION 100
#define IGNITION_ACCEPTION_FEATURE  1
#define KNOCK_DETONATION_INCREASING_ADVANCE 0
#define KNOCK_LOW_NOISE_ON_ENGINE_TEMP_THRESHOLD 70.0f
#define GBC_CALIBRATION_ON_ENGINE_TEMP_THRESHOLD 70.0f
#define FUEL_PUMP_ON_INJ_CH1_ONLY   1
#define INJECTORS_ON_INJ_CH1_ONLY   1
#define ACCELERATION_POINTS_COUNT   12
#define LEARN_ENRICHMENT_POST_CYCLES_DELAY  (ECU_CYLINDERS_COUNT * 16)
#define LEARN_ENLEANMENT_POST_CYCLES_DELAY  (ECU_CYLINDERS_COUNT * 32)
#define IDLE_ACCELERATE_POST_CYCLES_DELAY   (ECU_CYLINDERS_COUNT * 32)
#define LEARN_ACCEPT_CYCLES_BUFFER_SIZE     0

typedef float (*math_interpolate_2d_set_func_t)(sMathInterpolateInput input_x, sMathInterpolateInput input_y,
    uint32_t y_size, float (*table)[], float new_value, float limit_l, float limit_h);
typedef float (*math_interpolate_2d_func_t)(sMathInterpolateInput input_x, sMathInterpolateInput input_y,
    uint32_t y_size, const float (*table)[]);

#define ENRICHMENT_LOAD_STATES_COUNT     (64)
#define ASYNC_INJECTION_FIFO_SIZE   (32)
#define FAN_HIGH_SWITCH_TIME        (500 * 1000)
#define FUEL_PUMP_TIMEOUT           (2 * 1000 * 1000)
#define FAN_TIMEOUT                 (3 * 1000 * 1000)
#define CALIBRATION_MIN_RUNTIME     (3 * 1000 * 1000)
#define PID_MINIMUM_RUNTIME         (3 * 1000 * 1000)
#define FUEL_FILM_MINIMUM_RUNTIME   (1 * 1000 * 1000)
#define INJ_PHASE_CHANGE_PER_CYCLE  (6.0f)

typedef enum {
  KnockStatusOk = 0,
  KnockStatusLowNoise = 1,
  KnockStatusDedonation = 2,
  KnockStatusStrongDedonation = 4,
  KnockStatusContinuousDedonation = 8,
}eKnockStatus;

typedef struct {
    float filling_gbc_map[TABLE_PRESSURES_MAX][TABLE_ROTATES_MAX];
    float filling_gbc_tps[TABLE_THROTTLES_MAX][TABLE_ROTATES_MAX];
}sEcuCorrectionsTemp;

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
            HAL_StatusTypeDef Tps : 2;
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
        eKnockStatus CylinderStatus[ECU_CYLINDERS_COUNT];
        uint32_t DetonationLastCy[ECU_CYLINDERS_COUNT];
        uint32_t StrongDetonationLastCy[ECU_CYLINDERS_COUNT];
        uint32_t LowNoiseLastCy[ECU_CYLINDERS_COUNT];
        uint32_t DetonationCount;
        uint32_t DetonationCountCy[ECU_CYLINDERS_COUNT];
        float Voltages[ECU_CYLINDERS_COUNT];
        float VoltagesCorrected[ECU_CYLINDERS_COUNT];
        float VoltagesLpf[ECU_CYLINDERS_COUNT];
        float Denoised[ECU_CYLINDERS_COUNT];
        float Detonates[ECU_CYLINDERS_COUNT];
        float Advances[ECU_CYLINDERS_COUNT];
        float AdvancesFinal[ECU_CYLINDERS_COUNT];
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
        float DetonationSaturation;
        float AdaptationDetonates[ECU_CYLINDERS_COUNT];
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
        uint8_t critical_error;
        uint8_t sync_error;
        uint32_t sync_error_time;
        uint32_t sync_error_last;
    }Tsps;
    struct  {
        HAL_StatusTypeDef Comm;

        HAL_StatusTypeDef Pedal1;
        HAL_StatusTypeDef Pedal2;
        HAL_StatusTypeDef PedalMismatch;

        HAL_StatusTypeDef Tps1;
        HAL_StatusTypeDef Tps2;
        HAL_StatusTypeDef TpsMismatch;
        HAL_StatusTypeDef AdcErrorFlag;
        HAL_StatusTypeDef EtcCommErrorFlag;
        HAL_StatusTypeDef MotorAvailability;
        struct {
          HAL_StatusTypeDef OpenLoad;
          HAL_StatusTypeDef ShortToGND;
          HAL_StatusTypeDef ShortToSupply;
          HAL_StatusTypeDef Temperature;
          HAL_StatusTypeDef SupplyFailure;
          HAL_StatusTypeDef ErrorFlag;
          HAL_StatusTypeDef AlwaysHigh;
        }Motor;
    }Etc;
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

typedef volatile struct {
    float StartPosition;
    uint32_t StartDelay;
    float FinalPosition;
    uint32_t MoveTime;
    uint32_t FinalDelay;
    float CurrentPosition;
    uint32_t StartedTime;
    uint8_t Stage;
} sEtcTest;

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
    float IgnitionCorrectionCy[ECU_CYLINDERS_COUNT];
    float InjectionCorrectionCy[ECU_CYLINDERS_COUNT];
    volatile uint32_t RequestFillLast;
    volatile uint32_t ParametersAcceptLast;
    uint32_t RunningTime;
    uint32_t LastRunned;

    uint8_t PhasedInjection;
    uint8_t EnrichmentTriggered;
    uint8_t FuelFilmAllowed;
    uint8_t StartupInjectionStarted;

    float EtcThrottlePosition;
    float EtcPedalPosition;
    float EtcThrottleTargetPosition;
    float EtcThrottleDefaultPosition;
    float RequestedInjectionPhase;
    float AcceptedInjectionPhase;

    float KnockDetectPhaseStart;
    float KnockDetectPhaseEnd;

    float CycleFuelFlow;
    float CycleAirFlow;
    float FuelFlowPerUs;
    float InjectorLag;
    float DynamicFilmCorrection;
    float DynamicFilmCorrLpf;

    float FuelFilmTemp[ECU_CYLINDERS_COUNT];
    float FuelFilmNew[ECU_CYLINDERS_COUNT];
    float FuelFilmOld[ECU_CYLINDERS_COUNT];

    float FinalInjectionPulse;
    float FinalInjectionDutyCycle;
    float FinalInjectionPhaseDuration;
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
static sEcuCorrectionsTemp gEcuTempCorrections;
static sEcuCorrectionsProgress gEcuCorrectionsProgress;
static sEcuCriticalBackup gEcuCriticalBackup;
static uint8_t volatile gStatusReset = 0;
static sStatus gStatus = {{{0}}};
static sParameters gParameters = {0};
static sParameters gSharedParameters = {0};
static sForceParameters gForceParameters = {0};
static sIgnitionInjectionTest gIITest = {0};
static sEtcTest gEtcTest = {0};
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
static volatile int8_t gBackupSaveReq = 0;
static volatile int8_t gBackupSaveRes = 0;

static sMathPid gPidIdleValveAirFlow = {0};
static sMathPid gPidIdleValveRpm = {0};
static sMathPid gPidIdleThrottleAirFlow = {0};
static sMathPid gPidIdleThrottleRpm = {0};
static sMathPid gPidIdleIgnition = {0};
static sMathPid gPidShortTermCorr = {0};

volatile static uint8_t gEtcErrorReset = 0;
volatile static uint32_t gEtcCommLast = 0;
volatile static uint32_t gLoggerCommLast = 0;

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

#if defined(LEARN_ACCEPT_CYCLES_BUFFER_SIZE) && LEARN_ACCEPT_CYCLES_BUFFER_SIZE > 0
static sLearnParameters gLearnParamsBuffer[LEARN_ACCEPT_CYCLES_BUFFER_SIZE] = {0};
static sLearnParameters *gLearnParamsPtrs[LEARN_ACCEPT_CYCLES_BUFFER_SIZE] = {0};
static sLearnParameters *gLearnParamsPtrTmp = NULL;

static volatile uint8_t gLearnParamsUpdated = 0;

#ifdef DEBUG
static volatile float gLearnRpm = 0;
static volatile float gLearnMap = 0;
static volatile float gLearnTps = 0;
#endif /* DEBUG */
#endif /* LEARN_ACCEPT_CYCLES_BUFFER_SIZE */

static int8_t ecu_can_process_message(const sCanRawMessage *message);
static void can_signals_update(const sParameters *parameters);

static void can_log_signals_send(const sParameters *parameters);
static void can_etc_signals_send(const sParameters *parameters);

#if defined(LEARN_ACCEPT_CYCLES_BUFFER_SIZE) && LEARN_ACCEPT_CYCLES_BUFFER_SIZE > 0
static sLearnParameters ecu_convert_learn_parameters(const sParameters * params);

static sLearnParameters ecu_convert_learn_parameters(const sParameters * params)
{
  sLearnParameters ret;

  ret.RPM = params->RPM;
  ret.ManifoldAirPressure = params->ManifoldAirPressure;
  ret.ThrottlePosition = params->ThrottlePosition;
  ret.FuelRatio = params->FuelRatio;
  ret.WishFuelRatio = params->WishFuelRatio;
  ret.Timestamp = Delay_Tick;

  return ret;
}
#endif /* LEARN_ACCEPT_CYCLES_BUFFER_SIZE */

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
  memset(&gEcuTempCorrections, 0, sizeof(gEcuTempCorrections));
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

  config_transform_corrections_to_progress(&gEcuCorrectionsProgress, &gEcuCorrections);

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

STATIC_INLINE float ecu_recalculate_air_density(float temperature)
{
  //Output in mg/cc
  float mg_cc3 = 1.2047f;
  mg_cc3 *= (273.0f + 20.0f) / (273.0f + temperature);
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

STATIC_INLINE void ecu_pid_update(uint8_t isidle, uint8_t running, float idle_wish_to_rpm_relation, uint32_t now)
{
  uint32_t table_number = gParameters.CurrentTable;
  sEcuTable *table = &gEcuTable[table_number];
  sMathInterpolateInput ipIdleWishToRpmRelation = math_interpolate_input(idle_wish_to_rpm_relation, table->idle_pids_rpm_koffs, table->idle_pids_rpm_koffs_count);

  math_pid_set_clamp(&gPidIdleIgnition, table->idle_ign_deviation_min, table->idle_ign_deviation_max);

  if(!running) {
    math_pid_reset(&gPidIdleValveAirFlow, now);
    math_pid_reset(&gPidIdleValveRpm, now);
    math_pid_reset(&gPidIdleThrottleAirFlow, now);
    math_pid_reset(&gPidIdleThrottleRpm, now);
    math_pid_reset(&gPidIdleIgnition, now);
  }

  if(isidle) {
    if(gEcuParams.useEtc) {
      math_pid_set_koffs(&gPidIdleThrottleAirFlow, math_interpolate_1d(ipIdleWishToRpmRelation, table->idle_throttle_to_massair_pid_p),
          math_interpolate_1d(ipIdleWishToRpmRelation, table->idle_throttle_to_massair_pid_i), math_interpolate_1d(ipIdleWishToRpmRelation, table->idle_throttle_to_massair_pid_d));
      math_pid_set_koffs(&gPidIdleThrottleRpm, math_interpolate_1d(ipIdleWishToRpmRelation, table->idle_throttle_to_rpm_pid_p),
          math_interpolate_1d(ipIdleWishToRpmRelation, table->idle_throttle_to_rpm_pid_i), math_interpolate_1d(ipIdleWishToRpmRelation, table->idle_throttle_to_rpm_pid_d));
    } else {
      math_pid_set_koffs(&gPidIdleThrottleAirFlow, 0, 0, 0);
      math_pid_set_koffs(&gPidIdleThrottleRpm, 0, 0, 0);
    }
    math_pid_set_koffs(&gPidIdleValveAirFlow, math_interpolate_1d(ipIdleWishToRpmRelation, table->idle_valve_to_massair_pid_p),
        math_interpolate_1d(ipIdleWishToRpmRelation, table->idle_valve_to_massair_pid_i), math_interpolate_1d(ipIdleWishToRpmRelation, table->idle_valve_to_massair_pid_d));
    math_pid_set_koffs(&gPidIdleValveRpm, math_interpolate_1d(ipIdleWishToRpmRelation, table->idle_valve_to_rpm_pid_p),
        math_interpolate_1d(ipIdleWishToRpmRelation, table->idle_valve_to_rpm_pid_i), math_interpolate_1d(ipIdleWishToRpmRelation, table->idle_valve_to_rpm_pid_d));
    math_pid_set_koffs(&gPidIdleIgnition, math_interpolate_1d(ipIdleWishToRpmRelation, table->idle_ign_to_rpm_pid_p),
        math_interpolate_1d(ipIdleWishToRpmRelation, table->idle_ign_to_rpm_pid_i), math_interpolate_1d(ipIdleWishToRpmRelation, table->idle_ign_to_rpm_pid_d));
  } else {
    math_pid_set_koffs(&gPidIdleThrottleAirFlow, 0, 0, 0);
    math_pid_set_koffs(&gPidIdleThrottleRpm, 0, 0, 0);
    math_pid_set_koffs(&gPidIdleValveAirFlow, 0, 0, 0);
    math_pid_set_koffs(&gPidIdleValveRpm, 0, 0, 0);
    math_pid_set_koffs(&gPidIdleIgnition, 0, 0, 0);
  }

  math_pid_set_koffs(&gPidShortTermCorr, table->short_term_corr_pid_p, table->short_term_corr_pid_i, table->short_term_corr_pid_d);
}

static void ecu_pid_init(void)
{
  uint32_t now = Delay_Tick;

  math_pid_init(&gPidIdleThrottleAirFlow);
  math_pid_init(&gPidIdleThrottleRpm);
  math_pid_init(&gPidIdleValveAirFlow);
  math_pid_init(&gPidShortTermCorr);
  math_pid_init(&gPidIdleValveRpm);
  math_pid_init(&gPidIdleIgnition);

  math_pid_set_clamp(&gPidIdleThrottleAirFlow, -10, 15);
  math_pid_set_clamp(&gPidIdleThrottleRpm, -10, 15);
  math_pid_set_clamp(&gPidIdleValveAirFlow, -IDLE_VALVE_POS_MAX, IDLE_VALVE_POS_MAX);
  math_pid_set_clamp(&gPidIdleValveRpm, -IDLE_VALVE_POS_MAX, IDLE_VALVE_POS_MAX);
  math_pid_set_clamp(&gPidShortTermCorr, -0.25f, 0.25f);

  ecu_pid_update(0, 0, 0, now);
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
float gDebugMapFiltered = 103000;
float gDebugAirTemp = 30.0f;
float gDebugEngineTemp = 90.0f;
float gDebugThrottle = 0;
float gDebugReferenceVoltage = 5.1f;
float gDebugPowerVoltage = 14.4f;
#endif

static void ecu_update(void)
{
  static const float normal_density = 1.2047f;
  static uint32_t adaptation_last = 0;
  static uint32_t phased_last = 0;
  static uint32_t running_time_last = 0;
  static float fuel_consumed = 0;
  static float km_driven = 0;
  static uint32_t updated_last = 0;
  static float idle_advance_correction = 0;
  static float idle_valve_pos_correction = 0;
  static float throttle_target_idle_correction = 0;
  static uint8_t is_cold_start = 1;
  static float cold_start_idle_temperature = 0.0f;
  uint32_t table_number = gParameters.CurrentTable;
  sEcuTable *table = &gEcuTable[table_number];
  uint8_t calibration = gEcuParams.performAdaptation;
  uint8_t calibrate_gbc;
  uint8_t calibration_permitted_to_perform = 0;
  uint8_t idle_calibration = gEcuParams.performIdleAdaptation;
  uint8_t use_idle_valve = gEcuParams.useIdleValve;
  uint8_t use_etc = gEcuParams.useEtc;
  uint8_t use_map_sensor = 1;
  uint8_t use_tps_sensor = 1;
  uint32_t now = Delay_Tick;
  uint32_t hal_now = HAL_GetTick();
  float adapt_diff = DelayDiff(now, adaptation_last);
  float diff = DelayDiff(now, updated_last);
  float diff_sec = diff * 0.000001f;
  static uint32_t enrichment_time_last = 0;
  float enrichment_time_diff_sec;

  sMathInterpolateInput ipRpm;
  sMathInterpolateInput ipIdleRpm;
  sMathInterpolateInput ipPressure;
  sMathInterpolateInput ipEngineTemp;
  sMathInterpolateInput ipColdStartTemp;
  sMathInterpolateInput ipCalcAirTemp;
  sMathInterpolateInput ipSpeed;
  sMathInterpolateInput ipThrottle;
  sMathInterpolateInput ipVoltages;
  sMathInterpolateInput ipEnrLoadStart;
  sMathInterpolateInput ipEnrLoadDeriv;

  sMathInterpolateInput ipFilling;
  sMathInterpolateInput ipLearnRpm;
  sMathInterpolateInput ipLearnThrottle;
  sMathInterpolateInput ipLearnPressure;

#if defined(LEARN_ACCEPT_CYCLES_BUFFER_SIZE) && LEARN_ACCEPT_CYCLES_BUFFER_SIZE > 0
  sMathInterpolateInput ipLearmParamsIndex = {0};
#endif /* LEARN_ACCEPT_CYCLES_BUFFER_SIZE */

  sMathInterpolateInput ipPedal;
  float throttle_target_pedal;
  float throttle_target_stop;
  float throttle_startup_move_time;
  float throttle_target_start;
  float throttle_target_idle;
  float throttle_startup_move_koff;
  float throttle_target_adaptation;
  float throttle_target_econ;
  float throttle_target;
  float pedal_ignition_control;
  float pedal_ignition_control_value;
  float pedal = 0;
  uint8_t throttle_position_use_1d = table->throttle_position_use_1d;

  static uint32_t short_term_last = 0;
  static uint32_t rotates_last = 0;
  uint32_t request_fill_last = gLocalParams.RequestFillLast;
  float tsps_relative_position_table;
  float tsps_desync_thr_table;

  float rpm, rps;
  float pressure;
  float fuel_ratio;
  float lambda_value;
  float lambda_temperature;
  float lambda_heatervoltage;
  float air_temp;
  float calculated_air_temp;
  float engine_to_air_temp_koff;
  float engine_temp;
  float throttle;
  float power_voltage;
  float reference_voltage;
  float speed;
  float idle_valve_position;
  float idle_valve_econ_position;
  float period;
  float period_half;
  float cold_start_idle_mult;
  float cold_start_idle_time;
  float cold_start_idle_corr;

  float fuel_ratio_diff;
  float wish_fuel_ratio;
  float wish_fuel_ratio_learn;
  float filling_select_koff_tps;
  float map_tps_relation;
  float filling;
  float filling_tps;
  float filling_map;
  float filling_map_correction;
  float filling_map_temp_correction;
  float filling_tps_correction;
  float filling_tps_temp_correction;
  float filling_map_corrected;
  float filling_gbc_tps_corrected;
  float filling_nmap_tps_koff;
  float effective_volume;
  float ignition_advance;
  float start_ignition_advance;
  float ignition_time;
  float injector_lag;
  float injector_lag_mult;
  float cycle_air_flow_map;
  float cycle_air_flow_tps;
  float cycle_air_flow;
  float cycle_air_flow_idle_diff;
  float filling_map_tps_diff;
  float air_temp_recalc_mult;
  float cycle_air_flow_injection;
  float cycle_fuel_flow_startup;
  float idle_wish_cycle_air_flow;
  float mass_air_flow;

  float injection_start_mult;
  float knock_threshold;
  float air_density;
  float engine_load;
  float estimated_power_hp;
  float estimated_torque_nm;
  float fuel_flow_per_us;
  float knock_noise_level;
  static float knock_zone = 0;
  static float knock_low_noise_state[ECU_CYLINDERS_COUNT] = {0};
  float knock_low_noise_lpf = 0;
  float knock_cy_level_diff;
  static float knock_cy_level_multiplier[ECU_CYLINDERS_COUNT] = {0};
  static float knock_cy_level_multiplier_correction[ECU_CYLINDERS_COUNT] = {0};
  float ignition_advance_cy[ECU_CYLINDERS_COUNT];
  float injection_correction_cy[ECU_CYLINDERS_COUNT];

  float min, max;
  static uint32_t prev_halfturns = 0;
  uint32_t halfturns;
  uint32_t halfturns_performed = 0;
  float running_time = 0;
  static float running_time_latest = 0;
  static float throttle_idle_time = 0;
  static float injection_phase = 100;
  static float short_term_correction = 0.0f;
  float short_term_correction_pid = 0.0f;
  float injection_phase_start;
  float injection_phase_table;
  float injection_phase_lpf;
  float fuel_amount_per_cycle;
  float warmup_fuel_correction;
  float additional_fuel_correction;
  float dynamic_film_correction;
  float dynamic_fuel_corr_gbc;
  float dynamic_fuel_corr_temp;
  float dynamic_fuel_corr_lpf;

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
  static float enrichment_tps_selection = 0;

  int32_t enrichment_load_type;
  float enrichment_load_dead_band;
  float enrichment_accel_dead_band;
  float enrichment_ign_corr_decay_time;
  float enrichment_detect_duration;
  float enrichment_async_pulses_divider;
  float enrichment_async_period;

  float enrichment_load_value_start = 0;
  static float enrichment_load_value_start_accept = 0;
  float enrichment_load_value = 0;
  static float enrichment_load_derivative_final = 0;
  static float enrichment_load_derivative_accept = 0;
  static float enrichment_time_pass = 0;
  static uint32_t enrichment_async_last = 0;
  float enrichment_load_diff;
  float enrichment_load_derivative;
  uint32_t enrichment_load_values_count = 0;
  uint32_t enrichment_load_values_divider = 0;
  static uint32_t enrichment_load_values_counter = 0;

  static float enleanment_load_derivative_final = 0;
  static float enleanment_load_derivative_accept = 0;
  static float enleanment_time_pass = 0;
  float enleanment_load_diff;
  float enleanment_load_derivative;

  float enrichment_lpf;
  static uint8_t enrichment_triggered = 0;
  static uint8_t enleanment_triggered = 0;
  uint8_t enrichment_triggered_once = 0;
  static uint8_t enrichment_triggered_async = 0;
  uint8_t enrichment_post_injection_enabled;
  static uint32_t enrichment_post_cycles = 0;
  static uint32_t enleanment_post_cycles = 0;
  static uint32_t idle_accelerate_post_cycles = 0;
  static uint32_t last_temp_fill_correction = 0;
  static uint32_t last_temp_knock_correction = 0;
  static uint32_t enrichment_halfturns = 0;
  uint8_t enrichment_process_trigger = 0;

  float warmup_mixture;
  float warmup_mix_koff;
  float warmup_idle_mix_corr;
  float warmup_idle_mix_corr_final;
  float air_temp_mix_corr;
  float air_temp_ign_corr;
  float engine_temp_mix_corr;
  float engine_temp_ign_corr;

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

  float calib_cur_progress;
  float percentage;
  float lpf_calculation;
  float knock_lpf_calculation;
  float filling_lpf_calculation;
  float idle_valve_pos_adaptation;
  float idle_valve_pos_dif;
  float ignition_correction;
  float ignition_knock_correction;
  float detonation_count_table;
  float ignition_corr_final;
  float fan_ign_corr;
  float fan_air_corr;
  float tsps_rel_pos = 0;
  float tsps_desync_thr;
  float start_async_filling;
  float start_large_filling;
  float start_small_filling;
  float start_filling_time;
  float start_filling_mult;
  float start_async_time;
  float idle_wish_to_rpm_relation;
  float idle_ignition_time_by_tps;
  float idle_ignition_time_by_tps_lpf;
  float idle_econ_delay;
  float start_econ_delay;
  static float idle_econ_time = 0;

  float calculate_air_temp_kmin;
  float calculate_air_temp_kmax;
  float calculate_air_flow_min;
  float calculate_air_flow_max;
  float calculate_air_flow_temp;
  float calculate_cycle_air_flow_max;
  float calculate_cycle_air_flow_temp;
  float abs_knock_ign_corr_max;
  float knock_detect_phase_start;
  float knock_detect_phase_end;

  float learn_cycles_to_delay;
  float learn_cycles_delay_mult;

  uint8_t etc_econ_flag;
  uint8_t econ_flag;
  uint8_t enrichment_async_enabled;
  uint8_t enrichment_sync_enabled;

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
  static uint8_t was_idle = 0;
  uint8_t idle_flag;
  uint8_t throttle_idle_flag;
  uint8_t throttle_large_idle_flag;
  uint8_t idle_corr_flag;
  uint8_t idle_pid_flag;
  uint8_t o2_valid = 0;
  uint8_t lambda_force = gForceParameters.LambdaForceEnabled;
  uint8_t phased_mode = gEcuParams.phasedMode;
  uint8_t shift_processing = Shift.Shifting;
  uint8_t cutoff_processing = Cutoff.Processing;
  static uint8_t knock_running = 0;
  static float knock_running_time = 0;
  sCspsData csps = csps_data();
  sO2Status o2_data = sens_get_o2_status();
  HAL_StatusTypeDef tps_status = HAL_OK;

  float sens_lpf_slow;
  float sens_lpf_fast;
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
  period = csps_getperiod(csps);
  period_half = period * 0.5f;

  gStatus.Sensors.Struct.Csps = csps_iserror() == 0 ? HAL_OK : HAL_ERROR;
  rpm = csps_getrpm(csps);
  rps = rpm * 0.01666667f;
  speed = speed_getspeed();
  knock_status = Knock_GetStatus();

  if(rotates)
    rotates_last = now;

  if(running) {
    running_time_latest += diff;
    if(DelayDiff(now, running_time_last) > 1000000) {
      running_time_last = now;
      gLocalParams.RunningTime++;
    }
  } else {
    running_time_last = now;
    running_time_latest = 0;
  }

  running_time = gLocalParams.RunningTime;
  if(running)
    running_time += (float)DelayDiff(now, running_time_last) * 0.000001f;

  while(halfturns != prev_halfturns) {
    halfturns_performed++;
    prev_halfturns++;
  }

  if(running) {
    sens_lpf_slow = rps * 2.0f * 0.001f;
    sens_lpf_fast = sens_lpf_slow * 2.0f;
    sens_lpf_slow = CLAMP(sens_lpf_slow, 0.0f, 1.0f);
    sens_lpf_fast = CLAMP(sens_lpf_fast, 0.0f, 1.0f);
  } else {
    sens_lpf_slow = 1.0f;
    sens_lpf_fast = 1.0f;
  }

#ifdef SIMULATION
  gStatus.Sensors.Struct.Tps = tps_status;
  gDebugMapFiltered = gDebugMap * sens_lpf_slow + gDebugMapFiltered * (1.0f - sens_lpf_slow);
  pressure = gDebugMapFiltered;
  air_temp = gDebugAirTemp;
  engine_temp = gDebugEngineTemp;
  throttle = gDebugThrottle;
  reference_voltage = gDebugReferenceVoltage;
  power_voltage = gDebugPowerVoltage;
#else
  gStatus.Sensors.Struct.PowerVoltage = sens_get_power_voltage(&power_voltage);
  gStatus.Sensors.Struct.ReferenceVoltage = sens_get_reference_voltage(&reference_voltage);
  if(use_etc) {
    tps_status = gStatus.Etc.Comm;
    tps_status |= gStatus.Etc.Tps1;
    tps_status |= gStatus.Etc.Tps2;
    tps_status |= gStatus.Etc.TpsMismatch;
    gStatus.Sensors.Struct.Tps = tps_status;
    throttle = gLocalParams.EtcThrottlePosition;
    pedal = gLocalParams.EtcPedalPosition;
    if(pedal <= gEcuParams.etcPedalDeadZone) {
      pedal = 0;
    } else {
      pedal = (pedal - gEcuParams.etcPedalDeadZone) * 0.01f * (100.0f + gEcuParams.etcPedalDeadZone);
    }
    pedal = CLAMP(pedal, 0.0f, 100.0f);
  } else {
    gStatus.Sensors.Struct.Tps = sens_get_throttle_position(&throttle);
  }
  gStatus.Sensors.Struct.Map = sens_get_map(&pressure);
  gStatus.Sensors.Struct.AirTemp = sens_get_air_temperature(&air_temp);
  gStatus.Sensors.Struct.EngineTemp = sens_get_engine_temperature(&engine_temp);
#endif

  gStatus.Sensors.Struct.Map |= sens_set_map_lpf(sens_lpf_slow);

  if(use_map_sensor && gStatus.Sensors.Struct.Map != HAL_OK) {
    use_map_sensor = 0;
  }

  if(use_tps_sensor && gStatus.Sensors.Struct.Tps != HAL_OK) {
    use_tps_sensor = 0;
  }

  if(gStatus.OutputDiagnostic.Injectors.Availability != HAL_OK || gStatus.OutputDiagnostic.Injectors.Diagnostic.Byte != HAL_OK) {
    injector_status = HAL_ERROR;
  } else {
    injector_status = HAL_OK;
  }

  if(found != was_found) {
    was_found = found;
    if(found) {
      was_start_async = 0;
    }
  }

  if(rotates != was_rotating) {
    was_rotating = rotates;
  }

  if(!rotates && use_map_sensor && use_tps_sensor) {
    if(DelayDiff(now, rotates_last) > 3000000) {
      if(pressure < 85000 && (idle_valve_position > 10 || throttle > 2.0f)) {
        gStatus.Sensors.Struct.Map = HAL_ERROR;
      } else if(pressure > 90000) {
        gStatus.Sensors.Struct.Map = HAL_OK;
      }
    }
  }

  ipRpm = math_interpolate_input(rpm, table->rotates, table->rotates_count);

  tsps_relative_position_table = math_interpolate_1d(ipRpm, table->tsps_relative_pos);
  tsps_desync_thr_table = math_interpolate_1d(ipRpm, table->tsps_desync_thr);

  if(csps_phased_valid()) {
    tsps_rel_pos = csps_gettspsrelpos() - tsps_relative_position_table;
  } else {
    tsps_rel_pos = 0;
  }

  tsps_desync_thr = fabsf(tsps_desync_thr_table);
  csps_tsps_set_expected_position(tsps_relative_position_table, tsps_desync_thr);

  if(gStatus.Sensors.Struct.EngineTemp != HAL_OK)
    engine_temp = 0.0f;
  if(gStatus.Sensors.Struct.AirTemp != HAL_OK)
    air_temp = 10.0f;

  fuel_ratio = table->fuel_afr;
  lambda_value = 1.0f;
  lambda_temperature = 0;
  lambda_heatervoltage = 0;
  if(gEcuParams.useLambdaSensor) {
    gStatus.Sensors.Struct.Lambda = sens_get_o2_labmda(&o2_data, &lambda_value, &o2_valid);
    gStatus.Sensors.Struct.Lambda |= sens_set_o2_lpf(sens_lpf_fast);
    if(gStatus.Sensors.Struct.Lambda == HAL_OK) {
      sens_get_o2_temperature(&o2_data, &lambda_temperature);
    }
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

  if(running && phased_mode == PhasedModeWithSensor && phased) {
    if(fabsf(tsps_rel_pos) >= tsps_desync_thr) {
      if(!gStatus.Tsps.sync_error) {
        gStatus.Tsps.sync_error_time = 0;
      } else {
        gStatus.Tsps.sync_error_time += HAL_DelayDiff(hal_now, gStatus.Tsps.sync_error_last);
      }
      gStatus.Tsps.sync_error = 1;
    } else if(fabsf(tsps_rel_pos) > 45.0f) {
      gStatus.Tsps.critical_error = 1;
    } else {
      gStatus.Tsps.sync_error = 0;
      gStatus.Tsps.critical_error = 0;
    }
  } else {
    gStatus.Tsps.sync_error = 0;
    gStatus.Tsps.critical_error = 0;
  }
  gStatus.Tsps.sync_error_last = hal_now;

  if(gEcuParams.useKnockSensor) {
    gStatus.Sensors.Struct.Knock = knock_status;
  } else {
    gStatus.Sensors.Struct.Knock = HAL_OK;
  }

  if(use_etc) {
    throttle_idle_flag = pedal <= 0.2f;
    throttle_large_idle_flag = pedal <= 5.0f;
  } else {
    throttle_idle_flag = throttle < 0.2f;
    throttle_large_idle_flag = throttle <= 1.0f;
  }

  if(rotates && running) {
    throttle_idle_time = 0;
  } else {
    if(throttle_idle_flag) {
      throttle_idle_time += diff_sec;
    } else {
      throttle_idle_time = 0;
    }
  }

  idle_flag = throttle_idle_flag && running;

  if(was_idle != idle_flag) {
    was_idle = idle_flag;
    idle_accelerate_post_cycles = 0;
  } else {
    for(int ht = 0; idle_accelerate_post_cycles < UINT_MAX && ht < halfturns_performed; ht++) {
      idle_accelerate_post_cycles++;
    }
  }

  ipIdleRpm = math_interpolate_input(rpm, table->idle_rotates, table->idle_rotates_count);
  ipThrottle = math_interpolate_input(throttle, table->throttles, table->throttles_count);
  ipEngineTemp = math_interpolate_input(engine_temp, table->engine_temps, table->engine_temp_count);
  ipSpeed = math_interpolate_input(speed, table->speeds, table->speeds_count);
  ipVoltages = math_interpolate_input(power_voltage, table->voltages, table->voltages_count);

  idle_wish_massair = math_interpolate_1d(ipEngineTemp, table->idle_wish_massair);

  ipPedal = math_interpolate_input(pedal, table->pedals, table->pedals_count);
  if(throttle_position_use_1d) {
    throttle_target_pedal = math_interpolate_1d(ipPedal, table->throttle_position_1d);
  } else {
    throttle_target_pedal = math_interpolate_2d_limit(ipPedal, ipRpm, TABLE_PEDALS_MAX, table->throttle_position);
  }
  throttle_target_start = math_interpolate_1d(ipEngineTemp, table->start_throttle_position);
  throttle_target_stop = math_interpolate_1d(ipPedal, table->stop_throttle_position);
  throttle_target_idle = math_interpolate_1d(ipEngineTemp, table->idle_throttle_position);
  throttle_target_adaptation = 0;
  pedal_ignition_control = math_interpolate_1d(ipRpm, table->pedal_ignition_control);
  throttle_startup_move_time = math_interpolate_1d(ipEngineTemp, table->throttle_startup_move_time);
  throttle_startup_move_time *= 1000000.0f;

  if(use_tps_sensor) {
    ipThrottle = math_interpolate_input(throttle, table->throttles, table->throttles_count);

    filling_tps = math_interpolate_2d_limit(ipRpm, ipThrottle, TABLE_ROTATES_MAX, table->filling_gbc_tps);
    filling_tps_correction = corr_math_interpolate_2d_func(ipRpm, ipThrottle, TABLE_ROTATES_MAX, gEcuCorrections.filling_gbc_tps);
    filling_tps_temp_correction = corr_math_interpolate_2d_func(ipRpm, ipThrottle, TABLE_ROTATES_MAX, gEcuTempCorrections.filling_gbc_tps);
    filling_tps_correction += filling_tps_temp_correction;
    filling_tps *= filling_tps_correction + 1.0f;
    filling_tps = MAX(filling_tps, 0.0f);
  } else {
    filling_tps = 0;
    filling_tps_correction = 0;
    filling_tps_temp_correction = 0;
  }

  if(use_map_sensor) {
    ipPressure = math_interpolate_input(pressure, table->pressures, table->pressures_count);

    filling_map = math_interpolate_2d_limit(ipRpm, ipPressure, TABLE_ROTATES_MAX, table->filling_gbc_map);
    filling_map_correction = corr_math_interpolate_2d_func(ipRpm, ipPressure, TABLE_ROTATES_MAX, gEcuCorrections.filling_gbc_map);
    filling_map_temp_correction = corr_math_interpolate_2d_func(ipRpm, ipPressure, TABLE_ROTATES_MAX, gEcuTempCorrections.filling_gbc_map);
    filling_map_correction += filling_map_temp_correction;
    filling_map *= filling_map_correction + 1.0f;
    filling_map = MAX(filling_map, 0.0f);
  } else {
    filling_map = 0;
    filling_map_correction = 0;
    filling_map_temp_correction = 0;
  }

  filling_select_koff_tps = math_interpolate_1d(ipRpm, table->filling_select_koff_tps);
  filling_select_koff_tps = CLAMP(filling_select_koff_tps, 0.0f, 1.0f);

  fuel_flow_per_us = table->injector_performance * 1.66666667e-5f * table->fuel_mass_per_cc; // mg/ms

  air_density = ecu_recalculate_air_density(air_temp);
  calculate_cycle_air_flow_max = gEcuParams.engineVolume * normal_density * 0.25f;

  if(use_map_sensor) {
    effective_volume = filling_map * gEcuParams.engineVolume;
  } else if(use_tps_sensor) {
    effective_volume = filling_tps * gEcuParams.engineVolume;
  } else {
    effective_volume = 0;
  }

  calculate_cycle_air_flow_temp = effective_volume * air_density * 0.25f;

  calculate_air_temp_kmin = gEcuParams.air_temp_corr_koff_min;
  calculate_air_temp_kmax = gEcuParams.air_temp_corr_koff_max;
  calculate_air_flow_min = idle_wish_massair;
  calculate_air_flow_max = calculate_cycle_air_flow_max * 0.00012f * table->rotates[table->rotates_count - 1];
  calculate_air_flow_temp = calculate_cycle_air_flow_temp * 0.00012f * rpm;

  calculate_air_flow_temp = CLAMP(calculate_air_flow_temp, calculate_air_flow_min, calculate_air_flow_max);

  engine_to_air_temp_koff = (calculate_air_flow_temp - calculate_air_flow_min) / (calculate_air_flow_max - calculate_air_flow_min);
  engine_to_air_temp_koff *= calculate_air_temp_kmax - calculate_air_temp_kmin;
  engine_to_air_temp_koff += calculate_air_temp_kmin;
  engine_to_air_temp_koff = CLAMP(engine_to_air_temp_koff, calculate_air_temp_kmin, calculate_air_temp_kmax);

  calculated_air_temp = air_temp * engine_to_air_temp_koff + engine_temp * (1.0f - engine_to_air_temp_koff);
  ipCalcAirTemp = math_interpolate_input(calculated_air_temp, table->air_temps, table->air_temp_count);

  air_density = ecu_recalculate_air_density(calculated_air_temp);

  learn_cycles_delay_mult = gEcuParams.learn_cycles_delay_mult;

  air_temp_recalc_mult = (273.0f + 20.0f) / (273.0f + calculated_air_temp);

  cycle_air_flow_map = filling_map;
  cycle_air_flow_map *= calculate_cycle_air_flow_max;
  cycle_air_flow_map *= air_temp_recalc_mult;

  cycle_air_flow_tps = filling_tps;
  cycle_air_flow_tps *= calculate_cycle_air_flow_max;
  cycle_air_flow_tps *= air_temp_recalc_mult;

  if(use_map_sensor && use_tps_sensor) {
    filling_nmap_tps_koff = filling_select_koff_tps;
  } else if(use_tps_sensor) {
    filling_nmap_tps_koff = 1.0f;
  } else if(use_map_sensor) {
    filling_nmap_tps_koff = 0.0f;
  } else {
    filling_nmap_tps_koff = 0.5f;
  }

  cycle_air_flow = cycle_air_flow_map * (1.0f - filling_nmap_tps_koff) + cycle_air_flow_tps * filling_nmap_tps_koff;
  cycle_air_flow = cycle_air_flow * (1.0f - enrichment_tps_selection) + cycle_air_flow_tps * enrichment_tps_selection;

  filling = cycle_air_flow / calculate_cycle_air_flow_max;
  learn_cycles_to_delay = 1.0f / filling;
  learn_cycles_to_delay *= learn_cycles_delay_mult;
  learn_cycles_to_delay *= ECU_CYLINDERS_COUNT;
  learn_cycles_to_delay = CLAMP(learn_cycles_to_delay, 0.0f, LEARN_ACCEPT_CYCLES_BUFFER_SIZE - 1);

  engine_load = filling * 100.0f;
  engine_load = MAX(engine_load, 0);

  mass_air_flow = rpm * cycle_air_flow * 0.00012f;
  idle_wish_cycle_air_flow = idle_wish_massair / 0.00012f / rpm;

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
  ignition_advance = math_interpolate_2d_limit(ipRpm, ipFilling, TABLE_ROTATES_MAX, table->ignitions);
  idle_ignition_time_by_tps = math_interpolate_1d(ipThrottle, table->idle_ignition_time_by_tps);
  idle_econ_delay = math_interpolate_1d(ipEngineTemp, table->idle_econ_delay);
  start_econ_delay = math_interpolate_1d(ipEngineTemp, table->start_econ_delay);

  ignition_correction = corr_math_interpolate_2d_func(ipRpm, ipFilling, TABLE_ROTATES_MAX, gEcuCorrections.ignitions);
  air_temp_ign_corr = math_interpolate_2d_limit(ipFilling, ipCalcAirTemp, TABLE_FILLING_MAX, table->air_temp_ign_corr);
  engine_temp_ign_corr = math_interpolate_2d_limit(ipFilling, ipEngineTemp, TABLE_FILLING_MAX, table->engine_temp_ign_corr);

  idle_wish_rpm = math_interpolate_1d(ipEngineTemp, table->idle_wish_rotates);
  idle_wish_ignition_static = math_interpolate_1d(ipIdleRpm, table->idle_wish_ignition_static);
  idle_wish_ignition_table = math_interpolate_1d(ipEngineTemp, table->idle_wish_ignition);
  idle_valve_pos_adaptation = math_interpolate_1d(ipEngineTemp, gEcuCorrections.idle_valve_position);

  idle_rpm_shift = math_interpolate_1d(ipSpeed, table->idle_rpm_shift);
  knock_noise_level = math_interpolate_1d(ipRpm, table->knock_noise_level);
  knock_threshold = math_interpolate_1d(ipRpm, table->knock_threshold);
  knock_detect_phase_start = math_interpolate_1d(ipRpm, table->knock_detect_phase_start);
  knock_detect_phase_end = math_interpolate_1d(ipRpm, table->knock_detect_phase_end);

  idle_wish_rpm += idle_rpm_shift;

  if(gForceParameters.Enable.WishIdleRPM)
    idle_wish_rpm = gForceParameters.WishIdleRPM;

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

  idle_corr_flag = throttle_large_idle_flag && running && idle_rpm_flag;
  idle_pid_flag  = idle_flag && idle_rpm_flag;
  econ_flag = idle_flag && !idle_rpm_flag;
  if(econ_flag)
    idle_econ_time += diff_sec;
  else idle_econ_time = 0;

  ignition_corr_final = ignition_correction;

  idle_rpm_flag_koff = diff_sec * 4.0f; // 0.25 sec
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
    pedal_ignition_control_value = 1.0f;
  } else {
    idle_wish_ignition = idle_wish_ignition_table * idle_rpm_flag_value + idle_wish_ignition_static * (1.0f - idle_rpm_flag_value);
    idle_ignition_time_by_tps_value = idle_flag * idle_ignition_time_by_tps_lpf + idle_ignition_time_by_tps_value * (1.0f - idle_ignition_time_by_tps_lpf);
    if(pedal_ignition_control > 0) {
      pedal_ignition_control_value = (pedal / pedal_ignition_control);
      pedal_ignition_control_value = CLAMP(pedal_ignition_control_value, 0.0f, 1.0f);
    } else {
      pedal_ignition_control_value = throttle_idle_flag ? 0.0f : 1.0f;
    }
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

  if(use_etc) {
    ignition_advance = ignition_advance * pedal_ignition_control_value + idle_wish_ignition  * (1.0f - pedal_ignition_control_value);
  } else {
    ignition_advance = idle_wish_ignition * idle_ignition_time_by_tps_value + ignition_advance * (1.0f - idle_ignition_time_by_tps_value);
  }

  ignition_advance += air_temp_ign_corr;
  ignition_advance += engine_temp_ign_corr;
  ignition_advance += ignition_corr_final;

  detonation_count_table = math_interpolate_2d_limit(ipRpm, ipFilling, TABLE_ROTATES_MAX, gEcuCorrections.knock_detonation_counter);
  if(!gForceParameters.Enable.IgnitionAdvance) {
    for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
      ignition_advance_cy[i] = math_interpolate_2d_limit(ipRpm, ipFilling, TABLE_ROTATES_MAX, table->ignition_corr_cy[i]);
      ignition_advance_cy[i] += math_interpolate_2d_limit(ipRpm, ipFilling, TABLE_ROTATES_MAX, gEcuCorrections.ignition_corr_cy[i]);
    }
  } else {
    memset(ignition_advance_cy, 0, sizeof(ignition_advance_cy));
  }

  if(!gForceParameters.Enable.InjectionPulse) {
    for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
      injection_correction_cy[i] = math_interpolate_2d_limit(ipRpm, ipFilling, TABLE_ROTATES_MAX, table->injection_corr_cy[i]);
      injection_correction_cy[i] += math_interpolate_2d_limit(ipRpm, ipFilling, TABLE_ROTATES_MAX, gEcuCorrections.injection_corr_cy[i]);
    }
  } else {
    memset(injection_correction_cy, 0, sizeof(injection_correction_cy));
  }

  wish_fuel_ratio = math_interpolate_2d_limit(ipRpm, ipFilling, TABLE_ROTATES_MAX, table->fuel_mixtures);
  injection_phase_table = math_interpolate_2d_limit(ipRpm, ipFilling, TABLE_ROTATES_MAX, table->injection_phase);

  injection_phase_start = math_interpolate_1d(ipEngineTemp, table->start_injection_phase);
  air_temp_mix_corr = math_interpolate_2d_limit(ipFilling, ipCalcAirTemp, TABLE_FILLING_MAX, table->air_temp_mix_corr);
  engine_temp_mix_corr = math_interpolate_2d_limit(ipFilling, ipEngineTemp, TABLE_FILLING_MAX, table->engine_temp_mix_corr);
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

  warmup_idle_mix_corr = math_interpolate_1d(ipEngineTemp, table->warmup_mix_corrs);
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
    math_pid_reset(&gPidShortTermCorr, now);
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
      math_pid_reset(&gPidShortTermCorr, now);
      short_term_correction = 0;
    }

  }

  start_large_count = table->start_large_count;
  if(use_etc) {
    injection_start_mult = math_interpolate_1d(ipThrottle, table->start_tps_corrs);
  } else {
    injection_start_mult = math_interpolate_1d(ipPedal, table->start_tps_corrs);
  }
  etc_econ_flag = econ_flag;
  econ_flag = econ_flag && gEcuParams.isEconEnabled && (idle_econ_time > idle_econ_delay) && (running_time > start_econ_delay);

  if(econ_flag) {
    idle_accelerate_post_cycles = 0;
  }

  for(int i = 0; i < halfturns_performed; i++) {
    start_halfturns++;
  }
  if(!rotates || gLocalParams.StartupInjectionStarted)
    start_halfturns = 0;

  if(start_halfturns < start_large_count)
    cycle_fuel_flow_startup = start_large_filling;
  else cycle_fuel_flow_startup = start_small_filling;

  cycle_air_flow_injection = cycle_air_flow;

  fuel_amount_per_cycle = cycle_air_flow_injection / wish_fuel_ratio;

  start_async_time = start_async_filling / fuel_flow_per_us;
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

  if(running) {
    cycle_air_flow_idle_diff = idle_wish_cycle_air_flow / cycle_air_flow;
    cycle_air_flow_idle_diff = CLAMP(cycle_air_flow_idle_diff, 0.0f, 1.0f);
    warmup_idle_mix_corr_final = cycle_air_flow_idle_diff * warmup_idle_mix_corr;
  } else {
    warmup_idle_mix_corr_final = 0.0f;
  }

  dynamic_fuel_corr_gbc = math_interpolate_1d(ipFilling, table->dynamic_fuel_corr_gbc);
  dynamic_fuel_corr_temp = math_interpolate_1d(ipEngineTemp, table->dynamic_fuel_corr_temp);
  dynamic_fuel_corr_lpf = math_interpolate_1d(ipRpm, table->dynamic_fuel_corr_lpf);
  dynamic_film_correction = dynamic_fuel_corr_gbc * dynamic_fuel_corr_temp;

  dynamic_fuel_corr_lpf = CLAMP(dynamic_fuel_corr_lpf, 0.0f, 0.99f);
  if(dynamic_film_correction < 0.0f)
    dynamic_film_correction = 0;

  additional_fuel_correction = 1.0f;
  additional_fuel_correction *= air_temp_mix_corr + 1.0f;
  additional_fuel_correction *= engine_temp_mix_corr + 1.0f;

  warmup_fuel_correction = 1.0f;
  warmup_fuel_correction *= cold_start_idle_mult + 1.0f;
  warmup_fuel_correction *= warmup_idle_mix_corr_final + 1.0f;

  fuel_amount_per_cycle *= additional_fuel_correction;
  fuel_amount_per_cycle *= warmup_fuel_correction;

  if(!running) {
    fuel_amount_per_cycle = cycle_fuel_flow_startup;
    fuel_amount_per_cycle *= injection_start_mult;
  } else {
    if(running_time < start_filling_time && cycle_fuel_flow_startup > fuel_amount_per_cycle) {
      start_filling_mult = ((cycle_fuel_flow_startup / fuel_amount_per_cycle) - 1.0f) * (1.0f - (running_time / start_filling_time));
    } else {
      start_filling_mult = 0.0f;
    }
    fuel_amount_per_cycle *= 1.0f + start_filling_mult;
  }

  enrichment_load_type = table->enrichment_load_type;
  enrichment_load_dead_band = table->enrichment_load_dead_band;
  enrichment_accel_dead_band = math_interpolate_1d(ipRpm, table->enrichment_accel_dead_band);
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

  enrichment_load_value = 0;
  if(enrichment_load_type == 0) {
    if(use_tps_sensor) {
      if(use_etc) {
        enrichment_load_value = gParameters.WishThrottleTargetPosition;
      } else {
        enrichment_load_value = throttle;
      }
    }
  }
  else if(enrichment_load_type == 1) {
    if(use_map_sensor) {
      enrichment_load_value = pressure;
    }
  }
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
      enrichment_halfturns = 0;
      enrichment_time_last = now;
      enrichment_triggered = 1;
      enrichment_ignition_state = 1;
      enrichment_phase_state = 1;
      enrichment_triggered_async = 1;
      enrichment_triggered_once = 1;
      enrichment_async_last = now;
    } else if(enrichment_triggered) {
      for(int ht = 0; enrichment_halfturns < UINT_MAX && ht < halfturns_performed; ht++) {
        enrichment_halfturns++;
      }
      while (enrichment_halfturns >= ECU_CYLINDERS_COUNT) {
        enrichment_halfturns -= ECU_CYLINDERS_COUNT;
        enrichment_process_trigger = 1;
      }

      if(enrichment_process_trigger || !running) {
        enrichment_time_diff_sec = DelayDiff(now, enrichment_time_last) * 0.000001f;
        enrichment_time_pass += enrichment_time_diff_sec;
        enrichment_time_last = now;
        enrichment_load_derivative_final = enrichment_load_derivative_accept - enrichment_accel_dead_band * enrichment_time_pass;
        if(enrichment_load_derivative_final <= 0) {
          enrichment_load_derivative_final = 0;
          enrichment_triggered = 0;
          enrichment_halfturns = 0;
        }
      }
    }

    enleanment_load_diff = min - max;
    enleanment_load_derivative = enleanment_load_diff / enrichment_lpf;

    if((!enleanment_triggered && enleanment_load_derivative >= enrichment_load_dead_band) ||
        (enleanment_triggered && (enleanment_load_derivative > enleanment_load_derivative_accept))) {
      enleanment_load_derivative_accept = enleanment_load_derivative;
      enleanment_load_derivative_final = enleanment_load_derivative;
      enleanment_time_pass = 0;
      enleanment_triggered = 1;
    } else if(enleanment_triggered) {
      enleanment_time_pass += diff_sec;
      enleanment_load_derivative_final = enleanment_load_derivative_accept - enrichment_accel_dead_band * enleanment_time_pass;
      if(enleanment_load_derivative_final <= 0) {
        enleanment_load_derivative_final = 0;
        enleanment_triggered = 0;
      }
    }

    if(enrichment_triggered || enleanment_triggered) {
      sens_reset_map_lpf();
    }

    if(enrichment_triggered || !running) {
      enrichment_post_cycles = 0;
    } else {
      for(int ht = 0; enrichment_post_cycles < UINT_MAX && ht < halfturns_performed; ht++) {
        enrichment_post_cycles++;
      }
    }

    if(enleanment_triggered || !running) {
      enleanment_post_cycles = 0;
    } else {
      for(int ht = 0; enleanment_post_cycles < UINT_MAX && ht < halfturns_performed; ht++) {
        enleanment_post_cycles++;
      }
    }

    ipEnrLoadStart = math_interpolate_input_limit(enrichment_load_value_start_accept, table->enrichment_rate_start_load, table->enrichment_rate_start_load_count);
    ipEnrLoadDeriv = math_interpolate_input_limit(enrichment_load_derivative_final, table->enrichment_rate_load_derivative, table->enrichment_rate_load_derivative_count);

    enrichment_rate = math_interpolate_2d_limit(ipEnrLoadDeriv, ipEnrLoadStart, TABLE_ENRICHMENT_PERCENTS_MAX, table->enrichment_rate);
    enrichment_tps_selection = math_interpolate_2d_limit(ipEnrLoadDeriv, ipEnrLoadStart, TABLE_ENRICHMENT_PERCENTS_MAX, table->enrichment_tps_selection);
    enrichment_rate *= enrichment_temp_mult;

    if(!running) {
      enrichment_tps_selection = 0.0f;
    }

    enrichment_ign_corr = math_interpolate_2d_limit(ipEnrLoadStart, ipRpm, TABLE_ENRICHMENT_PERCENTS_MAX, table->enrichment_ign_corr);

    if(enrichment_sync_enabled) {
      enrichment_amount_sync = math_interpolate_1d(ipRpm, table->enrichment_sync_amount);
      enrichment_amount_sync *= enrichment_rate;
    }

    if(enrichment_async_enabled) {
      enrichment_amount_async = math_interpolate_1d(ipRpm, table->enrichment_async_amount);
      enrichment_amount_async *= enrichment_rate;
    }
  } else {
    enleanment_load_derivative_final = 0;
    enrichment_amount_sync = 0;
    enrichment_load_values_counter = 0;
    enrichment_phase_state = 0;
    enrichment_ignition_state = 0;
    enrichment_ign_corr = 0;
    enrichment_triggered_async = 0;
    enrichment_async_last = now;
    enleanment_triggered = 0;
  }

  enrichment_result = enrichment_amount_sync + enrichment_amount_async;

  if(running && enrichment_async_enabled) {
    enrichment_async_period = period_half / enrichment_async_pulses_divider;
    if(enrichment_amount_async >= 0.001f) {
      enrichment_async_time = 0;
      if(DelayDiff(now, enrichment_async_last) >= enrichment_async_period || enrichment_triggered_async) {
        enrichment_async_time = gLocalParams.FinalInjectionPulse - injector_lag_mult;
        enrichment_triggered_async = 0;
        enrichment_async_last = now;
      }
      if(enrichment_async_time > 0) {
        enrichment_async_time *= enrichment_amount_async;
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

  if(running) {
    if(idle_flag)
      fuel_amount_per_cycle *= gEcuCorrections.idle_correction + 1.0f;
    else
      fuel_amount_per_cycle *= gEcuCorrections.long_term_correction + 1.0f;
    fuel_amount_per_cycle *= short_term_correction + 1.0f;
  }

  fuel_amount_per_cycle *= enrichment_amount_sync + 1.0f;

  if(gForceParameters.Enable.InjectionPulse) {
    gLocalParams.FinalInjectionPulse = gForceParameters.InjectionPulse;
    if(gLocalParams.FinalInjectionPulse > injector_lag_mult) {
      fuel_amount_per_cycle = (gLocalParams.FinalInjectionPulse - injector_lag_mult) * fuel_flow_per_us;
      wish_fuel_ratio = cycle_air_flow / fuel_amount_per_cycle;
    } else {
      fuel_amount_per_cycle = 0;
      wish_fuel_ratio = 200.0f;
    }
  }

  if(Cutoff.FuelCutoff) {
    fuel_amount_per_cycle = 0;
  } else if(gStatus.OilPressure.is_error && gStatus.OilPressure.error_time > 500 && rpm >= gEcuParams.oilPressureCutoffRPM) {
    fuel_amount_per_cycle = 0;
  } else if(gStatus.OilPressure.is_error && gStatus.OilPressure.error_time > 10000) {
    fuel_amount_per_cycle = 0;
  }

  if(running && running_time_latest >= FUEL_FILM_MINIMUM_RUNTIME) {
    gLocalParams.FuelFilmAllowed = 1;
  } else {
    gLocalParams.FuelFilmAllowed = 0;
  }

  gLocalParams.InjectorLag = injector_lag_mult;
  gLocalParams.FuelFlowPerUs = fuel_flow_per_us;
  gLocalParams.CycleAirFlow = cycle_air_flow_injection;
  gLocalParams.CycleFuelFlow = fuel_amount_per_cycle;
  gLocalParams.DynamicFilmCorrection = dynamic_film_correction;
  gLocalParams.DynamicFilmCorrLpf = dynamic_fuel_corr_lpf;
  if(enrichment_post_injection_enabled && enrichment_triggered_once)
    gLocalParams.EnrichmentTriggered = 1;

  math_pid_set_target(&gPidIdleValveAirFlow, idle_wish_massair);
  math_pid_set_target(&gPidIdleValveRpm, idle_wish_rpm);
  math_pid_set_target(&gPidIdleThrottleAirFlow, idle_wish_massair);
  math_pid_set_target(&gPidIdleThrottleRpm, idle_wish_rpm);
  math_pid_set_target(&gPidIdleIgnition, idle_wish_rpm);

  idle_valve_econ_position = math_interpolate_1d(ipRpm, table->idle_valve_econ_position);
  throttle_target_econ = math_interpolate_1d(ipRpm, table->idle_throttle_econ_position);

  if (running_time_latest >= throttle_startup_move_time || throttle_startup_move_time <= 0.0f) {
    throttle_startup_move_koff = 1.0f;
  } else {
    throttle_startup_move_koff = running_time_latest / throttle_startup_move_time;
  }
  throttle_startup_move_koff = CLAMP(throttle_startup_move_koff, 0.0f, 1.0f);

  if(running) {
    throttle_target = throttle_target_idle * throttle_startup_move_koff + throttle_target_start * (1.0f - throttle_startup_move_koff);
    throttle_target += throttle_target_pedal * 0.01f * (100.0f - throttle_target);

    idle_table_valve_pos = math_interpolate_1d(ipEngineTemp, table->idle_valve_position);
    idle_table_valve_pos *= idle_valve_pos_adaptation + 1.0f;
    math_pid_set_clamp(&gPidIdleValveAirFlow, table->idle_valve_pos_min, table->idle_valve_pos_max);
    math_pid_set_clamp(&gPidIdleValveRpm, table->idle_valve_pos_min, table->idle_valve_pos_max);
    math_pid_set_clamp(&gPidIdleThrottleAirFlow, table->idle_throttle_pos_min, table->idle_throttle_pos_max);
    math_pid_set_clamp(&gPidIdleThrottleRpm, table->idle_throttle_pos_min, table->idle_throttle_pos_max);

    if(running_time_latest >= PID_MINIMUM_RUNTIME) {
      if(halfturns_performed) {
        idle_advance_correction = math_pid_update(&gPidIdleIgnition, rpm, now);
      }

      if(use_idle_valve && use_map_sensor) {
        if(halfturns_performed) {
          idle_valve_pos_correction = math_pid_update(&gPidIdleValveAirFlow, mass_air_flow, now);
          idle_valve_pos_correction += math_pid_update(&gPidIdleValveRpm, rpm, now);
        }
      } else {
        math_pid_reset(&gPidIdleValveAirFlow, now);
        math_pid_reset(&gPidIdleValveRpm, now);
        idle_valve_pos_correction = 0;
      }

      if(use_etc && use_map_sensor && use_tps_sensor) {
        if(halfturns_performed) {
          throttle_target_idle_correction = math_pid_update(&gPidIdleThrottleAirFlow, mass_air_flow, now);
          throttle_target_idle_correction += math_pid_update(&gPidIdleThrottleRpm, rpm, now);
        }
      } else {
        math_pid_reset(&gPidIdleThrottleAirFlow, now);
        math_pid_reset(&gPidIdleThrottleRpm, now);
        throttle_target_idle_correction = 0;
      }
    } else {
      math_pid_reset(&gPidIdleValveAirFlow, now);
      math_pid_reset(&gPidIdleValveRpm, now);
      math_pid_reset(&gPidIdleThrottleAirFlow, now);
      math_pid_reset(&gPidIdleThrottleRpm, now);
      idle_valve_pos_correction = 0;
      throttle_target_idle_correction = 0;
    }
  } else {
    idle_table_valve_pos = math_interpolate_1d(ipEngineTemp, table->start_idle_valve_pos);
    throttle_target = throttle_target_start;
    throttle_target += throttle_target_stop * 0.01f * (100.0f - throttle_target);
  }

  idle_wish_valve_pos = idle_table_valve_pos;

  ecu_pid_update(idle_pid_flag, running, idle_wish_to_rpm_relation, now);

  if(idle_pid_flag) {
    idle_wish_valve_pos += idle_valve_pos_correction;
    throttle_target += throttle_target_adaptation;
    throttle_target += throttle_target_idle_correction;
    ignition_advance += idle_advance_correction;
  }

  if (running) {
    idle_wish_valve_pos = CLAMP(idle_wish_valve_pos, 0, IDLE_VALVE_POS_MAX);

    if(!idle_rpm_flag) {
      idle_wish_valve_pos = idle_valve_econ_position;
    }
    if(etc_econ_flag) {
      throttle_target = throttle_target_econ;
      throttle_target += throttle_target_pedal * 0.01f * (100.0f - throttle_target);
    }
  }

  if(gForceParameters.Enable.IgnitionAdvance)
    ignition_advance = gForceParameters.IgnitionAdvance;
  if(gForceParameters.Enable.IgnitionOctane && !idle_flag)
    ignition_advance += gForceParameters.IgnitionOctane;

  if(idle_flag && running) {
    if(gForceParameters.Enable.WishIdleIgnitionAdvance) {
      math_pid_reset(&gPidIdleIgnition, now);
      ignition_advance = gForceParameters.WishIdleIgnitionAdvance;
      idle_advance_correction = 0;
    }
  }

  if(gForceParameters.Enable.WishIdleValvePosition) {
    math_pid_reset(&gPidIdleValveAirFlow, now);
    math_pid_reset(&gPidIdleValveRpm, now);
    idle_valve_pos_correction = 0;
    idle_wish_valve_pos = gForceParameters.WishIdleValvePosition;
  }

  if(gForceParameters.Enable.MaximumThrottlePosition) {
    if(throttle_target > gForceParameters.MaximumThrottlePosition) {
      throttle_target = gForceParameters.MaximumThrottlePosition;
    }
    gParameters.MaximumThrottlePosition = gForceParameters.MaximumThrottlePosition;
  } else {
    gParameters.MaximumThrottlePosition = 100.0f;
  }

  if(gForceParameters.Enable.WishThrottleTargetPosition) {
    math_pid_reset(&gPidIdleThrottleAirFlow, now);
    math_pid_reset(&gPidIdleThrottleRpm, now);
    throttle_target_idle_correction = 0;
    throttle_target = gForceParameters.WishThrottleTargetPosition;
  }

  throttle_target = CLAMP(throttle_target, 0.0f, 100.0f);

  fuel_amount_per_cycle = gLocalParams.FinalInjectionPulse * fuel_flow_per_us;

  idle_wish_valve_pos = CLAMP(idle_wish_valve_pos, 0, IDLE_VALVE_POS_MAX);

  if(!gIgnCanShutdown) {
	  out_set_idle_valve(roundf(idle_wish_valve_pos));
	  if (throttle_target <= 0.001f && gEtcTest.StartedTime == 0) {
      gParameters.EtcMotorFullCloseFlag = 1;
	  } else {
      gParameters.EtcMotorFullCloseFlag = 0;
	  }
	  if(throttle_idle_time > 10.0f && gEtcTest.StartedTime == 0) {
	    gParameters.EtcMotorActiveFlag = 0;
	  } else {
      gParameters.EtcMotorActiveFlag = 1;
	  }
  }

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
        gStatus.Knock.VoltagesCorrected[i] = gStatus.Knock.Voltages[i] * knock_cy_level_multiplier[i];
        gStatus.Knock.Denoised[i] = gStatus.Knock.VoltagesCorrected[i] - knock_noise_level;
        gStatus.Knock.Detonates[i] = (gStatus.Knock.Denoised[i] / knock_cy_level_multiplier[i]) - knock_threshold;
        if(gStatus.Knock.Denoised[i] < 0.0f)
          gStatus.Knock.Denoised[i] = 0.0f;
        if(gStatus.Knock.Detonates[i] < 0.0f) {
          gStatus.Knock.Detonates[i] = 0.0f;
        } else {
          gStatus.Knock.DetonationSaturation += gStatus.Knock.Detonates[i];
        }
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
      if(gStatus.Knock.AdaptationDetonates[i] < gStatus.Knock.Detonates[i])
        gStatus.Knock.AdaptationDetonates[i] = gStatus.Knock.Detonates[i];
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

  if(!use_map_sensor && !use_tps_sensor) {
    running = 0;
    rotates = 0;
  }

  km_drive = speed * 2.77777778e-10f * diff; // speed / (60 * 60 * 1.000.000) * diff
  km_driven += km_drive;

  if(!rotates) {
    running = 0;
    cycle_air_flow = 0;
    mass_air_flow = 0;
    effective_volume = 0;
    fuel_consumption_per_distance = 0;
    fuel_consumption_hourly = 0;
    estimated_power_hp = 0;
    estimated_torque_nm = 0;
  } else {
    fuel_consumption = fuel_amount_per_cycle / table->fuel_mass_per_cc * (diff / period) * 0.001f * 2.0f;
    fuel_consumed += fuel_consumption;

    fuel_consumption_per_distance = fuel_consumption / km_drive * 100.0f;
    fuel_consumption_hourly = fuel_consumption * 3600000.0f;

    estimated_power_hp = mass_air_flow * 0.34722222f; // mass_air_flow / 3.6f * 1.25f
    estimated_torque_nm = estimated_power_hp / (rpm * 0.0001404156f);
  }

  if(!running) {
    ignition_advance = start_ignition_advance;
  }

  if(gLocalParams.FinalInjectionDutyCycle > 0.90f) {
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

  gStatus.Knock.GeneralStatus = KnockStatusOk;
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
            gStatus.Knock.CylinderStatus[i] |= KnockStatusDedonation;
            gStatus.Knock.DetonationLastCy[i] = hal_now;
            if(gStatus.Knock.Detonates[i] > 0.7f) {
              gStatus.Knock.CylinderStatus[i] |= KnockStatusStrongDedonation;
              gStatus.Knock.StrongDetonationLastCy[i] = hal_now;
            }
            gStatus.Knock.Advances[i] += (gStatus.Knock.Period[i] * 0.000001f * 3.3f) * gStatus.Knock.Detonates[i];  //0.3 seconds to advance
          } else {
            if(gStatus.Knock.Advances[i] > 0.0f) {
              gStatus.Knock.Advances[i] -= gStatus.Knock.Period[i] * 0.000001f * 0.1f; //10 seconds to restore
            }
            knock_low_noise_lpf = gStatus.Knock.Period[i] * 0.000001f * 0.33f; // 3.0 seconds
            knock_low_noise_lpf = CLAMP(knock_low_noise_lpf, 0.0f, 0.2f);
            if(gStatus.Knock.VoltagesCorrected[i] < knock_noise_level * 0.5f) {
              knock_low_noise_state[i] = knock_low_noise_state[i] * (1.0f - knock_low_noise_lpf) + knock_low_noise_lpf;
            } else {
              knock_low_noise_state[i] *= 1.0f - knock_low_noise_lpf;
            }
          }
          gStatus.Knock.Advances[i] = CLAMP(gStatus.Knock.Advances[i], 0.0f, 1.0f);
          gStatus.Knock.UpdatedInternally[i] = 0;
        }

        if(knock_low_noise_state[i] >= 0.5f && engine_temp >= KNOCK_LOW_NOISE_ON_ENGINE_TEMP_THRESHOLD) {
          gStatus.Knock.CylinderStatus[i] |= KnockStatusLowNoise;
          gStatus.Knock.LowNoiseLastCy[i] = hal_now;
        }
        if((gStatus.Knock.CylinderStatus[i] & KnockStatusLowNoise) == KnockStatusLowNoise) {
            gStatus.Knock.Advances[i] = 1.0f;
        }
      }
    } else {
      knock_zone = 0.0f;
      memset(knock_low_noise_state, 0, sizeof(knock_low_noise_state));
      memset(gStatus.Knock.Advances, 0, sizeof(gStatus.Knock.Advances));
      memset(gStatus.Knock.Detonates, 0, sizeof(gStatus.Knock.Detonates));
      memset(gStatus.Knock.Voltages, 0, sizeof(gStatus.Knock.Voltages));
      memset(gStatus.Knock.Updated, 0, sizeof(gStatus.Knock.Updated));
      memset(gStatus.Knock.AdvancesFinal, 0, sizeof(gStatus.Knock.AdvancesFinal));
      memset(gStatus.Knock.VoltagesCorrected, 0, sizeof(gStatus.Knock.VoltagesCorrected));
      memset(gStatus.Knock.UpdatedInternally, 0, sizeof(gStatus.Knock.UpdatedInternally));
      memset(gStatus.Knock.UpdatedAdaptation, 0, sizeof(gStatus.Knock.UpdatedAdaptation));

      knock_running_time += diff;
      if(knock_running_time >= 1000000) {
        knock_running = 1;
        knock_running_time = 0;
      }
    }

    gStatus.Knock.Advance = 0;
    for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
      if(gStatus.Knock.Advances[i] > gStatus.Knock.Advance)
        gStatus.Knock.Advance = gStatus.Knock.Advances[i];

      if(gStatus.Knock.StrongDetonationLastCy[i] && HAL_DelayDiff(hal_now, gStatus.Knock.StrongDetonationLastCy[i]) > 5000) {
        gStatus.Knock.CylinderStatus[i] &= ~KnockStatusStrongDedonation;
        gStatus.Knock.StrongDetonationLastCy[i] = 0;
      }
      if(gStatus.Knock.DetonationLastCy[i] && HAL_DelayDiff(hal_now, gStatus.Knock.DetonationLastCy[i]) > 5000) {
        gStatus.Knock.CylinderStatus[i] &= ~KnockStatusDedonation;
        gStatus.Knock.DetonationLastCy[i] = 0;
      }
      if(gStatus.Knock.LowNoiseLastCy[i] && HAL_DelayDiff(hal_now, gStatus.Knock.LowNoiseLastCy[i]) > 5000) {
        gStatus.Knock.CylinderStatus[i] &= ~KnockStatusLowNoise;
        gStatus.Knock.LowNoiseLastCy[i] = 0;
      }
    }

    gStatus.Knock.DetonationRelation = gStatus.Knock.DetonationCountPerSecond / rpm * 30.0f;

  } else {
    knock_zone = 0.0f;
    knock_running = 0;
    knock_running_time = 0;
    gStatus.Knock.DetonationRelation = 0;
    memset(knock_low_noise_state, 0, sizeof(knock_low_noise_state));
    memset(gStatus.Knock.Advances, 0, sizeof(gStatus.Knock.Advances));
    memset(gStatus.Knock.Detonates, 0, sizeof(gStatus.Knock.Detonates));
    memset(gStatus.Knock.Voltages, 0, sizeof(gStatus.Knock.Voltages));
    memset(gStatus.Knock.Updated, 0, sizeof(gStatus.Knock.Updated));
    memset(gStatus.Knock.UpdatedInternally, 0, sizeof(gStatus.Knock.UpdatedInternally));
    memset(gStatus.Knock.UpdatedAdaptation, 0, sizeof(gStatus.Knock.UpdatedAdaptation));
    memset(gStatus.Knock.AdaptationDetonates, 0, sizeof(gStatus.Knock.AdaptationDetonates));
    memset(gStatus.Knock.CylinderStatus, 0, sizeof(gStatus.Knock.CylinderStatus));
    memset(gStatus.Knock.StrongDetonationLastCy, 0, sizeof(gStatus.Knock.StrongDetonationLastCy));
    memset(gStatus.Knock.DetonationLastCy, 0, sizeof(gStatus.Knock.DetonationLastCy));
    memset(gStatus.Knock.LowNoiseLastCy, 0, sizeof(gStatus.Knock.LowNoiseLastCy));
  }

  for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
    gStatus.Knock.GeneralStatus |= gStatus.Knock.CylinderStatus[i];
  }

  if(gStatus.Knock.DetonationRelation > 0.1f) {
    gStatus.Knock.GeneralStatus |= KnockStatusContinuousDedonation;
  } else {
    gStatus.Knock.GeneralStatus &= ~KnockStatusContinuousDedonation;
  }

  wish_fuel_ratio_learn = wish_fuel_ratio;
  fuel_ratio_diff = fuel_ratio / wish_fuel_ratio_learn;
  filling_map_tps_diff = filling_map / filling_tps;
  abs_knock_ign_corr_max = fabsf(table->knock_ign_corr_max);
  calibrate_gbc = engine_temp >= GBC_CALIBRATION_ON_ENGINE_TEMP_THRESHOLD || lambda_force;

#if defined(LEARN_ACCEPT_CYCLES_BUFFER_SIZE) && LEARN_ACCEPT_CYCLES_BUFFER_SIZE > 0

  if(gLearnParamsUpdated) {
    gLearnParamsUpdated = 0;

    ipLearmParamsIndex.size = LEARN_ACCEPT_CYCLES_BUFFER_SIZE;
    ipLearmParamsIndex.indexes[0] = roundf(learn_cycles_to_delay);
    ipLearmParamsIndex.indexes[1] = ipLearmParamsIndex.indexes[0] + 1;

    ipLearmParamsIndex.mult = learn_cycles_to_delay - ipLearmParamsIndex.indexes[0];

    ipLearmParamsIndex.indexes[0] = CLAMP(ipLearmParamsIndex.indexes[0], 0, LEARN_ACCEPT_CYCLES_BUFFER_SIZE - 1);
    ipLearmParamsIndex.indexes[1] = CLAMP(ipLearmParamsIndex.indexes[1], 0, LEARN_ACCEPT_CYCLES_BUFFER_SIZE - 1);

    wish_fuel_ratio_learn = math_interpolate_1d_offset(ipLearmParamsIndex, &gLearnParamsPtrs[0]->WishFuelRatio, sizeof(sLearnParameters));
    fuel_ratio_diff = fuel_ratio / wish_fuel_ratio_learn;

#else /* LEARN_ACCEPT_CYCLES_BUFFER_SIZE */
  if(halfturns_performed) {
#endif /* !LEARN_ACCEPT_CYCLES_BUFFER_SIZE */
    adaptation_last = now;
    if(running && running_time_latest >= CALIBRATION_MIN_RUNTIME) {
      calibration_permitted_to_perform = enleanment_post_cycles > LEARN_ENLEANMENT_POST_CYCLES_DELAY &&
          enrichment_post_cycles > LEARN_ENRICHMENT_POST_CYCLES_DELAY &&
          idle_accelerate_post_cycles >= IDLE_ACCELERATE_POST_CYCLES_DELAY;

      lpf_calculation = adapt_diff * 0.000001f * 1.4142f; // * sqrt(2)
      if(lpf_calculation > 0.1f)
        lpf_calculation = 0.1f;
      knock_lpf_calculation = lpf_calculation;
      filling_lpf_calculation = lpf_calculation;

      if(calibration && corr_math_interpolate_2d_set_func) {
        if(gEcuParams.useLambdaSensor && gStatus.Sensors.Struct.Lambda == HAL_OK && injector_status == HAL_OK && o2_valid &&
            (idle_calibration || !idle_corr_flag) && calibration_permitted_to_perform) {
          gEcuCorrections.long_term_correction = 0.0f;
          gEcuCorrections.idle_correction = 0.0f;
          short_term_correction = 0.0f;
          short_term_correction_pid = 0.0f;

#if defined(LEARN_ACCEPT_CYCLES_BUFFER_SIZE) && LEARN_ACCEPT_CYCLES_BUFFER_SIZE > 0
          float learn_rpm = math_interpolate_1d_offset(ipLearmParamsIndex, &gLearnParamsPtrs[0]->RPM, sizeof(sLearnParameters));
          float learn_map = math_interpolate_1d_offset(ipLearmParamsIndex, &gLearnParamsPtrs[0]->ManifoldAirPressure, sizeof(sLearnParameters));
          float learn_tps = math_interpolate_1d_offset(ipLearmParamsIndex, &gLearnParamsPtrs[0]->ThrottlePosition, sizeof(sLearnParameters));

#ifdef DEBUG
          gLearnRpm = learn_rpm;
          gLearnMap = learn_map;
          gLearnTps = learn_tps;
#endif /* DEBUG */

          ipLearnRpm = math_interpolate_input(learn_rpm, table->rotates, table->rotates_count);
          ipLearnPressure = math_interpolate_input(learn_map, table->pressures, table->pressures_count);
          ipLearnThrottle = math_interpolate_input(learn_tps, table->throttles, table->throttles_count);

#else /* LEARN_ACCEPT_CYCLES_BUFFER_SIZE */
          ipLearnRpm = ipRpm;
          ipLearnPressure = ipPressure;
          ipLearnThrottle = ipThrottle;
#endif /* !LEARN_ACCEPT_CYCLES_BUFFER_SIZE */

          if((use_map_sensor || use_tps_sensor) && !econ_flag && calibrate_gbc) {

            filling_map_corrected = fuel_ratio_diff * (1.0f - filling_nmap_tps_koff) + filling_map_tps_diff * filling_nmap_tps_koff;
            filling_gbc_tps_corrected = fuel_ratio_diff * filling_nmap_tps_koff + filling_map_tps_diff * (1.0f - filling_nmap_tps_koff);

            if(idle_corr_flag) {
              filling_lpf_calculation *= 0.4f; // 2.5 sec
            }

            percentage = fuel_ratio_diff;
            if(percentage > 1.0f) percentage = 1.0f / percentage;

            if(use_map_sensor) {
              filling_map_temp_correction += (filling_map_corrected - 1.0f) * filling_lpf_calculation * (1.0f - filling_nmap_tps_koff);
              corr_math_interpolate_2d_set_func(ipLearnRpm, ipLearnPressure, TABLE_ROTATES_MAX, gEcuTempCorrections.filling_gbc_map, filling_map_temp_correction, -1.0f, 1.0f);
              calib_cur_progress = corr_math_interpolate_2d_func(ipLearnRpm, ipLearnPressure, TABLE_ROTATES_MAX, gEcuCorrectionsProgress.progress_filling_gbc_map);
              calib_cur_progress = (percentage * filling_lpf_calculation) + (calib_cur_progress * (1.0f - filling_lpf_calculation));
              corr_math_interpolate_2d_set_func(ipLearnRpm, ipLearnPressure, TABLE_ROTATES_MAX, gEcuCorrectionsProgress.progress_filling_gbc_map, calib_cur_progress, 0.0f, 1.0f);
            }

            if(use_tps_sensor) {
              filling_tps_temp_correction += (filling_gbc_tps_corrected - 1.0f) * filling_lpf_calculation;
              corr_math_interpolate_2d_set_func(ipLearnRpm, ipLearnThrottle, TABLE_ROTATES_MAX, gEcuTempCorrections.filling_gbc_tps, filling_tps_temp_correction, -1.0f, 1.0f);
              calib_cur_progress = corr_math_interpolate_2d_func(ipLearnRpm, ipLearnThrottle, TABLE_ROTATES_MAX, gEcuCorrectionsProgress.progress_filling_gbc_tps);
              calib_cur_progress = (percentage * filling_lpf_calculation) + (calib_cur_progress * (1.0f - filling_lpf_calculation));
              corr_math_interpolate_2d_set_func(ipLearnRpm, ipLearnThrottle, TABLE_ROTATES_MAX, gEcuCorrectionsProgress.progress_filling_gbc_tps, calib_cur_progress, 0.0f, 1.0f);
            }
          }
        }

        if(idle_flag && use_map_sensor && use_tps_sensor &&
            !gForceParameters.Enable.WishIdleValvePosition && !gForceParameters.Enable.WishIdleMassAirFlow && use_idle_valve) {
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
                gStatus.Knock.UpdatedAdaptation[i] = 0;

                if(gStatus.Knock.AdaptationDetonates[i] > 0.05f) {
                  gStatus.Knock.AdaptationDetonates[i] = 0;
                  detonation_count_table += 1.0f;

                  knock_lpf_calculation *= 5.00f * 1.4142f; //1 degree per 0.2 sec * sqrt(2)

                  calib_cur_progress = 0.0f;

                  ignition_advance_cy[i] -= knock_zone * knock_lpf_calculation;
                  corr_math_interpolate_2d_set_func(ipRpm, ipFilling, TABLE_ROTATES_MAX, gEcuCorrections.ignition_corr_cy[i], ignition_advance_cy[i], -abs_knock_ign_corr_max, abs_knock_ign_corr_max);

                  math_interpolate_2d_set(ipRpm, ipFilling, TABLE_ROTATES_MAX, gEcuCorrections.knock_detonation_counter, detonation_count_table, 0.0f, 99.0f);
                } else {
#if defined(KNOCK_DETONATION_INCREASING_ADVANCE) && KNOCK_DETONATION_INCREASING_ADVANCE > 0
                  if(detonation_count_table < 3.0f) {
                    knock_lpf_calculation *= 0.2f; //5 sec
                    ignition_advance_cy[i] = -table->knock_ign_corr_max * knock_zone * knock_lpf_calculation + ignition_advance_cy[i] * (1.0f - knock_lpf_calculation);
                    corr_math_interpolate_2d_set_func(ipRpm, ipFilling, TABLE_ROTATES_MAX, gEcuCorrections.ignition_corr_cy[i], ignition_advance_cy[i], -abs_knock_ign_corr_max, abs_knock_ign_corr_max);
                  }
#endif /* KNOCK_DETONATION_INCREASING_ADVANCE */
                  calib_cur_progress = (1.0f * knock_lpf_calculation) + (calib_cur_progress * (1.0f - knock_lpf_calculation));
                }
                corr_math_interpolate_2d_set_func(ipRpm, ipFilling, TABLE_ROTATES_MAX, gEcuCorrectionsProgress.progress_ignitions, calib_cur_progress, 0.0f, 1.0f);
              }
            }
          } else if(idle_flag) {
            if(engine_temp >= KNOCK_LOW_NOISE_ON_ENGINE_TEMP_THRESHOLD && knock_running) {
              for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
                if(gStatus.Knock.UpdatedAdaptation[i]) {

                  if(gStatus.Knock.VoltagesCorrected[i] > knock_noise_level * 0.2f) {
                    knock_lpf_calculation *= 0.3f; //3 sec

                    knock_cy_level_diff = knock_noise_level;
                    knock_cy_level_diff /= gStatus.Knock.VoltagesCorrected[i];
                    knock_cy_level_diff -= 1.0f;

                    knock_cy_level_multiplier_correction[i] += knock_cy_level_diff * knock_lpf_calculation;
                    math_interpolate_1d_set(ipRpm, gEcuCorrections.knock_cy_level_multiplier[i], knock_cy_level_multiplier_correction[i], -1.0f, 1.0f);

                    calib_cur_progress = math_interpolate_1d(ipRpm, gEcuCorrectionsProgress.progress_knock_cy_level_multiplier[i]);
                    calib_cur_progress = (1.0f * knock_lpf_calculation) + (calib_cur_progress * (1.0f - knock_lpf_calculation));
                    math_interpolate_1d_set(ipRpm, gEcuCorrectionsProgress.progress_knock_cy_level_multiplier[i], calib_cur_progress, 0.0f, 1.0f);
                  }
                  gStatus.Knock.UpdatedAdaptation[i] = 0;
                }
              }
            } else {
              memset(gStatus.Knock.UpdatedAdaptation, 0, sizeof(gStatus.Knock.UpdatedAdaptation));
            }
          } else {
            memset(gStatus.Knock.UpdatedAdaptation, 0, sizeof(gStatus.Knock.UpdatedAdaptation));
          }
        }
        memset(gStatus.Knock.AdvancesFinal, 0, sizeof(gStatus.Knock.AdvancesFinal));
      } else {
        memset(gStatus.Knock.AdaptationDetonates, 0, sizeof(gStatus.Knock.AdaptationDetonates));
        for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
          gStatus.Knock.AdvancesFinal[i] = gStatus.Knock.Advances[i] * table->knock_ign_corr_max * knock_zone;
        }
      }

      if(!calibration || (!idle_calibration && idle_flag) || !calibrate_gbc) {
        if(gEcuParams.useLambdaSensor && gStatus.Sensors.Struct.Lambda == HAL_OK && injector_status == HAL_OK && o2_valid && !econ_flag) {
          if(gEcuParams.useLongTermCorr) {
            if(calibration_permitted_to_perform) {
              if(!idle_flag) {
                filling_lpf_calculation *= 0.1f * 1.4142f; //10 sec * sqrt(2)
                gEcuCorrections.long_term_correction += ((fuel_ratio_diff - 1.0f) + short_term_correction) * filling_lpf_calculation;
              }
              else if(idle_corr_flag) {
                filling_lpf_calculation *= 0.3333f * 1.4142f; //3 sec * sqrt(2)
                gEcuCorrections.idle_correction += ((fuel_ratio_diff - 1.0f) + short_term_correction) * filling_lpf_calculation;
              }
            }
          } else {
            gEcuCorrections.long_term_correction = 0;
            gEcuCorrections.idle_correction = 0;
          }
        }
      } else {
        gEcuCorrections.long_term_correction = 0;
        gEcuCorrections.idle_correction = 0;
      }
    }
  }

  if(DelayDiff(now, last_temp_fill_correction) >= 200000) {
    last_temp_fill_correction = now;

    for(int y = 0; y < table->pressures_count; y++) {
      for(int x = 0; x < table->rotates_count; x++) {
        filling_map_correction = gEcuCorrections.filling_gbc_map[y][x];
        filling_map_correction += gEcuTempCorrections.filling_gbc_map[y][x];
        filling_map_correction = CLAMP(filling_map_correction, -1.0f, 1.0f);
        gEcuCorrections.filling_gbc_map[y][x] = filling_map_correction;
        gEcuTempCorrections.filling_gbc_map[y][x] = 0;
      }
    }

    for(int y = 0; y < table->throttles_count; y++) {
      for(int x = 0; x < table->rotates_count; x++) {
        filling_tps_correction = gEcuCorrections.filling_gbc_tps[y][x];
        filling_tps_correction += gEcuTempCorrections.filling_gbc_tps[y][x];
        filling_tps_correction = CLAMP(filling_tps_correction, -1.0f, 1.0f);
        gEcuCorrections.filling_gbc_tps[y][x] = filling_tps_correction;
        gEcuTempCorrections.filling_gbc_tps[y][x] = 0;
      }
    }
  } else if(DelayDiff(now, last_temp_knock_correction) >= 500000) {
    last_temp_knock_correction = now;

    for(int y = 0; y < table->fillings_count; y++) {
      for(int x = 0; x < table->rotates_count; x++) {
        ignition_knock_correction = -FLT_MAX;
        for(int c = 0; c < ECU_CYLINDERS_COUNT; c++) {
          ignition_advance_cy[c] = gEcuCorrections.ignition_corr_cy[c][y][x];
          if(ignition_advance_cy[c] > ignition_knock_correction) {
            ignition_knock_correction = ignition_advance_cy[c];
          }
        }
        for(int c = 0; c < ECU_CYLINDERS_COUNT; c++) {
          ignition_advance_cy[c] -= ignition_knock_correction;
          gEcuCorrections.ignition_corr_cy[c][y][x] = ignition_advance_cy[c];
        }
        ignition_knock_correction += gEcuCorrections.ignitions[y][x];
        ignition_knock_correction = CLAMP(ignition_knock_correction, -abs_knock_ign_corr_max, abs_knock_ign_corr_max);
        gEcuCorrections.ignitions[y][x] = ignition_knock_correction;
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

  if(use_map_sensor && use_tps_sensor) {
    if(running &&
        ((!idle_flag || (rpm > idle_reg_rpm_2 && engine_temp > 80.0f)))) {
      map_tps_relation = cycle_air_flow_map / cycle_air_flow_tps;
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
  gParameters.KnockSaturation = gStatus.Knock.DetonationSaturation;
  gParameters.AirTemp = air_temp;
  gParameters.EngineTemp = engine_temp;
  gParameters.CalculatedAirTemp = calculated_air_temp;
  gParameters.ManifoldAirPressure = pressure;

  gParameters.ThrottlePosition = throttle;
  if(use_etc) {
    gParameters.PedalPosition = pedal;
    gParameters.ThrottleTargetPosition = gLocalParams.EtcThrottleTargetPosition;
    gParameters.ThrottleDefaultPosition = gLocalParams.EtcThrottleDefaultPosition;

    if(gEtcTest.StartedTime == 0) {
      gParameters.WishThrottleTargetPosition = throttle_target;
    }
  } else {
    gParameters.WishThrottleTargetPosition = throttle;
    gParameters.PedalPosition = 0;
    gParameters.ThrottleTargetPosition = 0;
    gParameters.ThrottleDefaultPosition = 0;
  }
  gParameters.ReferenceVoltage = reference_voltage;
  gParameters.PowerVoltage = power_voltage;
  gParameters.FuelRatio = fuel_ratio;
  gParameters.FuelRatioDiff = fuel_ratio_diff;
  gParameters.LambdaValue = lambda_value;
  gParameters.LambdaTemperature = lambda_temperature;
  gParameters.LambdaHeaterVoltage = lambda_heatervoltage;
  gParameters.ShortTermCorrection = short_term_correction;
  gParameters.LongTermCorrection = gEcuCorrections.long_term_correction;
  gParameters.IdleCorrection = gEcuCorrections.idle_correction;

  gParameters.IdleFlag = idle_flag;
  gParameters.IdleCorrFlag = idle_corr_flag;
  gParameters.IdleEconFlag = econ_flag;
  gParameters.RPM = rpm;
  gParameters.Speed = speed;
  gParameters.MassAirFlow = mass_air_flow;
  gParameters.CyclicAirFlow = cycle_air_flow;
  gParameters.EffectiveVolume = effective_volume;
  gParameters.EngineLoad = engine_load;
  gParameters.EstimatedPower = estimated_power_hp;
  gParameters.EstimatedTorque = estimated_torque_nm;
  gParameters.WishFuelRatio = wish_fuel_ratio;
  gParameters.IdleValvePosition = idle_valve_position;
  gParameters.IdleRegThrRPM = idle_reg_rpm_1;
  gParameters.WishIdleRPM = idle_wish_rpm;
  gParameters.WishIdleMassAirFlow = idle_wish_massair;
  gParameters.WishIdleValvePosition = idle_wish_valve_pos;
  gParameters.WishIdleIgnitionAdvance = idle_wish_ignition;
  gParameters.IgnitionAdvance = ignition_advance;
  gParameters.InjectionPhase = gLocalParams.AcceptedInjectionPhase;
  gParameters.InjectionPhaseDuration = gLocalParams.FinalInjectionPhaseDuration;
  gLocalParams.RequestedInjectionPhase = injection_phase;

  for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
    gLocalParams.IgnitionCorrectionCy[i] = ignition_advance_cy[i];
    gLocalParams.InjectionCorrectionCy[i] = injection_correction_cy[i];
  }

  gParameters.InjectionPulse = gLocalParams.FinalInjectionPulse;
  gParameters.InjectionDutyCycle = gLocalParams.FinalInjectionDutyCycle;

  gLocalParams.KnockDetectPhaseStart = knock_detect_phase_start;
  gLocalParams.KnockDetectPhaseEnd = knock_detect_phase_end;

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

  //gDiagWorkingMode.Bits.is_enrichment = enrichment_triggered;
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
  int8_t status;

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
      if(gBackupSaveReq && !gBackupSaveRes) {
        state++;
      }
      break;
    case 1:
      if(!CRC16_IsBusy()) {
        status = config_save_corrections(&gEcuCorrections);
        if(status) {
          state = 0;
          gBackupSaveReq = 0;
          gBackupSaveRes = 1;
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
  uint8_t rotates = csps_isrotates();
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
  float angle_ignite_param;
  float inj_phase_temp;
  static float angle_ignite = 10.0f;
  static float inj_phase = 200.0f;
  float inj_phase_req;
  float inj_phase_adder;
  float saturate;
  float angles_injection_per_turn;
  float angles_ignition_per_turn;
  float inj_pulse;
  float inj_pulse_final;
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

#if defined(LEARN_ACCEPT_CYCLES_BUFFER_SIZE) && LEARN_ACCEPT_CYCLES_BUFFER_SIZE > 0
  uint8_t injection_performed = 0;
#endif /* LEARN_ACCEPT_CYCLES_BUFFER_SIZE */

  static uint32_t async_inject_time = 0;
  static uint32_t async_inject_last = 0;
  float inj_lag, injection_dutycycle;
  float cycle_fuel_flow;
  float dynamic_film_correction;
  float dynamic_fuel_corr_lpf;
  float inj_phase_duration;
  uint8_t fuel_film_allowed;

  float phased_angles[ECU_CYLINDERS_COUNT];
  float non_phased_angles[ECU_CYLINDERS_COUNT_HALF];

  float cy_corr_injection[ECU_CYLINDERS_COUNT];
  float cy_corr_ignition[ECU_CYLINDERS_COUNT];

#ifndef SIMULATION
  throttle_status = sens_get_throttle_position(&throttle);
#else
  throttle = gDebugThrottle;
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
  phased_ignition = use_phased && !single_coil;
  phased_knock = use_phased;
  cy_count_injection = phased_injection ? ECU_CYLINDERS_COUNT : ECU_CYLINDERS_COUNT_HALF;
  cy_count_ignition = phased_ignition ? ECU_CYLINDERS_COUNT : ECU_CYLINDERS_COUNT_HALF;
  cy_count_knock = phased_knock ? ECU_CYLINDERS_COUNT : ECU_CYLINDERS_COUNT_HALF;
  angle_ignite_param = gParameters.IgnitionAdvance;
  injection_phase_by_end = table->is_fuel_phase_by_end;

  inj_lag = gLocalParams.InjectorLag;
  cycle_fuel_flow = gLocalParams.CycleFuelFlow;
  dynamic_film_correction = gLocalParams.DynamicFilmCorrection;
  dynamic_fuel_corr_lpf = gLocalParams.DynamicFilmCorrLpf;
  fuel_film_allowed = gLocalParams.FuelFilmAllowed;

  if(found != was_found) {
    was_found = found;
    last_start_triggered = 1;
    last_start = now;
    turns_count_last = turns_count;
  }

  if(!running) {
    gLocalParams.AcceptedInjectionPhase = gLocalParams.RequestedInjectionPhase;
  }

  if(!rotates) {
    gLocalParams.StartupInjectionStarted = 0;
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
  } else if(!phased_ignition || !individual_coils) {
    angle_ignite_koff = 0.01f;
  }

  uspa_koff = diff * 0.002f;

  uspa = uspa_raw * uspa_koff + uspa * (1.0f - uspa_koff);
  r_uspa = 1.0f / uspa;
  angle_ignite = angle_ignite_param * angle_ignite_koff + angle_ignite * (1.0f - angle_ignite_koff);
  inj_phase_req = gLocalParams.RequestedInjectionPhase;
  inj_phase = gLocalParams.AcceptedInjectionPhase;
  inj_phase_temp = inj_phase;

  if(single_coil) {
    time_sat = period_half * 0.65f;
    time_pulse = period_half * 0.35f;
  } else {
    time_sat = gParameters.IgnitionPulse;
    time_pulse = 2500;
  }

  for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
    cy_corr_ignition[i] = table->cy_corr_ignition[i];
    cy_corr_ignition[i] += gLocalParams.IgnitionCorrectionCy[i];
    cy_corr_ignition[i] += gStatus.Knock.AdvancesFinal[i];
  }

  for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
    cy_corr_injection[i] = table->cy_corr_injection[i];
    cy_corr_injection[i] *= gLocalParams.InjectionCorrectionCy[i] + 1.0f;
  }

  if(phased_ignition) {
    angles_ignition_per_turn = 720.0f;
    for(int i = 0; i < cy_count_ignition; i++) {
      var = angle_ignite;
      var += cy_corr_ignition[i];
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
      var += MIN(cy_corr_ignition[i], cy_corr_ignition[ECU_CYLINDERS_COUNT - 1 - i]);
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
  } else {
    angles_injection_per_turn = 360.0f;
  }

  for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
    gLocalParams.FuelFilmTemp[i] = cycle_fuel_flow * dynamic_film_correction;
  }

  if(!fuel_film_allowed) {
    for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
      gLocalParams.FuelFilmOld[i] = gLocalParams.FuelFilmTemp[i];
      gLocalParams.FuelFilmNew[i] = gLocalParams.FuelFilmTemp[i];
    }
  }

  inj_pulse = 0.0f;
  for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
    cy_injection[i] = cycle_fuel_flow;
    if(fuel_film_allowed) {
      cy_injection[i] += gLocalParams.FuelFilmNew[i] - gLocalParams.FuelFilmOld[i];
    }
    cy_injection[i] = MAX(cy_injection[i], 0.0f);
    cy_injection[i] /= gLocalParams.FuelFlowPerUs;
    cy_injection[i] *= cy_corr_injection[i] + 1.0f;
    inj_pulse += cy_injection[i];
  }
  inj_pulse /= ECU_CYLINDERS_COUNT;
  inj_pulse_final = inj_pulse;

  if(!phased_injection) {
    inj_pulse *= 0.5f;

    for(int i = 0; i < ECU_CYLINDERS_COUNT_HALF; i++) {
      injection[ECU_CYLINDERS_COUNT - 1 - i] = 1;
      injected[ECU_CYLINDERS_COUNT - 1 - i] = 1;

      cy_injection[i] *= 0.5f;
    }
  }

  if(inj_pulse > 0.0f && inj_pulse_final > 0.0f) {
    inj_pulse_final += inj_lag;
    inj_pulse += inj_lag;

    for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
      if(cy_injection[i] > 0.0f) {
        cy_injection[i] += inj_lag;
      } else {
        cy_injection[i] = 0.0f;
      }
    }
  }

  if(gForceParameters.Enable.InjectionPulse) {
    inj_pulse_final = gForceParameters.InjectionPulse;
    for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
      cy_injection[i] = inj_pulse_final;
    }
  }

  injection_dutycycle = inj_pulse_final / (period * 2.0f);

  if(injection_dutycycle > 1.0f) {
    inj_pulse_final = period * 2.0f;
  }

  inj_phase_duration = inj_pulse_final / uspa;
  gLocalParams.FinalInjectionPulse = inj_pulse_final;
  gLocalParams.FinalInjectionDutyCycle = injection_dutycycle;
  gLocalParams.FinalInjectionPhaseDuration = inj_phase_duration;

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
    for(int i = 0; i < ECU_CYLINDERS_COUNT_HALF; i++) {
      angle_ignition[i] = non_phased_angles[i];
    }
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

        //Knock part
        if(running) {
          if(knock_busy && knock_process[knock_cylinder]) {
            if(angle_knock[knock_cylinder] < gLocalParams.KnockDetectPhaseStart || angle_knock[knock_cylinder] >= gLocalParams.KnockDetectPhaseEnd) {
              Knock_SetState(KnockStateHold);
              knock_process[knock_cylinder] = 0;
              knock_busy = 0;
            }
          }
          if(!knock_busy) {
            for(int i = 0; i < cy_count_knock; i++) {
              if(angle_knock[i] >= gLocalParams.KnockDetectPhaseStart && angle_knock[i] < gLocalParams.KnockDetectPhaseEnd) {
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
                if(shift_ign_act && cutoff_ign_act) {
                  if(individual_coils) {
                    ecu_coil_saturate(cy_count_ignition, i);
                  } else {
                    ecu_coil_saturate(cy_count_ignition, i);
                    ecu_coil_saturate(cy_count_ignition, ECU_CYLINDERS_COUNT - 1 - i);
                  }
                }
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
                  if(individual_coils) {
                    ecu_coil_ignite(cy_count_ignition, i);
                  } else {
                    ecu_coil_ignite(cy_count_ignition, i);
                    ecu_coil_ignite(cy_count_ignition, ECU_CYLINDERS_COUNT - 1 - i);
                  }
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
              if(/*ignition_ready[i] && */cutoff_inj_act && shift_inj_act && cy_injection[i] > inj_lag * 1.1f && !econ_flag) {
                if(phased_injection) {
                  ecu_inject(ECU_CYLINDERS_COUNT, i, cy_injection[i]);
                } else {
                  i_inv = ECU_CYLINDERS_COUNT - i - 1;

                  //Camshaft position detection
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
                            if(cy_injection[i] > inj_lag * 1.1f) {
                              phase_detect_cycles++;
                            }
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
                injection_time_prev[i] = cy_injection[i] - inj_lag;
              } else {
                if(phased_injection) {
                  gLocalParams.FuelFilmTemp[i] = gLocalParams.FuelFilmOld[i] / (1.0f + dynamic_film_correction);
                } else {
                  i_inv = ECU_CYLINDERS_COUNT - i - 1;
                  gLocalParams.FuelFilmTemp[i] = gLocalParams.FuelFilmOld[i] / (1.0f + dynamic_film_correction);
                  gLocalParams.FuelFilmTemp[i_inv] = gLocalParams.FuelFilmOld[i_inv] / (1.0f + dynamic_film_correction);
                }
              }

              if(phased_injection) {
                gLocalParams.FuelFilmOld[i] = gLocalParams.FuelFilmNew[i] * (1.0f - dynamic_fuel_corr_lpf) + gLocalParams.FuelFilmOld[i] * dynamic_fuel_corr_lpf;
                gLocalParams.FuelFilmNew[i] = gLocalParams.FuelFilmTemp[i];
              } else {
                gLocalParams.FuelFilmOld[i] = gLocalParams.FuelFilmNew[i] * (1.0f - dynamic_fuel_corr_lpf) + gLocalParams.FuelFilmOld[i] * dynamic_fuel_corr_lpf;
                gLocalParams.FuelFilmNew[i] = gLocalParams.FuelFilmTemp[i];
                gLocalParams.FuelFilmOld[i_inv] = gLocalParams.FuelFilmNew[i_inv] * (1.0f - dynamic_fuel_corr_lpf) + gLocalParams.FuelFilmOld[i_inv] * dynamic_fuel_corr_lpf;
                gLocalParams.FuelFilmNew[i_inv] = gLocalParams.FuelFilmTemp[i_inv];
              }

              gLocalParams.ParametersAcceptLast = 0;
              gParameters.CylinderInjectionBitmask ^= 1 << i;
#if defined(LEARN_ACCEPT_CYCLES_BUFFER_SIZE) && LEARN_ACCEPT_CYCLES_BUFFER_SIZE > 0
              injection_performed = 1 << i;
#endif /* LEARN_ACCEPT_CYCLES_BUFFER_SIZE */
              enrichment_triggered[i] = 0;

#if defined(IGNITION_ACCEPTION_FEATURE) && IGNITION_ACCEPTION_FEATURE > 0
              ignition_accepted_angle[i] = cy_ignition[i];
#endif
              inj_phase_adder = fabsf(inj_phase_req - inj_phase);
              if(inj_phase_adder > INJ_PHASE_CHANGE_PER_CYCLE) {
                inj_phase_adder = INJ_PHASE_CHANGE_PER_CYCLE;
              }

              if(inj_phase_req < inj_phase) {
                inj_phase_adder = -inj_phase_adder;
              }
              gLocalParams.AcceptedInjectionPhase += inj_phase_adder;
              gLocalParams.StartupInjectionStarted = 1;
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

          //Post Injection
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
              gLocalParams.ParametersAcceptLast = 0;

              // TODO: Probably, no need to accept the fuel film here
              //if(phased_injection) {
              //gLocalParams.FuelFilmOld[i] = gLocalParams.FuelFilmNew[i] * (1.0f - dynamic_fuel_corr_lpf) + gLocalParams.FuelFilmOld[i] * dynamic_fuel_corr_lpf;
              //gLocalParams.FuelFilmNew[i] = gLocalParams.FuelFilmTemp[i];
              //} else {
              //gLocalParams.FuelFilmOld[i] = gLocalParams.FuelFilmNew[i] * (1.0f - dynamic_fuel_corr_lpf) + gLocalParams.FuelFilmOld[i] * dynamic_fuel_corr_lpf;
              //gLocalParams.FuelFilmNew[i] = gLocalParams.FuelFilmTemp[i];
              //gLocalParams.FuelFilmOld[i_inv] = gLocalParams.FuelFilmNew[i_inv] * (1.0f - dynamic_fuel_corr_lpf) + gLocalParams.FuelFilmOld[i_inv] * dynamic_fuel_corr_lpf;
              //gLocalParams.FuelFilmNew[i_inv] = gLocalParams.FuelFilmTemp[i_inv];
              //}

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

    memset(gLocalParams.FuelFilmNew, 0, sizeof(gLocalParams.FuelFilmNew));
    memset(gLocalParams.FuelFilmOld, 0, sizeof(gLocalParams.FuelFilmOld));
    memset(gLocalParams.FuelFilmTemp, 0, sizeof(gLocalParams.FuelFilmTemp));

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
    memset(gLocalParams.FuelFilmNew, 0, sizeof(gLocalParams.FuelFilmNew));
    memset(gLocalParams.FuelFilmOld, 0, sizeof(gLocalParams.FuelFilmOld));
    memset(gLocalParams.FuelFilmTemp, 0, sizeof(gLocalParams.FuelFilmTemp));
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

    memset(gLocalParams.FuelFilmNew, 0, sizeof(gLocalParams.FuelFilmNew));
    memset(gLocalParams.FuelFilmOld, 0, sizeof(gLocalParams.FuelFilmOld));
    memset(gLocalParams.FuelFilmTemp, 0, sizeof(gLocalParams.FuelFilmTemp));
  }

#if defined(LEARN_ACCEPT_CYCLES_BUFFER_SIZE) && LEARN_ACCEPT_CYCLES_BUFFER_SIZE > 0
  if(injection_performed) {
    gLearnParamsPtrTmp = gLearnParamsPtrs[LEARN_ACCEPT_CYCLES_BUFFER_SIZE - 1];
    for(int i = LEARN_ACCEPT_CYCLES_BUFFER_SIZE - 2; i >= 0; i--) {
      gLearnParamsPtrs[i + 1] = gLearnParamsPtrs[i];
    }
    gLearnParamsPtrs[0] = gLearnParamsPtrTmp;
    *gLearnParamsPtrTmp = ecu_convert_learn_parameters(&gParameters);
    gLearnParamsUpdated = 1;
  }
#endif /* LEARN_ACCEPT_CYCLES_BUFFER_SIZE */
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
  fan_advance_control = math_interpolate_2d_limit(ipSpeed, ipEngineTemp, TABLE_SPEEDS_MAX, table->fan_advance_control);

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
  uint8_t ignored = 0;
  uint8_t iserror = 0;
  uint8_t status_reset = gStatusReset;
  GPIO_PinState ign_status = sens_get_ign(NULL);

  memset(gCheckBitmap, 0, sizeof(gCheckBitmap));

  CHECK_STATUS(iserror, CheckFlashLoadFailure, gStatus.Flash.Struct.Load != HAL_OK || gStatus.Bkpsram.Struct.CorrsSave != HAL_OK);
  CHECK_STATUS(iserror, CheckFlashSaveFailure, gStatus.Flash.Struct.Save != HAL_OK || gStatus.Bkpsram.Struct.CorrsLoad != HAL_OK);
  CHECK_STATUS(iserror, CheckFlashInitFailure, gStatus.Flash.Struct.Init != HAL_OK);
  CHECK_STATUS(ignored, CheckBkpsramSaveFailure, gStatus.Bkpsram.Struct.CriticalSave != HAL_OK);
#ifdef DEBUG
  CHECK_STATUS(ignored, CheckBkpsramLoadFailure, gStatus.Bkpsram.Struct.CriticalLoad != HAL_OK);
#endif

  CHECK_STATUS(iserror, CheckSensorMapFailure, gStatus.Sensors.Struct.Map != HAL_OK);
  CHECK_STATUS(iserror, CheckSensorKnockFailure, gStatus.Sensors.Struct.Knock != HAL_OK);
  CHECK_STATUS(iserror, CheckSensorCspsFailure, gStatus.Sensors.Struct.Csps != HAL_OK);
  CHECK_STATUS(iserror, CheckSensorTspsFailure, gStatus.Sensors.Struct.Tsps != HAL_OK);
  CHECK_STATUS(iserror, CheckSensorAirTempFailure, gStatus.Sensors.Struct.AirTemp != HAL_OK);
  CHECK_STATUS(iserror, CheckSensorEngineTempFailure, gStatus.Sensors.Struct.EngineTemp != HAL_OK);
  CHECK_STATUS(iserror, CheckSensorTPSFailure, gStatus.Sensors.Struct.Tps != HAL_OK);
  CHECK_STATUS(iserror, CheckSensorRefVoltageFailure, gStatus.Sensors.Struct.ReferenceVoltage != HAL_OK);
  CHECK_STATUS(iserror, CheckSensorPwrVoltageFailure, gStatus.Sensors.Struct.PowerVoltage != HAL_OK);
  CHECK_STATUS(iserror, CheckSensorLambdaFailure, gStatus.Sensors.Struct.Lambda != HAL_OK);
  CHECK_STATUS(iserror, CheckOutputDriverFailure, gStatus.OutputStatus != HAL_OK);

  CHECK_STATUS(ignored, CheckCanInitFailure, gStatus.CanInitStatus != HAL_OK);
  CHECK_STATUS(ignored, CheckCanTestFailure, gStatus.CanTestStatus != HAL_OK);
  CHECK_STATUS(ignored, CheckKlineProtocolFailure, gStatus.KlineProtocolStatus != HAL_OK);
  CHECK_STATUS(ignored, CheckKlineLoopbackFailure, gStatus.KlineLoopbackStatus != HAL_OK);

#if defined(INJECTORS_ON_INJ_CH1_ONLY) && INJECTORS_ON_INJ_CH1_ONLY == 1
  if(gParameters.InjectorChannel == InjectorChannel1) {
#endif
    CHECK_STATUS(iserror, CheckInjector4OpenCircuit, gStatus.OutputDiagnostic.Injectors.Diagnostic.Data.InjCy4 == OutputDiagOpenCircuit);
    CHECK_STATUS(iserror, CheckInjector3OpenCircuit, gStatus.OutputDiagnostic.Injectors.Diagnostic.Data.InjCy3 == OutputDiagOpenCircuit);
    CHECK_STATUS(iserror, CheckInjector2OpenCircuit, gStatus.OutputDiagnostic.Injectors.Diagnostic.Data.InjCy2 == OutputDiagOpenCircuit);
    CHECK_STATUS(iserror, CheckInjector1OpenCircuit, gStatus.OutputDiagnostic.Injectors.Diagnostic.Data.InjCy1 == OutputDiagOpenCircuit);
#if defined(INJECTORS_ON_INJ_CH1_ONLY) && INJECTORS_ON_INJ_CH1_ONLY == 1
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
  CHECK_STATUS(ignored, CheckCheckEngineShortToBatOrOverheat, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.CheckEngine == OutputDiagShortToBatOrOvertemp);
  //CHECK_STATUS(iserror, CheckCheckEngineShortToGND, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.CheckEngine == OutputDiagShortToGnd);
  //CHECK_STATUS(iserror, CheckSpeedMeterOpenCirtuit, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.Speedmeeter == OutputDiagOpenCircuit);
  CHECK_STATUS(ignored, CheckSpeedMeterShortToBatOrOverheat, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.Speedmeeter == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(ignored, CheckSpeedMeterShortToGND, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.Speedmeeter == OutputDiagShortToGnd);
  //CHECK_STATUS(iserror, CheckTachometerOpenCirtuit, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.Tachometer == OutputDiagOpenCircuit);
  CHECK_STATUS(ignored, CheckTachometerShortToBatOrOverheat, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.Tachometer == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(ignored, CheckTachometerShortToGND, gStatus.OutputDiagnostic.Outs1.Diagnostic.Data.Tachometer == OutputDiagShortToGnd);
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
  CHECK_STATUS(ignored, CheckStarterRelayShortToBatOrOverheat, gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.StarterRelay == OutputDiagShortToBatOrOvertemp);
  //CHECK_STATUS(iserror, CheckStarterRelayShortToGND, gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.StarterRelay == OutputDiagShortToGnd);
  CHECK_STATUS(iserror, CheckFanRelayOpenCirtuit, gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.FanRelay == OutputDiagOpenCircuit);
  CHECK_STATUS(iserror, CheckFanRelayShortToBatOrOverheat, gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.FanRelay == OutputDiagShortToBatOrOvertemp);
  CHECK_STATUS(iserror, CheckFanRelayShortToGND, gStatus.OutputDiagnostic.Outs2.Diagnostic.Data.FanRelay == OutputDiagShortToGnd);
  CHECK_STATUS(iserror, CheckOutputs2CommunicationFailure, gStatus.OutputDiagnostic.Outs2.Availability != HAL_OK);

  if(gEcuParams.useIdleValve) {
    CHECK_STATUS(iserror, CheckIdleValveFailure, gStatus.OutputDiagnostic.IdleValvePosition.Status != HAL_OK);
    CHECK_STATUS(iserror, CheckIdleValveDriverFailure, gStatus.IdleValvePosition != HAL_OK);
  }

  if(gEcuParams.useEtc) {
    CHECK_STATUS(iserror, CheckEtcCommError, gStatus.Etc.Comm != HAL_OK);
    CHECK_STATUS(iserror, CheckEtcPedal1, gStatus.Etc.Pedal1 != HAL_OK);
    CHECK_STATUS(iserror, CheckEtcPedal2, gStatus.Etc.Pedal2 != HAL_OK);
    CHECK_STATUS(iserror, CheckEtcPedalMismatch, gStatus.Etc.PedalMismatch != HAL_OK);
    CHECK_STATUS(iserror, CheckEtcTps1, gStatus.Etc.Tps1 != HAL_OK);
    CHECK_STATUS(iserror, CheckEtcTps2, gStatus.Etc.Tps2 != HAL_OK);
    CHECK_STATUS(iserror, CheckEtcTpsMismatch, gStatus.Etc.TpsMismatch != HAL_OK);
    CHECK_STATUS(iserror, CheckEtcAdcError, gStatus.Etc.AdcErrorFlag != HAL_OK);
    CHECK_STATUS(iserror, CheckEtcInternalComm, gStatus.Etc.EtcCommErrorFlag != HAL_OK);
    CHECK_STATUS(iserror, CheckEtcMotorAvailability, gStatus.Etc.MotorAvailability != HAL_OK);
    CHECK_STATUS(iserror, CheckEtcMotorOpenLoad, gStatus.Etc.Motor.OpenLoad != HAL_OK);
    CHECK_STATUS(iserror, CheckEtcMotorShortToGND, gStatus.Etc.Motor.ShortToGND != HAL_OK);
    CHECK_STATUS(iserror, CheckEtcMotorShortToSupply, gStatus.Etc.Motor.ShortToSupply != HAL_OK);
    CHECK_STATUS(iserror, CheckEtcMotorTemperature, gStatus.Etc.Motor.Temperature != HAL_OK);
    CHECK_STATUS(iserror, CheckEtcMotorSupplyFailure, gStatus.Etc.Motor.SupplyFailure != HAL_OK);
    CHECK_STATUS(iserror, CheckEtcMotorErrorFlag, gStatus.Etc.Motor.ErrorFlag != HAL_OK);
    CHECK_STATUS(iserror, CheckEtcMotorAlwaysHigh, gStatus.Etc.Motor.AlwaysHigh != HAL_OK);
  }

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

    CHECK_STATUS(ignored, CheckEngineLeanMixture, gStatus.LeanMixture.is_error && gStatus.LeanMixture.error_time > 5000);
    CHECK_STATUS(ignored, CheckEngineRichMixture, gStatus.RichMixture.is_error && gStatus.RichMixture.error_time > 5000);
    CHECK_STATUS(ignored, CheckEngineLeanIdleMixture, gStatus.LeanIdleMixture.is_error && gStatus.LeanIdleMixture.error_time > 10000);
    CHECK_STATUS(ignored, CheckEngineRichIdleMixture, gStatus.RichIdleMixture.is_error && gStatus.RichIdleMixture.error_time > 10000);

    CHECK_STATUS(iserror, CheckEngineLeanMixture, gStatus.LeanMixture.is_error && gStatus.LeanMixture.error_time > 10000);
    CHECK_STATUS(iserror, CheckEngineRichMixture, gStatus.RichMixture.is_error && gStatus.RichMixture.error_time > 10000);
    CHECK_STATUS(iserror, CheckEngineLeanIdleMixture, gStatus.LeanIdleMixture.is_error && gStatus.LeanIdleMixture.error_time > 30000);
    CHECK_STATUS(iserror, CheckEngineRichIdleMixture, gStatus.RichIdleMixture.is_error && gStatus.RichIdleMixture.error_time > 30000);
  }
  if(gEcuParams.useKnockSensor) {
    CHECK_STATUS(iserror, CheckKnockDetonationFound, (gStatus.Knock.GeneralStatus & KnockStatusStrongDedonation) > 0);
    CHECK_STATUS(iserror, CheckKnockLowNoiseLevel, (gStatus.Knock.GeneralStatus & KnockStatusLowNoise) > 0);

    for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
      CHECK_STATUS(iserror, (CheckKnockDetonationCy1 + i), (gStatus.Knock.CylinderStatus[i] & KnockStatusStrongDedonation) > 0);
    }
  }
  if(gEcuParams.phasedMode == PhasedModeWithSensor) {
    CHECK_STATUS(iserror, CheckTspsDesynchronized, (gStatus.Tsps.sync_error && gStatus.Tsps.sync_error_time > 100) || (gStatus.Tsps.critical_error));
  }
  CHECK_STATUS(iserror, CheckSensorMapTpsMismatch, gStatus.MapTpsRelation.is_error && gStatus.MapTpsRelation.error_time > 5000);

  CHECK_STATUS(iserror, CheckNoOilPressure, gStatus.OilPressure.is_error);
  CHECK_STATUS(iserror, CheckNoBatteryCharge, gStatus.BatteryCharge.is_error);

  gDiagErrors.Bits.csps_error = gStatus.Sensors.Struct.Csps != HAL_OK;
  gDiagErrors.Bits.not_used1 = 0;
  gDiagErrors.Bits.eeprom_error = gStatus.Flash.Byte != HAL_OK;
  gDiagErrors.Bits.lambda_heater = gStatus.O2Diagnostic.DIAHGD > 0;
  gDiagErrors.Bits.tsps_error = gStatus.Sensors.Struct.Tsps != HAL_OK;
  gDiagErrors.Bits.reset_error = 0;
  gDiagErrors.Bits.ram_error = gStatus.Bkpsram.Byte != HAL_OK;
  gDiagErrors.Bits.flash_error = gStatus.Flash.Byte != HAL_OK;

  gDiagErrors.Bits.low_voltage = gSharedParameters.PowerVoltage < 7.0f;
  gDiagErrors.Bits.not_used2 = 0;
  gDiagErrors.Bits.not_used3 = 0;
  gDiagErrors.Bits.engine_temp_low = gStatus.Sensors.Struct.EngineTemp != HAL_OK;
  gDiagErrors.Bits.lambda_low = gStatus.Sensors.Struct.Lambda != HAL_OK;
  gDiagErrors.Bits.tps_low = gStatus.Sensors.Struct.Tps != HAL_OK;
  gDiagErrors.Bits.maf_low = gStatus.Sensors.Struct.Map != HAL_OK;
  gDiagErrors.Bits.low_noise = (gStatus.Knock.GeneralStatus & KnockStatusLowNoise) > 0;

  gDiagErrors.Bits.high_voltage = gSharedParameters.PowerVoltage > 16.0f;
  gDiagErrors.Bits.not_used4 = 0;
  gDiagErrors.Bits.not_used5 = 0;
  gDiagErrors.Bits.engine_temp_high = gStatus.Sensors.Struct.EngineTemp != HAL_OK;
  gDiagErrors.Bits.lambda_high = gStatus.Sensors.Struct.Lambda != HAL_OK;
  gDiagErrors.Bits.tps_high = gStatus.Sensors.Struct.Tps != HAL_OK;
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
    if(HAL_DelayDiff(hal_now, hal_error_last) < 10000) {
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
          if(addr && addr < sizeof(gParameters) / sizeof(uDword)) {
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
    flashstatus = config_save_all(&gEcuParams, &gEcuCorrections, gEcuTable, TABLE_SETUPS_MAX);
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
      memset(&gEcuTempCorrections, 0, sizeof(gEcuTempCorrections));
    }
    prev_conversion = now;
  }
  if(DelayDiff(now, prev_conversion) > 500000)
  {
    prev_conversion = now;
    config_transform_progress_to_corrections(&gEcuCorrections, &gEcuCorrectionsProgress);
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
  uint8_t rotates = csps_isrotates();
  uint8_t is_idle_valve_moving = out_is_idle_valve_moving();

  switch(stage) {
    case 0:
      if(rotates) {
        gParameters.EtcMotorFullCloseFlag = 1;
        gParameters.EtcMotorActiveFlag = 1;
      } else {
        gParameters.EtcMotorFullCloseFlag = 0;
        gParameters.EtcMotorActiveFlag = 0;
      }

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
      gBackupSaveRes = 0;
      gBackupSaveReq = 1;
      stage++;
      break;
    case 6:
      if(gBackupSaveRes && !gBackupSaveReq) {
        stage++;
      }
      break;
    case 7:
      Mem.lock = 0;
      stage++;
      break;
    case 8:
      if(!rotates) {
        gParameters.EtcMotorFullCloseFlag = 0;
        gParameters.EtcMotorActiveFlag = 0;
        retval = 1;
        stage = 0;
      }
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
  int32_t integrator_time;
  sCspsData data = csps_data();
  float rpm = csps_getrpm(data);
  sMathInterpolateInput ipRpm;

  O2_SetLambdaForceEnabled(gForceParameters.LambdaForceEnabled);

  if(gEcuParams.useKnockSensor) {
    ipRpm =  math_interpolate_input(rpm, table->rotates, table->rotates_count);
    knock_frequency = roundf(math_interpolate_1d(ipRpm, table->knock_filter_frequency));
    knock_gain = roundf(math_interpolate_1d(ipRpm, table->knock_gain));
    integrator_time = roundf(math_interpolate_1d(ipRpm, table->knock_integrator_time));

    Knock_SetBandpassFilterFrequency(knock_frequency);
    Knock_SetGainValue(knock_gain);
    Knock_SetIntegratorTimeConstant(integrator_time);
  }

  csps_tsps_enable(gEcuParams.phasedMode == PhasedModeWithSensor);

  sens_configure_map(gEcuParams.map_pressure_gain, gEcuParams.map_pressure_offset);
  sens_configure_tps(gEcuParams.tps_voltage_low, gEcuParams.tps_voltage_high);
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
  uint32_t starter_time = 0;

  starter_state = out_get_starter(&starter_time);

  start_allowed = gEcuImmoStartAllowed && gEcuIgnStartAllowed;

  if(start_allowed && !running) {
    starter_state = GPIO_PIN_SET;
  } else if(starter_time > 150000) {
    starter_state = GPIO_PIN_RESET;
  }

  gParameters.StartAllowed = start_allowed;

  out_set_starter(starter_state);
}

static void ecu_can_init(void)
{
  const uint16_t filter_ids[] = { 0x110, 0x010 };
  const uint16_t filter_masks[] = { 0x7F0, 0x7F0 };
  gStatus.CanInitStatus = can_start(filter_ids, filter_masks, ITEMSOF(filter_ids));
  if(gStatus.CanInitStatus == HAL_OK) {
    gCanTestStarted = 1;
  }

  can_message_register_msg(&g_can_message_id010_ETC);
  can_message_register_msg(&g_can_message_id011_ETC);
  can_message_register_msg(&g_can_message_id012_ETC);
  can_message_register_msg(&g_can_message_id013_ETC);
  can_message_register_msg(&g_can_message_id018_ETC_ECU);
  can_message_register_msg(&g_can_message_id019_ETC_ECU);
  can_message_register_msg(&g_can_message_id01A_ETC_ECU);
  can_message_register_msg(&g_can_message_id01B_ETC_ECU);
  can_message_register_msg(&g_can_message_id01C_ETC_ECU);

  can_message_register_msg(&g_can_message_id110_LOG_ECU);
}

static void ecu_kline_init(void)
{
  kline_start();
}

static void ecu_can_mid_process(void)
{
  if(gStatus.CanInitStatus == HAL_OK) {
    if(!gCanTestStarted) {
      can_loop();
    }
  }
}

static void ecu_can_slow_process(void)
{
  if(gStatus.CanInitStatus == HAL_OK) {
    if(!gCanTestStarted) {
      can_signals_update(&gParameters);
    }
  }
}

static void ecu_can_loop(void)
{
  static sCanRawMessage message = {0};
  uint32_t now = Delay_Tick;
  int8_t status;
  if(gStatus.CanInitStatus == HAL_OK) {
    if(gCanTestStarted) {
      //status = can_test();
      status = 1;
      if(status != 0) {
        gCanTestStarted = 0;
        gStatus.CanTestStatus = status > 0 ? HAL_OK : HAL_ERROR;
      }
    } else {
      status = can_receive(&message);
      if(status > 0) {
        ecu_can_process_message(&message);
      }
    }
  }

  if(gEcuParams.useEtc) {
    if(gStatus.Etc.Comm == HAL_OK) {
      now = Delay_Tick;
      if(DelayDiff(now, gEtcCommLast) > 500000) {
        gStatus.Etc.Comm = HAL_ERROR;
      }
    }
  } else {
    gEtcCommLast = now;
    gStatus.Etc.Comm = HAL_OK;
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
            string = "AutoECU v2.2";
          } else if(rx_message.data[1] == 0x92) { //systemSupplierECUHardwareNumber
            string = "AutoECU v2.2";
          } else if(rx_message.data[1] == 0x94) { //systemSupplierECUSoftwareNumber
            string = "AutoECU v2.2";
          } else if(rx_message.data[1] == 0x97) { //systemNameOrEngineType
            string = "AutoECU v2.2";
          } else if(rx_message.data[1] == 0x98) { //repairShopCode
            string = "AutoECU v2.2";
          } else if(rx_message.data[1] == 0x99) { //ProgrammingDate
            string = "09.2023";
          } else if(rx_message.data[1] == 0x9A) { //vehicleManufacturerECUIdentifier
            string = "AutoECU v2.2";
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
            tx_message.data[tx_message.length++] = CLAMP(gSharedParameters.EngineTemp, -40, 150) + 40; //Температура охлаждающей жидкости
            tx_message.data[tx_message.length++] = CLAMP(gSharedParameters.FuelRatio, 7.5f, 21.0f) / 14.7f * 256 - 128; //Соотношение воздух/топливо
            tx_message.data[tx_message.length++] = CLAMP(gSharedParameters.ThrottlePosition, 0, 100); //Положение дроссельной заслонки
            tx_message.data[tx_message.length++] = gSharedParameters.RPM / 40.0f; //Скорость вращения двигателя
            tx_message.data[tx_message.length++] = gSharedParameters.WishIdleRPM / 10.0f; //Скорость вращения двигателя на холостом ходу
            tx_message.data[tx_message.length++] = CLAMP(gSharedParameters.WishIdleValvePosition, 0, 255); //Желаемое положение регулятора холостого хода
            tx_message.data[tx_message.length++] = CLAMP(gSharedParameters.IdleValvePosition, 0, 255); //Текущее положение регулятора холостого хода
            tx_message.data[tx_message.length++] = gSharedParameters.LongTermCorrection + gSharedParameters.ShortTermCorrection * 256.0f + 128; //Коэффициент коррекции времени впрыска
            tx_message.data[tx_message.length++] = (int8_t)(gSharedParameters.IgnitionAdvance * 2.0f); //Угол опережения зажигания
            tx_message.data[tx_message.length++] = gSharedParameters.Speed; //Скорость автомобиля
            tx_message.data[tx_message.length++] = (CLAMP(gSharedParameters.PowerVoltage, 5.5f, 17.5f) - 5.2f) * 20.0f; //Напряжение бортсети
            tx_message.data[tx_message.length++] = gSharedParameters.WishIdleRPM / 10.0f; //Желаемые обороты холостого хода
            tx_message.data[tx_message.length++] = CLAMP(gSharedParameters.AdcLambdaUA, 0.0f, 4.95f) / 5.0f * 256.0f; //Напряжение на датчике кислорода
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
            tx_message.data[tx_message.length++] = CLAMP(gSharedParameters.EngineTemp, -40, 150) + 40; //Температура охлаждающей жидкости
            tx_message.data[tx_message.length++] = CLAMP(gSharedParameters.FuelRatio, 7.5f, 21.0f) / 14.7f * 256 - 128; //Соотношение воздух/топливо
            tx_message.data[tx_message.length++] = CLAMP(gSharedParameters.ThrottlePosition, 0, 100); //Положение дроссельной заслонки
            tx_message.data[tx_message.length++] = gSharedParameters.RPM / 40.0f; //Скорость вращения двигателя
            tx_message.data[tx_message.length++] = gSharedParameters.WishIdleRPM / 10.0f; //Скорость вращения двигателя на холостом ходу
            tx_message.data[tx_message.length++] = CLAMP(gSharedParameters.IdleValvePosition, 0, 255); //Текущее положение регулятора холостого хода
            tx_message.data[tx_message.length++] = (int8_t)(gSharedParameters.IgnitionAdvance * 2.0f); //Угол опережения зажигания
            tx_message.data[tx_message.length++] = gSharedParameters.Speed; //Скорость автомобиля
            tx_message.data[tx_message.length++] = (CLAMP(gSharedParameters.PowerVoltage, 5.5f, 17.5f) - 5.2f) * 20.0f; //Напряжение бортсети
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
            tx_message.data[tx_message.length++] = (CLAMP(gSharedParameters.KnockSensor, 0.0f, 4.95f) / 5.0f * 256);
            tx_message.data[tx_message.length++] = (CLAMP(gSharedParameters.AdcEngineTemp, 0.0f, 4.95f) / 5.0f * 256);
            tx_message.data[tx_message.length++] = (CLAMP(gSharedParameters.AdcManifoldAirPressure, 0.0f, 4.95f) / 5.0f * 256);
            tx_message.data[tx_message.length++] = (CLAMP(gSharedParameters.PowerVoltage, 0.0f, 4.95f) / 5.0f * 256);
            tx_message.data[tx_message.length++] = (CLAMP(gSharedParameters.AdcLambdaUA, 0.0f, 4.95f) / 5.0f * 256);
            tx_message.data[tx_message.length++] = (CLAMP(gSharedParameters.AdcThrottlePosition, 0.0f, 4.95f) / 5.0f * 256);
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
  float rpm = csps_getrpm(csps_data());

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

      if(DelayDiff(now, gStatus.OilPressure.run_time) > 5000000) {
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
          if(rpm > 750) {
            if(DelayDiff(now, pressure_last) > 100000) {
              prev = pressure;
              pressure_last = now;
            }
          } else {
            if(DelayDiff(now, pressure_last) > 1000000) {
              prev = pressure;
              pressure_last = now;
            }
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

#ifdef SIMULATION
  GPIO_PinState ignition = GPIO_PIN_SET;
#else
  GPIO_PinState ignition = sens_get_ign(NULL);
#endif

  if(is_running && ignition == GPIO_PIN_SET) {
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

static void ecu_etc_test_process(void)
{
  uint32_t now = Delay_Tick;
  uint32_t diff;
  float throttle_position;

  if(gEtcTest.StartedTime) {
    if(!now)
      now++;
    diff = DelayDiff(now, gEtcTest.StartedTime);

    if(gEtcTest.Stage == 0) {
      if(diff < gEtcTest.StartDelay) {
        gEtcTest.CurrentPosition = gEtcTest.StartPosition;
        gParameters.WishThrottleTargetPosition = gEtcTest.CurrentPosition;
      } else {
        gEtcTest.Stage = 1;
        gEtcTest.StartedTime = now;
      }
    } else if(gEtcTest.Stage == 1) {
      throttle_position = (gEtcTest.FinalPosition - gEtcTest.StartPosition) * ((float)diff / (float)gEtcTest.MoveTime);
      throttle_position += gEtcTest.StartPosition;
      gEtcTest.CurrentPosition = CLAMP(throttle_position, 0.0f, 100.0f);
      gParameters.WishThrottleTargetPosition = gEtcTest.CurrentPosition;

      if(diff >= gEtcTest.MoveTime && ((gEtcTest.FinalPosition > gEtcTest.StartPosition && throttle_position >= gEtcTest.FinalPosition) ||
          (gEtcTest.FinalPosition <= gEtcTest.StartPosition && throttle_position <= gEtcTest.FinalPosition))) {
        gEtcTest.Stage = 2;
        gEtcTest.StartedTime = now;
      }
    } else if(gEtcTest.Stage == 2) {
      if(diff < gEtcTest.FinalDelay) {
        gEtcTest.CurrentPosition = gEtcTest.FinalPosition;
        gParameters.WishThrottleTargetPosition = gEtcTest.CurrentPosition;
      } else {
        gEtcTest.Stage = 0;
        gEtcTest.StartedTime = 0;
      }
    } else {
      gEtcTest.Stage = 0;
      gEtcTest.StartedTime = 0;
    }
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

  memset((void *)&gIITest, 0, sizeof(gIITest));
  memset((void *)&gEtcTest, 0, sizeof(gEtcTest));

#if defined(LEARN_ACCEPT_CYCLES_BUFFER_SIZE) && LEARN_ACCEPT_CYCLES_BUFFER_SIZE > 0
  memset(gLearnParamsBuffer, 0, sizeof(gLearnParamsBuffer));
  for(int i = 0; i < LEARN_ACCEPT_CYCLES_BUFFER_SIZE; i++) {
    gLearnParamsPtrs[i] = &gLearnParamsBuffer[i];
  }
#endif /* LEARN_ACCEPT_CYCLES_BUFFER_SIZE */

  csps_register_acceleration_callback(ecu_csps_acceleration_callback);

#ifdef SIMULATION
  gEcuIgnStartAllowed = 1;
#endif /* SIMULATION */

  gEcuInitialized = 1;


  gParameters.EtcStandaloneFlag = 0;
  gParameters.EtcMotorActiveFlag = 1;

}

void ecu_irq_fast_loop(void)
{
  if(!gEcuInitialized)
    return;

  ecu_process();
  ecu_phase_detect_process();
}

void ecu_irq_mid_loop(void)
{
  if(!gEcuInitialized)
    return;

  ecu_can_mid_process();
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
  ecu_can_slow_process();

  ecu_etc_test_process();

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
      PK_DragUpdateResponse.Data.Ignition = gSharedParameters.IgnitionAdvance;
      PK_DragUpdateResponse.Data.CycleAirFlow = gSharedParameters.CyclicAirFlow;
      PK_DragUpdateResponse.Data.MassAirFlow = gSharedParameters.MassAirFlow;
      PK_DragUpdateResponse.Data.Throttle = gSharedParameters.ThrottlePosition;
      PK_DragUpdateResponse.Data.Speed = gSharedParameters.Speed;
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
        PK_DragPointResponse.Point.Speed = 0;
        PK_DragPointResponse.Point.Mixture = 0;
        PK_DragPointResponse.Point.Acceleration = 0;
        PK_DragPointResponse.Point.Ignition = 0;
        PK_DragPointResponse.Point.Time = 0;
        PK_DragPointResponse.Point.CycleAirFlow = 0;
        PK_DragPointResponse.Point.MassAirFlow = 0;
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
      gEtcErrorReset = 1;
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

    case PK_EtcTestRequestID :
      PK_Copy(&PK_EtcTestRequest, msgBuf);

      gEtcTest.Stage = 0;
      gEtcTest.StartedTime = 0;

      gEtcTest.StartPosition = PK_EtcTestRequest.StartPosition;
      gEtcTest.StartDelay = PK_EtcTestRequest.StartDelay * 1000;
      gEtcTest.FinalPosition = PK_EtcTestRequest.FinalPosition;
      gEtcTest.MoveTime = PK_EtcTestRequest.MoveTime * 1000;
      gEtcTest.FinalDelay = PK_EtcTestRequest.FinalDelay * 1000;

      if(gEtcTest.MoveTime > 0 && gEtcTest.StartDelay > 0) {
        gEtcTest.StartedTime = Delay_Tick;
        if(gEtcTest.StartedTime == 0)
          gEtcTest.StartedTime++;
      }

      PK_EtcTestResponse.ErrorCode = 0;
      PK_SendCommand(xChaSrc, &PK_EtcTestResponse, sizeof(PK_EtcTestResponse));
      break;

    default:
      break;
  }
}

static void can_etc_signals_send(const sParameters *parameters)
{
  can_signal_message_clear(&g_can_message_id080_ECU_ETC);
  can_signal_append_uint(&g_can_message_id080_ECU_ETC, &g_can_signal_id080_ECU_ETC_StandaloneMode, parameters->EtcStandaloneFlag);
  can_signal_append_uint(&g_can_message_id080_ECU_ETC, &g_can_signal_id080_ECU_ETC_MotorActive, parameters->EtcMotorActiveFlag);
  can_signal_append_float(&g_can_message_id080_ECU_ETC, &g_can_signal_id080_ECU_ETC_TargetPosition, parameters->WishThrottleTargetPosition);
  can_signal_append_uint(&g_can_message_id080_ECU_ETC, &g_can_signal_id080_ECU_ETC_MotorErrorReset, gEtcErrorReset); gEtcErrorReset = 0;
  can_signal_append_uint(&g_can_message_id080_ECU_ETC, &g_can_signal_id080_ECU_ETC_MotorFullCloseRequest, parameters->EtcMotorFullCloseFlag);
  can_signal_append_uint(&g_can_message_id080_ECU_ETC, &g_can_signal_id080_ECU_ETC_AdaptationRequest, 0);
  can_signal_append_uint(&g_can_message_id080_ECU_ETC, &g_can_signal_id080_ECU_ETC_Rsvd1, 0);
  can_signal_append_uint(&g_can_message_id080_ECU_ETC, &g_can_signal_id080_ECU_ETC_Rsvd2, 0);
  can_signal_append_uint(&g_can_message_id080_ECU_ETC, &g_can_signal_id080_ECU_ETC_OutCruizeR, parameters->EtcOutCruizeR);
  can_signal_append_uint(&g_can_message_id080_ECU_ETC, &g_can_signal_id080_ECU_ETC_OutCruizeG, parameters->EtcOutCruizeG);
  can_signal_append_uint(&g_can_message_id080_ECU_ETC, &g_can_signal_id080_ECU_ETC_OutRsvd3, parameters->EtcOutRsvd3);
  can_signal_append_uint(&g_can_message_id080_ECU_ETC, &g_can_signal_id080_ECU_ETC_OutRsvd4, parameters->EtcOutRsvd4);

  can_message_send(&g_can_message_id080_ECU_ETC);
}

static int8_t ecu_can_process_message(const sCanRawMessage *message)
{
  sCanRawMessage transmit = {0};
  uint32_t now = Delay_Tick;
  int8_t status = 0;
  uint32_t value = 0;

  sCanMessage * can_msg = can_message_get_msg(message);

  if(can_msg != NULL) {
    if(gEcuParams.useEtc) {
      switch(can_msg->Id) {
        case 0x010:
          can_signal_get_float(&g_can_message_id010_ETC, &g_can_signal_id010_ETC_Tps1, &gParameters.AdcEtcTps1);
          can_signal_get_float(&g_can_message_id010_ETC, &g_can_signal_id010_ETC_Tps2, &gParameters.AdcEtcTps2);
          can_signal_get_float(&g_can_message_id010_ETC, &g_can_signal_id010_ETC_Pedal1, &gParameters.AdcEtcPedal1);
          can_signal_get_float(&g_can_message_id010_ETC, &g_can_signal_id010_ETC_Pedal2, &gParameters.AdcEtcPedal2);
          break;
        case 0x011:
          can_signal_get_float(&g_can_message_id011_ETC, &g_can_signal_id011_ETC_Rsvd5, &gParameters.AdcEtcRsvd5);
          can_signal_get_float(&g_can_message_id011_ETC, &g_can_signal_id011_ETC_Rsvd6, &gParameters.AdcEtcRsvd6);
          can_signal_get_float(&g_can_message_id011_ETC, &g_can_signal_id011_ETC_ReferenceVoltage, &gParameters.AdcEtcReferenceVoltage);
          can_signal_get_float(&g_can_message_id011_ETC, &g_can_signal_id011_ETC_PowerVoltage, &gParameters.AdcEtcPowerVoltage);
          break;
        case 0x012:
          can_signal_get_float(&g_can_message_id012_ETC, &g_can_signal_id012_ETC_ThrottlePosition, &gLocalParams.EtcThrottlePosition);
          can_signal_get_float(&g_can_message_id012_ETC, &g_can_signal_id012_ETC_TargetPosition, &gLocalParams.EtcThrottleTargetPosition);
          can_signal_get_float(&g_can_message_id012_ETC, &g_can_signal_id012_ETC_DefaultPosition, &gLocalParams.EtcThrottleDefaultPosition);
          can_signal_get_float(&g_can_message_id012_ETC, &g_can_signal_id012_ETC_PedalPosition, &gLocalParams.EtcPedalPosition);
          can_signal_get_uint(&g_can_message_id012_ETC, &g_can_signal_id012_ETC_Tps1ErrorFlag, &value); gStatus.Etc.Tps1 = value ? HAL_ERROR : HAL_OK;
          can_signal_get_uint(&g_can_message_id012_ETC, &g_can_signal_id012_ETC_Tps2ErrorFlag, &value); gStatus.Etc.Tps2 = value ? HAL_ERROR : HAL_OK;
          can_signal_get_uint(&g_can_message_id012_ETC, &g_can_signal_id012_ETC_Pedal1ErrorFlag, &value); gStatus.Etc.Pedal1 = value ? HAL_ERROR : HAL_OK;
          can_signal_get_uint(&g_can_message_id012_ETC, &g_can_signal_id012_ETC_Pedal2ErrorFlag, &value); gStatus.Etc.Pedal2 = value ? HAL_ERROR : HAL_OK;
          can_signal_get_uint(&g_can_message_id012_ETC, &g_can_signal_id012_ETC_TpsMismatchFlag, &value); gStatus.Etc.TpsMismatch = value ? HAL_ERROR : HAL_OK;
          can_signal_get_uint(&g_can_message_id012_ETC, &g_can_signal_id012_ETC_PedalMismatchFlag, &value); gStatus.Etc.PedalMismatch = value ? HAL_ERROR : HAL_OK;
          can_signal_get_uint(&g_can_message_id012_ETC, &g_can_signal_id012_ETC_MotorErrorFlag, NULL);
          can_signal_get_uint(&g_can_message_id012_ETC, &g_can_signal_id012_ETC_StandaloneFlag, NULL);

          gEtcCommLast = now;
          gStatus.Etc.Comm = HAL_OK;
          break;
        case 0x013:
          can_signal_get_uint8(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_InCruizeStart, &gParameters.EtcInCruizeStart);
          can_signal_get_uint8(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_InCruizeStop, &gParameters.EtcInCruizeStop);
          can_signal_get_uint8(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_InBrake, &gParameters.EtcInBrake);
          can_signal_get_uint8(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_InRsvd4, &gParameters.EtcInRsvd4);
          can_signal_get_uint8(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_InRsvd5, &gParameters.EtcInRsvd5);
          can_signal_get_uint8(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_InRsvd6, &gParameters.EtcInRsvd6);
          can_signal_get_uint(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_OutCruizeG, NULL);
          can_signal_get_uint(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_OutCruizeR, NULL);
          can_signal_get_uint(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_OutRsvd3, NULL);
          can_signal_get_uint(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_OutRsvd4, NULL);
          can_signal_get_uint(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_OutCruizeGState, NULL);
          can_signal_get_uint(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_OutCruizeRState, NULL);
          can_signal_get_uint(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_OutRsvd3State, NULL);
          can_signal_get_uint(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_OutRsvd4State, NULL);
          can_signal_get_uint(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_OutputsAvailability, NULL);
          can_signal_get_uint(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_MotorErrorFlag, &value); gStatus.Etc.Motor.ErrorFlag = value ? HAL_ERROR : HAL_OK;
          can_signal_get_uint(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_MotorTemperature, &value); gStatus.Etc.Motor.Temperature = value ? HAL_ERROR : HAL_OK;
          can_signal_get_uint(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_MotorOpenLoad, &value); gStatus.Etc.Motor.OpenLoad = value ? HAL_ERROR : HAL_OK;
          can_signal_get_uint(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_MotorShortToGND, &value); gStatus.Etc.Motor.ShortToGND = value ? HAL_ERROR : HAL_OK;
          can_signal_get_uint(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_MotorShortToSupply, &value); gStatus.Etc.Motor.ShortToSupply = value ? HAL_ERROR : HAL_OK;
          can_signal_get_uint(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_MotorSupplyFailure, &value); gStatus.Etc.Motor.SupplyFailure = value ? HAL_ERROR : HAL_OK;
          can_signal_get_uint(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_MotorAlwaysHigh, &value); gStatus.Etc.Motor.AlwaysHigh = value ? HAL_OK : HAL_ERROR;
          can_signal_get_uint(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_MotorAvailability, &value); gStatus.Etc.MotorAvailability = value ? HAL_ERROR : HAL_OK;
          can_signal_get_uint(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_AdcErrorFlag, &value); gStatus.Etc.AdcErrorFlag = value ? HAL_ERROR : HAL_OK;
          can_signal_get_uint(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_CommErrorFlag, &value); gStatus.Etc.EtcCommErrorFlag = value ? HAL_ERROR : HAL_OK;
          can_signal_get_uint(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_CanInitFlag, NULL);
          can_signal_get_uint(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_CanTestFlag, NULL);
          can_signal_get_uint(&g_can_message_id013_ETC, &g_can_signal_id013_ETC_AdaptationCompletedFlag, NULL);
          break;
        case 0x018:
          can_signal_get_uint(&g_can_message_id018_ETC_ECU, &g_can_signal_id018_ETC_ECU_ConfigApplyIdAck, NULL);
          break;
        case 0x019:
          can_signal_get_uint(&g_can_message_id019_ETC_ECU, &g_can_signal_id019_ETC_ECU_Tps1Min, NULL);
          can_signal_get_uint(&g_can_message_id019_ETC_ECU, &g_can_signal_id019_ETC_ECU_Tps1Max, NULL);
          can_signal_get_uint(&g_can_message_id019_ETC_ECU, &g_can_signal_id019_ETC_ECU_Tps1Limit, NULL);
          can_signal_get_uint(&g_can_message_id019_ETC_ECU, &g_can_signal_id019_ETC_ECU_Rsvd, NULL);
          break;
        case 0x01A:
          can_signal_get_uint(&g_can_message_id01A_ETC_ECU, &g_can_signal_id01A_ETC_ECU_Tps2Min, NULL);
          can_signal_get_uint(&g_can_message_id01A_ETC_ECU, &g_can_signal_id01A_ETC_ECU_Tps2Max, NULL);
          can_signal_get_uint(&g_can_message_id01A_ETC_ECU, &g_can_signal_id01A_ETC_ECU_Tps2Limit, NULL);
          can_signal_get_uint(&g_can_message_id01A_ETC_ECU, &g_can_signal_id01A_ETC_ECU_Rsvd, NULL);
          break;
        case 0x01B:
          can_signal_get_uint(&g_can_message_id01B_ETC_ECU, &g_can_signal_id01B_ETC_ECU_Pedal1Min, NULL);
          can_signal_get_uint(&g_can_message_id01B_ETC_ECU, &g_can_signal_id01B_ETC_ECU_Pedal1Max, NULL);
          can_signal_get_uint(&g_can_message_id01B_ETC_ECU, &g_can_signal_id01B_ETC_ECU_Pedal2Min, NULL);
          can_signal_get_uint(&g_can_message_id01B_ETC_ECU, &g_can_signal_id01B_ETC_ECU_Pedal2Max, NULL);
          break;
        case 0x01C:
          can_signal_get_uint(&g_can_message_id01C_ETC_ECU, &g_can_signal_id01C_ETC_ECU_PidP, NULL);
          can_signal_get_uint(&g_can_message_id01C_ETC_ECU, &g_can_signal_id01C_ETC_ECU_PidI, NULL);
          can_signal_get_uint(&g_can_message_id01C_ETC_ECU, &g_can_signal_id01C_ETC_ECU_PidD, NULL);
          can_signal_get_uint(&g_can_message_id01C_ETC_ECU, &g_can_signal_id01C_ETC_ECU_TimPsc, NULL);
          break;
        case 0x110:
          can_signal_get_uint(&g_can_message_id110_LOG_ECU, &g_can_signal_id110_LOG_ECU_Unique, NULL);
          gLoggerCommLast = now;
          break;
        default:
          break;
      }
    }
  } else {
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
        if(message->data.dwords[0] < (sizeof(gEcuTable) / (sizeof(uint32_t)))) {
          transmit.id = message->id + 0x100;
          transmit.rtr = CAN_RTR_DATA;
          transmit.length = 8;
          transmit.data.dwords[0] = message->data.dwords[0];
          transmit.data.dwords[1] = ((uint32_t *)&gEcuTable[0])[message->data.dwords[0]];
        }
      }
    } else if(message->id == 0x104) { //Config memory request
      if(message->rtr == CAN_RTR_DATA && message->length == 4) {
        if(message->data.dwords[0] < sizeof(gEcuParams) / (sizeof(uint32_t))) {
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
  }

  return status;
}

void ecu_hardfault_handle(void)
{
  gEcuCriticalBackup.CheckBitmapRecorded[CheckHardFaultException >> 3] |= 1 << (CheckHardFaultException & 7);
  config_save_critical_backup(&gEcuCriticalBackup);
}

static void can_log_signals_send(const sParameters *parameters)
{
  can_signal_message_clear(&g_can_message_id020_ECU);
  can_signal_append_float(&g_can_message_id020_ECU, &g_can_signal_id020_ECU_AdcKnockVoltage, parameters->AdcKnockVoltage);
  can_signal_append_float(&g_can_message_id020_ECU, &g_can_signal_id020_ECU_AdcAirTemp, parameters->AdcAirTemp);
  can_signal_append_float(&g_can_message_id020_ECU, &g_can_signal_id020_ECU_AdcEngineTemp, parameters->AdcEngineTemp);
  can_signal_append_float(&g_can_message_id020_ECU, &g_can_signal_id020_ECU_AdcManifoldAirPressure, parameters->AdcManifoldAirPressure);
  can_signal_append_float(&g_can_message_id020_ECU, &g_can_signal_id020_ECU_AdcThrottlePosition, parameters->AdcThrottlePosition);
  can_signal_append_float(&g_can_message_id020_ECU, &g_can_signal_id020_ECU_AdcReferenceVoltage, parameters->AdcReferenceVoltage);
  can_signal_append_float(&g_can_message_id020_ECU, &g_can_signal_id020_ECU_AdcLambdaUR, parameters->AdcLambdaUR);
  can_signal_append_float(&g_can_message_id020_ECU, &g_can_signal_id020_ECU_AdcLambdaUA, parameters->AdcLambdaUA);

  can_signal_message_clear(&g_can_message_id021_ECU);
  can_signal_append_float(&g_can_message_id021_ECU, &g_can_signal_id021_ECU_KnockSensor, parameters->KnockSensor);
  can_signal_append_float(&g_can_message_id021_ECU, &g_can_signal_id021_ECU_KnockSensorFiltered, parameters->KnockSensorFiltered);
  can_signal_append_float(&g_can_message_id021_ECU, &g_can_signal_id021_ECU_KnockSensorDetonate, parameters->KnockSensorDetonate);
  can_signal_append_float(&g_can_message_id021_ECU, &g_can_signal_id021_ECU_KnockSaturation, parameters->KnockSaturation);
  can_signal_append_float(&g_can_message_id021_ECU, &g_can_signal_id021_ECU_KnockAdvance, parameters->KnockAdvance);
  can_signal_append_uint(&g_can_message_id021_ECU, &g_can_signal_id021_ECU_KnockCountCy1, parameters->KnockCountCy[0]);
  can_signal_append_uint(&g_can_message_id021_ECU, &g_can_signal_id021_ECU_KnockCountCy2, parameters->KnockCountCy[1]);
  can_signal_append_uint(&g_can_message_id021_ECU, &g_can_signal_id021_ECU_KnockCountCy3, parameters->KnockCountCy[2]);
  can_signal_append_uint(&g_can_message_id021_ECU, &g_can_signal_id021_ECU_KnockCountCy4, parameters->KnockCountCy[3]);
  can_signal_append_uint(&g_can_message_id021_ECU, &g_can_signal_id021_ECU_KnockCount, parameters->KnockCount);

  can_signal_message_clear(&g_can_message_id022_ECU);
  can_signal_append_float(&g_can_message_id022_ECU, &g_can_signal_id022_ECU_AirTemp, parameters->AirTemp);
  can_signal_append_float(&g_can_message_id022_ECU, &g_can_signal_id022_ECU_EngineTemp, parameters->EngineTemp);
  can_signal_append_float(&g_can_message_id022_ECU, &g_can_signal_id022_ECU_CalculatedAirTemp, parameters->CalculatedAirTemp);
  can_signal_append_float(&g_can_message_id022_ECU, &g_can_signal_id022_ECU_ManifoldAirPressure, parameters->ManifoldAirPressure);
  can_signal_append_float(&g_can_message_id022_ECU, &g_can_signal_id022_ECU_ThrottlePosition, parameters->ThrottlePosition);
  can_signal_append_float(&g_can_message_id022_ECU, &g_can_signal_id022_ECU_RPM, parameters->RPM);

  can_signal_message_clear(&g_can_message_id023_ECU);
  can_signal_append_float(&g_can_message_id023_ECU, &g_can_signal_id023_ECU_FuelRatio, parameters->FuelRatio);
  can_signal_append_float(&g_can_message_id023_ECU, &g_can_signal_id023_ECU_FuelRatioDiff, parameters->FuelRatioDiff);
  can_signal_append_float(&g_can_message_id023_ECU, &g_can_signal_id023_ECU_LambdaValue, parameters->LambdaValue);
  can_signal_append_float(&g_can_message_id023_ECU, &g_can_signal_id023_ECU_LambdaTemperature, parameters->LambdaTemperature);
  can_signal_append_float(&g_can_message_id023_ECU, &g_can_signal_id023_ECU_LambdaHeaterVoltage, parameters->LambdaHeaterVoltage);
  can_signal_append_float(&g_can_message_id023_ECU, &g_can_signal_id023_ECU_ShortTermCorrection, parameters->ShortTermCorrection);
  can_signal_append_float(&g_can_message_id023_ECU, &g_can_signal_id023_ECU_LongTermCorrection, parameters->LongTermCorrection);
  can_signal_append_float(&g_can_message_id023_ECU, &g_can_signal_id023_ECU_IdleCorrection, parameters->IdleCorrection);

  can_signal_message_clear(&g_can_message_id024_ECU);
  can_signal_append_float(&g_can_message_id024_ECU, &g_can_signal_id024_ECU_Speed, parameters->Speed);
  can_signal_append_float(&g_can_message_id024_ECU, &g_can_signal_id024_ECU_MassAirFlow, parameters->MassAirFlow);
  can_signal_append_float(&g_can_message_id024_ECU, &g_can_signal_id024_ECU_CyclicAirFlow, parameters->CyclicAirFlow);
  can_signal_append_float(&g_can_message_id024_ECU, &g_can_signal_id024_ECU_EffectiveVolume, parameters->EffectiveVolume);
  can_signal_append_float(&g_can_message_id024_ECU, &g_can_signal_id024_ECU_EngineLoad, parameters->EngineLoad);
  can_signal_append_float(&g_can_message_id024_ECU, &g_can_signal_id024_ECU_EstimatedPower, parameters->EstimatedPower);
  can_signal_append_float(&g_can_message_id024_ECU, &g_can_signal_id024_ECU_EstimatedTorque, parameters->EstimatedTorque);

  can_signal_message_clear(&g_can_message_id025_ECU);
  can_signal_append_float(&g_can_message_id025_ECU, &g_can_signal_id025_ECU_WishFuelRatio, parameters->WishFuelRatio);
  can_signal_append_float(&g_can_message_id025_ECU, &g_can_signal_id025_ECU_IdleValvePosition, parameters->IdleValvePosition);
  can_signal_append_float(&g_can_message_id025_ECU, &g_can_signal_id025_ECU_IdleRegThrRPM, parameters->IdleRegThrRPM);
  can_signal_append_float(&g_can_message_id025_ECU, &g_can_signal_id025_ECU_WishIdleRPM, parameters->WishIdleRPM);
  can_signal_append_float(&g_can_message_id025_ECU, &g_can_signal_id025_ECU_WishIdleMassAirFlow, parameters->WishIdleMassAirFlow);
  can_signal_append_float(&g_can_message_id025_ECU, &g_can_signal_id025_ECU_WishIdleValvePosition, parameters->WishIdleValvePosition);
  can_signal_append_float(&g_can_message_id025_ECU, &g_can_signal_id025_ECU_WishIdleIgnitionAdvance, parameters->WishIdleIgnitionAdvance);
  can_signal_append_float(&g_can_message_id025_ECU, &g_can_signal_id025_ECU_IdleSpeedShift, parameters->IdleSpeedShift);

  can_signal_message_clear(&g_can_message_id026_ECU);
  can_signal_append_float(&g_can_message_id026_ECU, &g_can_signal_id026_ECU_InjectionPhase, parameters->InjectionPhase);
  can_signal_append_float(&g_can_message_id026_ECU, &g_can_signal_id026_ECU_InjectionPhaseDuration, parameters->InjectionPhaseDuration);
  can_signal_append_float(&g_can_message_id026_ECU, &g_can_signal_id026_ECU_InjectionPulse, parameters->InjectionPulse);
  can_signal_append_float(&g_can_message_id026_ECU, &g_can_signal_id026_ECU_InjectionDutyCycle, parameters->InjectionDutyCycle);
  can_signal_append_float(&g_can_message_id026_ECU, &g_can_signal_id026_ECU_IgnitionPulse, parameters->IgnitionPulse);
  can_signal_append_float(&g_can_message_id026_ECU, &g_can_signal_id026_ECU_InjectionLag, parameters->InjectionLag);
  can_signal_append_float(&g_can_message_id026_ECU, &g_can_signal_id026_ECU_TspsRelativePosition, parameters->TspsRelativePosition);
  can_signal_append_float(&g_can_message_id026_ECU, &g_can_signal_id026_ECU_IgnitionAdvance, parameters->IgnitionAdvance);

  can_signal_message_clear(&g_can_message_id027_ECU);
  can_signal_append_float(&g_can_message_id027_ECU, &g_can_signal_id027_ECU_EnrichmentSyncAmount, parameters->EnrichmentSyncAmount);
  can_signal_append_float(&g_can_message_id027_ECU, &g_can_signal_id027_ECU_EnrichmentAsyncAmount, parameters->EnrichmentAsyncAmount);
  can_signal_append_float(&g_can_message_id027_ECU, &g_can_signal_id027_ECU_EnrichmentStartLoad, parameters->EnrichmentStartLoad);
  can_signal_append_float(&g_can_message_id027_ECU, &g_can_signal_id027_ECU_EnrichmentLoadDerivative, parameters->EnrichmentLoadDerivative);
  can_signal_append_float(&g_can_message_id027_ECU, &g_can_signal_id027_ECU_InjectionEnrichment, parameters->InjectionEnrichment);
  can_signal_append_float(&g_can_message_id027_ECU, &g_can_signal_id027_ECU_IdleWishToRpmRelation, parameters->IdleWishToRpmRelation);

  can_signal_message_clear(&g_can_message_id028_ECU);
  can_signal_append_float(&g_can_message_id028_ECU, &g_can_signal_id028_ECU_DrivenKilometers, parameters->DrivenKilometers);
  can_signal_append_float(&g_can_message_id028_ECU, &g_can_signal_id028_ECU_FuelConsumption, parameters->FuelConsumption);
  can_signal_append_float(&g_can_message_id028_ECU, &g_can_signal_id028_ECU_FuelConsumed, parameters->FuelConsumed);
  can_signal_append_float(&g_can_message_id028_ECU, &g_can_signal_id028_ECU_FuelHourly, parameters->FuelHourly);

  can_signal_message_clear(&g_can_message_id029_ECU);
  can_signal_append_uint(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_LambdaValid, parameters->LambdaValid > 0);
  can_signal_append_uint(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_OilSensor, parameters->OilSensor > 0);
  can_signal_append_uint(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_FanForceSwitch, parameters->FanForceSwitch > 0);
  can_signal_append_uint(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_HandbrakeSensor, parameters->HandbrakeSensor > 0);
  can_signal_append_uint(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_ChargeSensor, parameters->ChargeSensor > 0);
  can_signal_append_uint(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_ClutchSensor, parameters->ClutchSensor > 0);
  can_signal_append_uint(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_IgnSensor, parameters->IgnSensor > 0);
  can_signal_append_uint(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_FuelPumpRelay, parameters->FuelPumpRelay > 0);
  can_signal_append_uint(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_FanRelay, parameters->FanRelay > 0);
  can_signal_append_uint(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_CheckEngine, parameters->CheckEngine > 0);
  can_signal_append_uint(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_StarterRelay, parameters->StarterRelay > 0);
  can_signal_append_uint(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_FanSwitch, parameters->FanSwitch > 0);
  can_signal_append_uint(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_IgnOutput, parameters->IgnOutput > 0);
  can_signal_append_uint(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_StartAllowed, parameters->StartAllowed > 0);
  can_signal_append_uint(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_IsRunning, parameters->IsRunning > 0);
  can_signal_append_uint(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_IsCheckEngine, parameters->IsCheckEngine > 0);
  can_signal_append_uint(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_CylinderIgnitionBitmask, parameters->CylinderIgnitionBitmask);
  can_signal_append_uint(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_CylinderInjectionBitmask, parameters->CylinderInjectionBitmask);
  can_signal_append_float(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_PowerVoltage, parameters->PowerVoltage);
  can_signal_append_uint(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_IdleFlag, parameters->IdleFlag > 0);
  can_signal_append_uint(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_IdleCorrFlag, parameters->IdleCorrFlag > 0);
  can_signal_append_uint(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_IdleEconFlag, parameters->IdleEconFlag > 0);
  can_signal_append_uint(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_SwitchPosition, parameters->SwitchPosition);
  can_signal_append_uint(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_CurrentTable, parameters->CurrentTable);
  can_signal_append_uint(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_InjectorChannel, parameters->InjectorChannel);

  can_signal_message_clear(&g_can_message_id02A_ECU);
  can_signal_append_float(&g_can_message_id02A_ECU, &g_can_signal_id02A_ECU_EtcAdcTps1, parameters->AdcEtcTps1);
  can_signal_append_float(&g_can_message_id02A_ECU, &g_can_signal_id02A_ECU_EtcAdcTps2, parameters->AdcEtcTps2);
  can_signal_append_float(&g_can_message_id02A_ECU, &g_can_signal_id02A_ECU_EtcAdcPedal1, parameters->AdcEtcPedal1);
  can_signal_append_float(&g_can_message_id02A_ECU, &g_can_signal_id02A_ECU_EtcAdcPedal2, parameters->AdcEtcPedal2);
  can_signal_append_float(&g_can_message_id02A_ECU, &g_can_signal_id02A_ECU_EtcAdcRsvd5, parameters->AdcEtcRsvd5);
  can_signal_append_float(&g_can_message_id02A_ECU, &g_can_signal_id02A_ECU_EtcAdcRsvd6, parameters->AdcEtcRsvd6);
  can_signal_append_float(&g_can_message_id02A_ECU, &g_can_signal_id02A_ECU_EtcAdcReferenceVoltage, parameters->AdcEtcReferenceVoltage);
  can_signal_append_float(&g_can_message_id02A_ECU, &g_can_signal_id02A_ECU_EtcAdcPowerVoltage, parameters->AdcEtcPowerVoltage);

  can_signal_message_clear(&g_can_message_id02B_ECU);
  can_signal_append_float(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcThrottleTargetPosition, parameters->ThrottleTargetPosition);
  can_signal_append_float(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcThrottleDefaultPosition, parameters->ThrottleDefaultPosition);
  can_signal_append_float(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcWishThrottleTargetPosition, parameters->WishThrottleTargetPosition);
  can_signal_append_float(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcPedalPosition, parameters->PedalPosition);
  can_signal_append_uint(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcStandaloneFlag, parameters->EtcStandaloneFlag > 0);
  can_signal_append_uint(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcMotorActive, parameters->EtcMotorActiveFlag > 0);
  can_signal_append_uint(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcMotorFullCloseFlag, parameters->EtcMotorFullCloseFlag > 0);
  can_signal_append_uint(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcInCruizeStart, parameters->EtcInCruizeStart > 0);
  can_signal_append_uint(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcInCruizeStop, parameters->EtcInCruizeStop > 0);
  can_signal_append_uint(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcInBrake, parameters->EtcInBrake > 0);
  can_signal_append_uint(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcInRsvd4, parameters->EtcInRsvd4 > 0);
  can_signal_append_uint(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcInRsvd5, parameters->EtcInRsvd5 > 0);
  can_signal_append_uint(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcInRsvd6, parameters->EtcInRsvd6 > 0);
  can_signal_append_uint(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcOutCruizeG, parameters->EtcOutCruizeG > 0);
  can_signal_append_uint(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcOutCruizeR, parameters->EtcOutCruizeR > 0);
  can_signal_append_uint(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcOutRsvd3, parameters->EtcOutRsvd3 > 0);
  can_signal_append_uint(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcOutRsvd4, parameters->EtcOutRsvd4 > 0);

  can_message_send(&g_can_message_id020_ECU);
  can_message_send(&g_can_message_id021_ECU);
  can_message_send(&g_can_message_id022_ECU);
  can_message_send(&g_can_message_id023_ECU);
  can_message_send(&g_can_message_id024_ECU);
  can_message_send(&g_can_message_id025_ECU);
  can_message_send(&g_can_message_id026_ECU);
  can_message_send(&g_can_message_id027_ECU);
  can_message_send(&g_can_message_id028_ECU);
  can_message_send(&g_can_message_id029_ECU);
  can_message_send(&g_can_message_id02A_ECU);
  can_message_send(&g_can_message_id02B_ECU);
}

static void can_signals_update(const sParameters *parameters)
{
  static uint32_t signals_last = 0;
  static uint32_t etc_last = 0;
  uint32_t now = Delay_Tick;

  if (DelayDiff(now, signals_last) >= 10000) {
    signals_last = now;
    if (DelayDiff(now, gLoggerCommLast) < 3000000) {
      can_log_signals_send(parameters);
    }
  }

  if (DelayDiff(now, etc_last) >= 5000) {
    etc_last = now;
    if(gEcuParams.useEtc) {
      can_etc_signals_send(parameters);
    }
  }
}
