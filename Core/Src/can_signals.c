/*
 * CAN_SIGNALs.c
 *
 *  Created on: 6 нояб. 2023 г.
 *      Author: VHEMaster
 */

#include <string.h>
#include <math.h>
#include "main.h"
#include "defines.h"
#include "structs.h"
#include "can_signals.h"
#include "can.h"
#include "delay.h"

#define CAN_SIGNAL_BYTES_MAX  (8U)
#define CAN_SIGNAL_BITS_MAX   (CAN_SIGNAL_BYTES_MAX * 8U)

typedef enum {
  CAN_SIGNAL_TYPE_UNSIGNED,
  CAN_SIGNAL_TYPE_FLOAT,
}eCanSignalType;

typedef struct {
    eCanSignalType SignalType;
    float Gain;
    float Offset;
    uint8_t StartBit;
    uint8_t LengthBit;
    uint8_t MinMaxDefined;
    float MinValue;
    float MaxValue;
}sCanSignal;

typedef struct {
    uint32_t Id;
    uint8_t Length;
    uint8_t MessageBuffer[CAN_SIGNAL_BYTES_MAX];
}sCanMessage;

sCanMessage g_can_message_id020_ECU = { 0x020, 8 }; // ADC

sCanSignal g_can_signal_id020_ECU_AdcKnockVoltage = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.023529f,
    .Offset = 0,
    .StartBit = 0,
    .LengthBit = 8
};

sCanSignal g_can_signal_id020_ECU_AdcAirTemp = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.023529f,
    .Offset = 0,
    .StartBit = 8,
    .LengthBit = 8
};

sCanSignal g_can_signal_id020_ECU_AdcEngineTemp = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.023529f,
    .Offset = 0,
    .StartBit = 16,
    .LengthBit = 8
};

sCanSignal g_can_signal_id020_ECU_AdcManifoldAirPressure = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.023529f,
    .Offset = 0,
    .StartBit = 24,
    .LengthBit = 8
};

sCanSignal g_can_signal_id020_ECU_AdcThrottlePosition = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.023529f,
    .Offset = 0,
    .StartBit = 32,
    .LengthBit = 8
};

sCanSignal g_can_signal_id020_ECU_AdcReferenceVoltage = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.023529f,
    .Offset = 0,
    .StartBit = 40,
    .LengthBit = 8
};

sCanSignal g_can_signal_id020_ECU_AdcLambdaUR = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.023529f,
    .Offset = 0,
    .StartBit = 48,
    .LengthBit = 8
};

sCanSignal g_can_signal_id020_ECU_AdcLambdaUA = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.023529f,
    .Offset = 0,
    .StartBit = 56,
    .LengthBit = 8
};

sCanMessage g_can_message_id021_ECU = { 0x021, 8 }; // Knock

sCanSignal g_can_signal_id021_ECU_KnockSensor = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.023529f,
    .Offset = 0,
    .StartBit = 0,
    .LengthBit = 8
};

sCanSignal g_can_signal_id021_ECU_KnockSensorFiltered = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.023529f,
    .Offset = 0,
    .StartBit = 8,
    .LengthBit = 8
};

sCanSignal g_can_signal_id021_ECU_KnockSensorDetonate = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.023529f,
    .Offset = 0,
    .StartBit = 16,
    .LengthBit = 8
};

sCanSignal g_can_signal_id021_ECU_KnockSaturation = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.024414f,
    .Offset = 0,
    .StartBit = 24,
    .LengthBit = 12
};

sCanSignal g_can_signal_id021_ECU_KnockAdvance = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.5f,
    .Offset = 0,
    .StartBit = 36,
    .LengthBit = 4
};

sCanSignal g_can_signal_id021_ECU_KnockCountCy1 = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 40,
    .LengthBit = 4
};

sCanSignal g_can_signal_id021_ECU_KnockCountCy2 = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 44,
    .LengthBit = 4
};

sCanSignal g_can_signal_id021_ECU_KnockCountCy3 = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 48,
    .LengthBit = 4
};

sCanSignal g_can_signal_id021_ECU_KnockCountCy4 = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 52,
    .LengthBit = 4
};

sCanSignal g_can_signal_id021_ECU_KnockCount = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 56,
    .LengthBit = 8
};


sCanMessage g_can_message_id022_ECU = { 0x022, 8 }; // Parameters

sCanSignal g_can_signal_id022_ECU_AirTemp = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.2f,
    .Offset = -50,
    .StartBit = 0,
    .LengthBit = 10
};

sCanSignal g_can_signal_id022_ECU_EngineTemp = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.2f,
    .Offset = -50,
    .StartBit = 10,
    .LengthBit = 10
};

sCanSignal g_can_signal_id022_ECU_CalculatedAirTemp = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.2f,
    .Offset = -50,
    .StartBit = 20,
    .LengthBit = 10
};

sCanSignal g_can_signal_id022_ECU_ManifoldAirPressure = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 30,
    .Offset = 0,
    .StartBit = 30,
    .LengthBit = 12
};

sCanSignal g_can_signal_id022_ECU_ThrottlePosition = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.1f,
    .Offset = 0,
    .StartBit = 42,
    .LengthBit = 10
};

sCanSignal g_can_signal_id022_ECU_RPM = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 2.4414f,
    .Offset = 0,
    .StartBit = 52,
    .LengthBit = 12
};

sCanMessage g_can_message_id023_ECU = { 0x023, 8 }; // Lambda & Correction

sCanSignal g_can_signal_id023_ECU_FuelRatio = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.1f,
    .Offset = 0,
    .StartBit = 0,
    .LengthBit = 8
};

sCanSignal g_can_signal_id023_ECU_FuelRatioDiff = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.0078125f,
    .Offset = 0,
    .StartBit = 8,
    .LengthBit = 8
};

sCanSignal g_can_signal_id023_ECU_LambdaValue = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.0078125f,
    .Offset = 0,
    .StartBit = 16,
    .LengthBit = 8
};

sCanSignal g_can_signal_id023_ECU_LambdaTemperature = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 4,
    .Offset = 0,
    .StartBit = 24,
    .LengthBit = 8
};

sCanSignal g_can_signal_id023_ECU_LambdaHeaterVoltage = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.06f,
    .Offset = 0,
    .StartBit = 32,
    .LengthBit = 8
};

sCanSignal g_can_signal_id023_ECU_ShortTermCorrection = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.00390625,
    .Offset = -0.5f,
    .StartBit = 40,
    .LengthBit = 8
};

sCanSignal g_can_signal_id023_ECU_LongTermCorrection = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.00390625,
    .Offset = -0.5f,
    .StartBit = 48,
    .LengthBit = 8
};

sCanSignal g_can_signal_id023_ECU_IdleCorrection = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.00390625,
    .Offset = -0.5f,
    .StartBit = 56,
    .LengthBit = 8
};


sCanMessage g_can_message_id024_ECU = { 0x024, 8 }; // Air and Power

sCanSignal g_can_signal_id024_ECU_Speed = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.25f,
    .Offset = 0,
    .StartBit = 0,
    .LengthBit = 10
};

sCanSignal g_can_signal_id024_ECU_MassAirFlow = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.25f,
    .Offset = 0,
    .StartBit = 10,
    .LengthBit = 11
};

sCanSignal g_can_signal_id024_ECU_CyclicAirFlow = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.4f,
    .Offset = 0,
    .StartBit = 21,
    .LengthBit = 11
};

sCanSignal g_can_signal_id024_ECU_EffectiveVolume = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 10.0f,
    .Offset = 0,
    .StartBit = 32,
    .LengthBit = 8
};

sCanSignal g_can_signal_id024_ECU_EngineLoad = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.5f,
    .Offset = 0,
    .StartBit = 40,
    .LengthBit = 8
};

sCanSignal g_can_signal_id024_ECU_EstimatedPower = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 48,
    .LengthBit = 8
};

sCanSignal g_can_signal_id024_ECU_EstimatedTorque = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 56,
    .LengthBit = 8
};


sCanMessage g_can_message_id025_ECU = { 0x025, 8 }; // Ignition and Idle

sCanSignal g_can_signal_id025_ECU_WishFuelRatio = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.1f,
    .Offset = 0,
    .StartBit = 0,
    .LengthBit = 8
};

sCanSignal g_can_signal_id025_ECU_IdleValvePosition = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 8,
    .LengthBit = 8
};

sCanSignal g_can_signal_id025_ECU_IdleRegThrRPM = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 12,
    .Offset = 0,
    .StartBit = 16,
    .LengthBit = 8
};

sCanSignal g_can_signal_id025_ECU_WishIdleRPM = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 12,
    .Offset = 0,
    .StartBit = 24,
    .LengthBit = 8
};

sCanSignal g_can_signal_id025_ECU_WishIdleMassAirFlow = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.25f,
    .Offset = 0,
    .StartBit = 32,
    .LengthBit = 8
};

sCanSignal g_can_signal_id025_ECU_WishIdleValvePosition = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 40,
    .LengthBit = 8
};

sCanSignal g_can_signal_id025_ECU_WishIdleIgnitionAdvance = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.3f,
    .Offset = -20,
    .StartBit = 48,
    .LengthBit = 8
};

sCanSignal g_can_signal_id025_ECU_IdleSpeedShift = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.12f,
    .Offset = 0,
    .StartBit = 56,
    .LengthBit = 8
};


sCanMessage g_can_message_id026_ECU = { 0x026, 8 }; // Injection and Ignition

sCanSignal g_can_signal_id026_ECU_InjectionPhase = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 2.8125f,
    .Offset = 0,
    .StartBit = 0,
    .LengthBit = 8
};

sCanSignal g_can_signal_id026_ECU_InjectionPhaseDuration = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 2.8125f,
    .Offset = 0,
    .StartBit = 8,
    .LengthBit = 8
};

sCanSignal g_can_signal_id026_ECU_InjectionPulse = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 50,
    .Offset = 0,
    .StartBit = 16,
    .LengthBit = 10
};

sCanSignal g_can_signal_id026_ECU_InjectionDutyCycle = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.016f,
    .Offset = 0,
    .StartBit = 26,
    .LengthBit = 6
};

sCanSignal g_can_signal_id026_ECU_IgnitionPulse = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 40,
    .Offset = 0,
    .StartBit = 32,
    .LengthBit = 8
};

sCanSignal g_can_signal_id026_ECU_InjectionLag = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.02f,
    .Offset = 0,
    .StartBit = 40,
    .LengthBit = 8
};

sCanSignal g_can_signal_id026_ECU_TspsRelativePosition = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.2f,
    .Offset = -25,
    .StartBit = 48,
    .LengthBit = 8
};

sCanSignal g_can_signal_id026_ECU_IgnitionAdvance = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.3f,
    .Offset = -20,
    .StartBit = 56,
    .LengthBit = 8
};


sCanMessage g_can_message_id027_ECU = { 0x027, 8 }; // Enrichment

sCanSignal g_can_signal_id027_ECU_EnrichmentSyncAmount = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.0005f,
    .Offset = 0,
    .StartBit = 0,
    .LengthBit = 12
};

sCanSignal g_can_signal_id027_ECU_EnrichmentAsyncAmount = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.0005f,
    .Offset = 0,
    .StartBit = 12,
    .LengthBit = 12
};

sCanSignal g_can_signal_id027_ECU_EnrichmentStartLoad = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.5f,
    .Offset = 0,
    .StartBit = 24,
    .LengthBit = 8
};

sCanSignal g_can_signal_id027_ECU_EnrichmentLoadDerivative = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 5,
    .Offset = 0,
    .StartBit = 32,
    .LengthBit = 12
};

sCanSignal g_can_signal_id027_ECU_InjectionEnrichment = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.0005,
    .Offset = 0,
    .StartBit = 44,
    .LengthBit = 12
};

sCanSignal g_can_signal_id027_ECU_IdleWishToRpmRelation = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.02f,
    .Offset = 0,
    .StartBit = 56,
    .LengthBit = 8
};

sCanMessage g_can_message_id028_ECU = { 0x028, 8 }; // Odo and Fuel

sCanSignal g_can_signal_id028_ECU_DrivenKilometers = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.001f,
    .Offset = 0,
    .StartBit = 0,
    .LengthBit = 24
};

sCanSignal g_can_signal_id028_ECU_FuelConsumption = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.1f,
    .Offset = 0,
    .StartBit = 24,
    .LengthBit = 10
};

sCanSignal g_can_signal_id028_ECU_FuelConsumed = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.001f,
    .Offset = 0,
    .StartBit = 34,
    .LengthBit = 16
};

sCanSignal g_can_signal_id028_ECU_FuelHourly = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.1f,
    .Offset = 0,
    .StartBit = 50,
    .LengthBit = 14
};


sCanMessage g_can_message_id029_ECU = { 0x029, 8 }; // Flags

sCanSignal g_can_signal_id029_ECU_LambdaValid = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 0,
    .LengthBit = 1
};

sCanSignal g_can_signal_id029_ECU_OilSensor = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 1,
    .LengthBit = 1
};

sCanSignal g_can_signal_id029_ECU_FanForceSwitch = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 2,
    .LengthBit = 1
};

sCanSignal g_can_signal_id029_ECU_HandbrakeSensor = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 3,
    .LengthBit = 1
};

sCanSignal g_can_signal_id029_ECU_ChargeSensor = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 4,
    .LengthBit = 1
};

sCanSignal g_can_signal_id029_ECU_ClutchSensor = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 5,
    .LengthBit = 1
};

sCanSignal g_can_signal_id029_ECU_IgnSensor = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 6,
    .LengthBit = 1
};

sCanSignal g_can_signal_id029_ECU_FuelPumpRelay = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 7,
    .LengthBit = 1
};

sCanSignal g_can_signal_id029_ECU_FanRelay = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 8,
    .LengthBit = 1
};

sCanSignal g_can_signal_id029_ECU_CheckEngine = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 9,
    .LengthBit = 1
};

sCanSignal g_can_signal_id029_ECU_StarterRelay = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 10,
    .LengthBit = 1
};

sCanSignal g_can_signal_id029_ECU_FanSwitch = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 11,
    .LengthBit = 1
};

sCanSignal g_can_signal_id029_ECU_IgnOutput = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 12,
    .LengthBit = 1
};

sCanSignal g_can_signal_id029_ECU_StartAllowed = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 13,
    .LengthBit = 1
};

sCanSignal g_can_signal_id029_ECU_IsRunning = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 14,
    .LengthBit = 1
};

sCanSignal g_can_signal_id029_ECU_IsCheckEngine = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 15,
    .LengthBit = 1
};

sCanSignal g_can_signal_id029_ECU_CylinderIgnitionBitmask = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 16,
    .LengthBit = 4
};

sCanSignal g_can_signal_id029_ECU_CylinderInjectionBitmask = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 20,
    .LengthBit = 4
};

sCanSignal g_can_signal_id029_ECU_PowerVoltage = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.06f,
    .Offset = 5,
    .StartBit = 24,
    .LengthBit = 8
};

sCanSignal g_can_signal_id029_ECU_IdleFlag = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 32,
    .LengthBit = 1
};

sCanSignal g_can_signal_id029_ECU_IdleCorrFlag = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 33,
    .LengthBit = 1
};

sCanSignal g_can_signal_id029_ECU_IdleEconFlag = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 34,
    .LengthBit = 1
};

sCanSignal g_can_signal_id029_ECU_SwitchPosition = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 35,
    .LengthBit = 3
};

sCanSignal g_can_signal_id029_ECU_CurrentTable = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 38,
    .LengthBit = 3
};

sCanSignal g_can_signal_id029_ECU_InjectorChannel = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 41,
    .LengthBit = 3
};

static HAL_StatusTypeDef can_signal_message_clear(sCanMessage *message)
{
  HAL_StatusTypeDef ret = HAL_ERROR;

  if(message != NULL) {
    memset(message->MessageBuffer, 0, sizeof(message->MessageBuffer));
  }

  return ret;
}

static HAL_StatusTypeDef can_signal_append_raw(sCanMessage *message, const sCanSignal *signal, uint32_t raw_value)
{
  HAL_StatusTypeDef ret = HAL_ERROR;

  // TODO: implement

  return ret;
}

static HAL_StatusTypeDef can_signal_append_float(sCanMessage *message, sCanSignal *signal, float value)
{
  HAL_StatusTypeDef ret = HAL_ERROR;
  uint32_t raw_value;

  if(!signal->MinMaxDefined) {
    signal->MinValue = signal->Offset;
    signal->MaxValue = (powf(2, signal->LengthBit) - 1.0f) * signal->Gain + signal->Offset;
    signal->MinMaxDefined = 1;
  }

  value = CLAMP(value, signal->MinValue, signal->MaxValue);

  value -= signal->Offset;
  if (signal->Gain != 1.0f) {
    value /= signal->Gain;
  }

  raw_value = (uint32_t)value;

  ret = can_signal_append_raw(message, signal, raw_value);

  return ret;
}

static INLINE HAL_StatusTypeDef can_signal_append_uint(sCanMessage *message, sCanSignal *signal, uint32_t value)
{
  HAL_StatusTypeDef ret = HAL_ERROR;

  ret = can_signal_append_raw(message, signal, value);

  return ret;
}

static HAL_StatusTypeDef can_message_send(const sCanMessage *message)
{
  HAL_StatusTypeDef ret = HAL_ERROR;
  int8_t status = 0;
  sCanRawMessage raw_msg;

  raw_msg.id = message->Id;
  raw_msg.length = message->Length;
  raw_msg.rtr = CAN_RTR_DATA;
  memcpy(raw_msg.data.bytes, message->MessageBuffer, raw_msg.length);

  status = can_send(&raw_msg);
  if(status == 1) {
    ret = HAL_OK;
  }

  return ret;
}

static void can_signals_send(const sParameters *parameters)
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
}

void can_signals_update(const sParameters *parameters)
{
  static uint32_t last = 0;
  uint32_t now = Delay_Tick;

  if (DelayDiff(now, last) >= 10000) {
    can_signals_send(parameters);
  }
}
