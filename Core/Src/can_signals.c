/*
 * CAN_SIGNALs.c
 *
 *  Created on: 6 нояб. 2023 г.
 *      Author: VHEMaster
 */

#include "main.h"
#include "defines.h"
#include "structs.h"
#include "can_signals.h"

#define CAN_SIGNAL_BYTES_MAX  (8U)
#define CAN_SIGNAL_BITS_MAX   (CAN_SIGNAL_BYTES_MAX * 8U)

typedef enum {
  CAN_SIGNAL_TYPE_UNSIGNED,
  CAN_SIGNAL_TYPE_SIGNED,
  CAN_SIGNAL_TYPE_FLOAT,
}eCanSignalType;

typedef struct {
    eCanSignalType SignalType;
    float Gain;
    float Offset;
    uint8_t StartBit;
    uint8_t LengthBit;
    float MinValue;
    float MaxValue;
}sCanSignal;

typedef struct {
    uint32_t Id;
    uint8_t Length;
    uint8_t MessageBuffer[CAN_SIGNAL_BYTES_MAX];
}sCanMessage;

sCanMessage g_can_message_id020_ECU = { 0x020, 8 }; // ADC

sCanSignal CAN_SIGNAL_id020_ECU_AdcKnockVoltage = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.023529f,
    .Offset = 0,
    .StartBit = 0,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id020_ECU_AdcAirTemp = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.023529f,
    .Offset = 0,
    .StartBit = 8,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id020_ECU_AdcEngineTemp = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.023529f,
    .Offset = 0,
    .StartBit = 16,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id020_ECU_AdcManifoldAirPressure = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.023529f,
    .Offset = 0,
    .StartBit = 24,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id020_ECU_AdcThrottlePosition = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.023529f,
    .Offset = 0,
    .StartBit = 32,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id020_ECU_AdcReferenceVoltage = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.023529f,
    .Offset = 0,
    .StartBit = 40,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id020_ECU_AdcLambdaUR = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.023529f,
    .Offset = 0,
    .StartBit = 48,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id020_ECU_AdcLambdaUA = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.023529f,
    .Offset = 0,
    .StartBit = 56,
    .LengthBit = 8
};

sCanMessage g_can_message_id021_ECU = { 0x021, 8 }; // Knock

sCanSignal CAN_SIGNAL_id021_ECU_KnockSensor = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.023529f,
    .Offset = 0,
    .StartBit = 0,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id021_ECU_KnockSensorFiltered = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.023529f,
    .Offset = 0,
    .StartBit = 8,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id021_ECU_KnockSensorDetonate = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.023529f,
    .Offset = 0,
    .StartBit = 16,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id021_ECU_KnockSaturation = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.024414f,
    .Offset = 0,
    .StartBit = 24,
    .LengthBit = 12
};

sCanSignal CAN_SIGNAL_id021_ECU_KnockAdvance = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.5f,
    .Offset = 0,
    .StartBit = 36,
    .LengthBit = 4
};

sCanSignal CAN_SIGNAL_id021_ECU_KnockCountCy1 = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 40,
    .LengthBit = 4
};

sCanSignal CAN_SIGNAL_id021_ECU_KnockCountCy2 = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 44,
    .LengthBit = 4
};

sCanSignal CAN_SIGNAL_id021_ECU_KnockCountCy3 = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 48,
    .LengthBit = 4
};

sCanSignal CAN_SIGNAL_id021_ECU_KnockCountCy4 = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 52,
    .LengthBit = 4
};

sCanSignal CAN_SIGNAL_id021_ECU_KnockCount = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 56,
    .LengthBit = 8
};


sCanMessage g_can_message_id022_ECU = { 0x022, 8 }; // Parameters

sCanSignal CAN_SIGNAL_id022_ECU_AirTemp = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.2f,
    .Offset = -50,
    .StartBit = 0,
    .LengthBit = 10
};

sCanSignal CAN_SIGNAL_id022_ECU_EngineTemp = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.2f,
    .Offset = -50,
    .StartBit = 10,
    .LengthBit = 10
};

sCanSignal CAN_SIGNAL_id022_ECU_CalculatedAirTemp = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.2f,
    .Offset = -50,
    .StartBit = 20,
    .LengthBit = 10
};

sCanSignal CAN_SIGNAL_id022_ECU_ManifoldAirPressure = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 30,
    .Offset = 0,
    .StartBit = 30,
    .LengthBit = 12
};

sCanSignal CAN_SIGNAL_id022_ECU_ThrottlePosition = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.1f,
    .Offset = 0,
    .StartBit = 42,
    .LengthBit = 10
};

sCanSignal CAN_SIGNAL_id022_ECU_RPM = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 2.4414f,
    .Offset = 0,
    .StartBit = 52,
    .LengthBit = 12
};

sCanMessage g_can_message_id023_ECU = { 0x023, 8 }; // Lambda & Correction

sCanSignal CAN_SIGNAL_id023_ECU_FuelRatio = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.1f,
    .Offset = 0,
    .StartBit = 0,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id023_ECU_FuelRatioDiff = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.0078125f,
    .Offset = 0,
    .StartBit = 8,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id023_ECU_LambdaValue = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.0078125f,
    .Offset = 0,
    .StartBit = 16,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id023_ECU_LambdaTemperature = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 4,
    .Offset = 0,
    .StartBit = 24,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id023_ECU_LambdaHeaterVoltage = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.06f,
    .Offset = 0,
    .StartBit = 32,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id023_ECU_ShortTermCorrection = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.00390625,
    .Offset = -0.5f,
    .StartBit = 40,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id023_ECU_LongTermCorrection = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.00390625,
    .Offset = -0.5f,
    .StartBit = 48,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id023_ECU_IdleCorrection = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.00390625,
    .Offset = -0.5f,
    .StartBit = 56,
    .LengthBit = 8
};


sCanMessage g_can_message_id024_ECU = { 0x024, 8 }; // Air and Power

sCanSignal CAN_SIGNAL_id024_ECU_Speed = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.25f,
    .Offset = 0,
    .StartBit = 0,
    .LengthBit = 10
};

sCanSignal CAN_SIGNAL_id024_ECU_MassAirFlow = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.25f,
    .Offset = 0,
    .StartBit = 10,
    .LengthBit = 11
};

sCanSignal CAN_SIGNAL_id024_ECU_CyclicAirFlow = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.4f,
    .Offset = 0,
    .StartBit = 21,
    .LengthBit = 11
};

sCanSignal CAN_SIGNAL_id024_ECU_EffectiveVolume = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 10.0f,
    .Offset = 0,
    .StartBit = 32,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id024_ECU_EngineLoad = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.5f,
    .Offset = 0,
    .StartBit = 40,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id024_ECU_EstimatedPower = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 48,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id024_ECU_EstimatedTorque = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 56,
    .LengthBit = 8
};


sCanMessage g_can_message_id025_ECU = { 0x025, 8 }; // Ignition and Idle

sCanSignal CAN_SIGNAL_id025_ECU_WishFuelRatio = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.1f,
    .Offset = 0,
    .StartBit = 0,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id025_ECU_IdleValvePosition = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 8,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id025_ECU_IdleRegThrRPM = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 12,
    .Offset = 0,
    .StartBit = 16,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id025_ECU_WishIdleRPM = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 12,
    .Offset = 0,
    .StartBit = 24,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id025_ECU_WishIdleMassAirFlow = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.25f,
    .Offset = 0,
    .StartBit = 32,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id025_ECU_WishIdleValvePosition = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 40,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id025_ECU_WishIdleIgnitionAdvance = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.3f,
    .Offset = -20,
    .StartBit = 48,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id025_ECU_IdleSpeedShift = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.12f,
    .Offset = 0,
    .StartBit = 56,
    .LengthBit = 8
};


sCanMessage g_can_message_id026_ECU = { 0x026, 8 }; // Injection and Ignition

sCanSignal CAN_SIGNAL_id026_ECU_InjectionPhase = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 2.8125f,
    .Offset = 0,
    .StartBit = 0,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id026_ECU_InjectionPhaseDuration = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 2.8125f,
    .Offset = 0,
    .StartBit = 8,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id026_ECU_InjectionPulse = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 50,
    .Offset = 0,
    .StartBit = 16,
    .LengthBit = 10
};

sCanSignal CAN_SIGNAL_id026_ECU_InjectionDutyCycle = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.016f,
    .Offset = 0,
    .StartBit = 26,
    .LengthBit = 6
};

sCanSignal CAN_SIGNAL_id026_ECU_IgnitionPulse = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 40,
    .Offset = 0,
    .StartBit = 32,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id026_ECU_InjectionLag = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.02f,
    .Offset = 0,
    .StartBit = 40,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id026_ECU_TspsRelativePosition = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.2f,
    .Offset = -25,
    .StartBit = 48,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id026_ECU_IgnitionAdvance = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.3f,
    .Offset = -20,
    .StartBit = 56,
    .LengthBit = 8
};


sCanMessage g_can_message_id027_ECU = { 0x027, 8 }; // Enrichment

sCanSignal CAN_SIGNAL_id027_ECU_EnrichmentSyncAmount = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.0005f,
    .Offset = 0,
    .StartBit = 0,
    .LengthBit = 12
};

sCanSignal CAN_SIGNAL_id027_ECU_EnrichmentAsyncAmount = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.0005f,
    .Offset = 0,
    .StartBit = 12,
    .LengthBit = 12
};

sCanSignal CAN_SIGNAL_id027_ECU_EnrichmentStartLoad = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.5f,
    .Offset = 0,
    .StartBit = 24,
    .LengthBit = 8
};

sCanSignal CAN_SIGNAL_id027_ECU_EnrichmentLoadDerivative = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 5,
    .Offset = 0,
    .StartBit = 32,
    .LengthBit = 12
};

sCanSignal CAN_SIGNAL_id027_ECU_InjectionEnrichment = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.0005,
    .Offset = 0,
    .StartBit = 44,
    .LengthBit = 12
};

sCanSignal CAN_SIGNAL_id027_ECU_IdleWishToRpmRelation = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.02f,
    .Offset = 0,
    .StartBit = 56,
    .LengthBit = 8
};

sCanMessage g_can_message_id028_ECU = { 0x028, 8 }; // Odo and Fuel

sCanSignal CAN_SIGNAL_id028_ECU_DrivenKilometers = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.001f,
    .Offset = 0,
    .StartBit = 0,
    .LengthBit = 24
};

sCanSignal CAN_SIGNAL_id028_ECU_FuelConsumption = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.1f,
    .Offset = 0,
    .StartBit = 24,
    .LengthBit = 10
};

sCanSignal CAN_SIGNAL_id028_ECU_FuelConsumed = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.001f,
    .Offset = 0,
    .StartBit = 34,
    .LengthBit = 16
};

sCanSignal CAN_SIGNAL_id028_ECU_FuelHourly = {
    .SignalType = CAN_SIGNAL_TYPE_FLOAT,
    .Gain = 0.1f,
    .Offset = 0,
    .StartBit = 50,
    .LengthBit = 14
};


sCanMessage g_can_message_id029_ECU = { 0x029, 8 }; // Flags

sCanSignal CAN_SIGNAL_id029_ECU_LambdaValid = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 0,
    .LengthBit = 1
};

sCanSignal CAN_SIGNAL_id029_ECU_OilSensor = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 1,
    .LengthBit = 1
};

sCanSignal CAN_SIGNAL_id029_ECU_FanForceSwitch = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 2,
    .LengthBit = 1
};

sCanSignal CAN_SIGNAL_id029_ECU_HandbrakeSensor = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 3,
    .LengthBit = 1
};

sCanSignal CAN_SIGNAL_id029_ECU_ChargeSensor = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 4,
    .LengthBit = 1
};

sCanSignal CAN_SIGNAL_id029_ECU_ClutchSensor = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 5,
    .LengthBit = 1
};

sCanSignal CAN_SIGNAL_id029_ECU_IgnSensor = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 6,
    .LengthBit = 1
};

sCanSignal CAN_SIGNAL_id029_ECU_FuelPumpRelay = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 7,
    .LengthBit = 1
};

sCanSignal CAN_SIGNAL_id029_ECU_FanRelay = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 8,
    .LengthBit = 1
};

sCanSignal CAN_SIGNAL_id029_ECU_CheckEngine = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 9,
    .LengthBit = 1
};

sCanSignal CAN_SIGNAL_id029_ECU_StarterRelay = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 10,
    .LengthBit = 1
};

sCanSignal CAN_SIGNAL_id029_ECU_FanSwitch = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 11,
    .LengthBit = 1
};

sCanSignal CAN_SIGNAL_id029_ECU_IgnOutput = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 12,
    .LengthBit = 1
};

sCanSignal CAN_SIGNAL_id029_ECU_StartAllowed = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 13,
    .LengthBit = 1
};

sCanSignal CAN_SIGNAL_id029_ECU_IsRunning = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 14,
    .LengthBit = 1
};

sCanSignal CAN_SIGNAL_id029_ECU_IsCheckEngine = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 15,
    .LengthBit = 1
};

sCanSignal CAN_SIGNAL_id029_ECU_CylinderIgnitionBitmask = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 16,
    .LengthBit = 4
};

sCanSignal CAN_SIGNAL_id029_ECU_CylinderInjectionBitmask = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 20,
    .LengthBit = 4
};

sCanSignal CAN_SIGNAL_id029_ECU_IdleFlag = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 24,
    .LengthBit = 1
};

sCanSignal CAN_SIGNAL_id029_ECU_IdleCorrFlag = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 25,
    .LengthBit = 1
};

sCanSignal CAN_SIGNAL_id029_ECU_IdleEconFlag = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 26,
    .LengthBit = 1
};

sCanSignal CAN_SIGNAL_id029_ECU_SwitchPosition = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 26,
    .LengthBit = 3
};

sCanSignal CAN_SIGNAL_id029_ECU_CurrentTable = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 29,
    .LengthBit = 3
};

sCanSignal CAN_SIGNAL_id029_ECU_InjectorChannel = {
    .SignalType = CAN_SIGNAL_TYPE_UNSIGNED,
    .Gain = 1,
    .Offset = 0,
    .StartBit = 32,
    .LengthBit = 3
};

void can_signals_update(const sParameters *parameters)
{

}
