/*
 * structs.h
 *
 *  Created on: 3 мар. 2022 г.
 *      Author: VHEMaster
 */

#ifndef STRUCTS_H_
#define STRUCTS_H_

#include "main.h"
#include "defines.h"

typedef struct {
    float RPM;
    float Speed;
    float Acceleration;
    float Pressure;
    float Ignition;
    float Mixture;
    uint32_t Time;
}sDragPoint;

typedef enum {
  InjectorChannel1 = 0,
  InjectorChannel2,
  InjectorChannelCount
}eInjChannel;

typedef struct {
    char name[TABLE_STRING_MAX];

    eInjChannel inj_channel;

    float ignition_initial;
    float injector_performance;
    float fuel_pressure;
    float fuel_mass_per_cc;

    int32_t voltages_count;
    float voltages[TABLE_VOLTAGES_MAX];

    int32_t pressures_count;
    float pressures[TABLE_PRESSURES_MAX];

    int32_t rotates_count;
    float rotates[TABLE_ROTATES_MAX];

    int32_t throttles_count;
    float throttles[TABLE_THROTTLES_MAX];

    float fill_proportion_map_vs_thr;
    float enrichment_proportion_map_vs_thr;
    float fill_by_map[TABLE_PRESSURES_MAX][TABLE_ROTATES_MAX];
    float fill_by_thr[TABLE_THROTTLES_MAX][TABLE_ROTATES_MAX];
    float enrichment_by_map_sens[TABLE_PRESSURES_MAX];
    float enrichment_by_map_hpf[TABLE_ROTATES_MAX];
    float enrichment_by_thr_sens[TABLE_THROTTLES_MAX];
    float enrichment_by_thr_hpf[TABLE_ROTATES_MAX];

    int32_t fillings_count;
    float fillings[TABLE_FILLING_MAX];
    float ignitions[TABLE_FILLING_MAX][TABLE_ROTATES_MAX];
    float fuel_mixtures[TABLE_FILLING_MAX][TABLE_ROTATES_MAX];
    float injection_phase[TABLE_FILLING_MAX][TABLE_ROTATES_MAX];

    float ignition_time_rpm_mult[TABLE_ROTATES_MAX];
    float ignition_time[TABLE_VOLTAGES_MAX];
    float injector_lag[TABLE_VOLTAGES_MAX];

    int32_t engine_temp_count;
    float engine_temps[TABLE_TEMPERATURES_MAX];

    //TODO: maybe not needed?..
    //int32_t air_temp_count;
    //float air_temps[TABLE_TEMPERATURES_MAX];
    //float air_temp_mix_corr[TABLE_TEMPERATURES_MAX][TABLE_ROTATES_MAX];

    float idle_wish_rotates[TABLE_TEMPERATURES_MAX];
    float idle_wish_massair[TABLE_TEMPERATURES_MAX];
    float idle_wish_ignition[TABLE_ROTATES_MAX];
    float idle_valve_to_rpm[TABLE_ROTATES_MAX];

    float idle_valve_to_massair_pid_p;
    float idle_valve_to_massair_pid_i;
    float idle_valve_to_massair_pid_d;

    float idle_ign_to_rpm_pid_p;
    float idle_ign_to_rpm_pid_i;
    float idle_ign_to_rpm_pid_d;

    float idle_ign_deviation_max;
    float idle_ign_deviation_min;

    float idle_ign_fan_corr;

    int32_t idle_speeds_shift_count;
    float idle_rpm_shift_speeds[TABLE_SPEEDS_MAX];
    float idle_rpm_shift[TABLE_SPEEDS_MAX];

    float knock_noise_level[TABLE_ROTATES_MAX];

    int32_t Reserved[128];
}sEcuTable __attribute__((aligned(32)));

typedef struct {
    int16_t ignitions[TABLE_FILLING_MAX][TABLE_ROTATES_MAX];
    int16_t fill_by_map[TABLE_PRESSURES_MAX][TABLE_ROTATES_MAX];
    int16_t fill_by_thr[TABLE_THROTTLES_MAX][TABLE_ROTATES_MAX];
    int16_t idle_valve_to_rpm[TABLE_ROTATES_MAX];
}sEcuCorrections;

typedef struct {
    int32_t tables_count;

    float engineVolume;

    int32_t isCutoffEnabled;
    int32_t isForceTable;
    int32_t isSwitchByExternal;
    int32_t startupTableNumber;
    int32_t switchPos1Table;
    int32_t switchPos0Table;
    int32_t switchPos2Table;
    int32_t switchTime;
    int32_t forceTable;

    float cutoffRPM;
    int32_t cutoffMode;
    float cutoffAngle;
    float cutoffMixture;

    float speedCorrection;

    int32_t useLambdaSensor;
    int32_t useTSPS;
    int32_t useKnockSensor;
    int32_t performAdaptation;
    int32_t isIndividualCoils;

    float fanHighTemperature;
    float fanLowTemperature;

    int32_t Reserved32[1001];
}sEcuParams;

typedef struct {
    int32_t SwitchPosition;
    int32_t CurrentTable;

    float AdcKnockVoltage;
    float AdcAirTemp;
    float AdcEngineTemp;
    float AdcManifoldAirPressure;
    float AdcThrottlePosition;
    float AdcPowerVoltage;
    float AdcReferenceVoltage;
    float AdcLambdaUR;
    float AdcLambdaUA;

    float KnockSensor;
    float KnockSensorFiltered;
    float AirTemp;
    float EngineTemp;
    float ManifoldAirPressure;
    float ThrottlePosition;
    float ReferenceVoltage;
    float PowerVoltage;
    float FuelRatio;

    int32_t IdleFlag;
    float RPM;
    float Speed;
    float MassAirFlow;
    float CyclicAirFlow;
    float EffectiveVolume;
    float AirDestiny;
    float WishFuelRatio;
    float IdleValvePosition;
    float WishIdleRPM;
    float WishIdleMassAirFlow;
    float WishIdleValvePosition;
    float WishIdleIgnitionAngle;
    float IgnitionAngle;
    float InjectionPhase;
    float InjectionPulse;
    float InjectionDutyCycle;
    float InjectionEnrichment;
    float IgnitionPulse;
    float IdleSpeedShift;

    float DrivenKilometers;
    float FuelConsumed;
    float FuelConsumption;

    int32_t OilSensor;
    int32_t StarterSensor;
    int32_t HandbrakeSensor;
    int32_t ChargeSensor;
    int32_t Rsvd1Sensor;
    int32_t Rsvd2Sensor;

    int32_t FuelPumpRelay;
    int32_t FanRelay;
    int32_t CheckEngine;
    int32_t StarterRelay;
    int32_t Rsvd1Output;
    int32_t Rsvd2Output;

    int32_t StartAllowed;
    int32_t IsRunning;
}sParameters;

typedef struct {
    union {
      struct {
        uint8_t IgnitionAngle : 1;
        uint8_t InjectionPhase : 1;
        uint8_t IgnitionOctane : 1;
        uint8_t WishFuelRatio : 1;
        uint8_t WishIdleRPM : 1;
        uint8_t WishIdleValvePosition : 1;
        uint8_t WishIdleIgnitionAngle : 1;
        uint8_t InjectionTime : 1;
        uint8_t IgnitionTime : 1;
        uint8_t FanRelay : 1;
        uint8_t FuelPumpRelay : 1;
      } params;
      uint32_t dword;
    } Enable;
    float IgnitionAngle;
    float InjectionPhase;
    float IgnitionOctane;
    float WishFuelRatio;
    float WishIdleRPM;
    float WishIdleValvePosition;
    float WishIdleIgnitionAngle;
    float InjectionTime;
    float IgnitionTime;
    float FanRelay;
    float FuelPumpRelay;
}sForceParameters;

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
            HAL_StatusTypeDef Save : 2;
            HAL_StatusTypeDef Load : 2;
        }Struct;
        uint8_t Byte;
    }Bkpsram;
    union {
        struct {
            //TODO: continue sensors
            HAL_StatusTypeDef Sensor : 2;
        };
        uint32_t Dword;
    }Sensors;
    //TODO: add more diagnostic fields
}sStatus;

typedef struct {
    float km_driven;
    float fuel_consumed;
    sStatus status_recorded;
    uint8_t idle_valve_position;
}sEcuCriticalBackup;

#endif /* STRUCTS_H_ */
