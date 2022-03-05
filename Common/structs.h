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

typedef struct {
    char name[TABLE_STRING_MAX];

    uint32_t inj_channel;

    float ignition_initial;
    float fuel_volume;

    int32_t pressures_count;
    float pressures[TABLE_PRESSURES_MAX];

    int32_t rotates_count;
    float rotates[TABLE_ROTATES_MAX];

    int32_t throttles_count;
    float throttles[TABLE_THROTTLES_MAX];

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

    float ignition_time[TABLE_ROTATES_MAX];

    int32_t engine_temp_count;
    float engine_temps[TABLE_TEMPERATURES_MAX];

    int32_t air_temp_count;
    float air_temps[TABLE_TEMPERATURES_MAX];
    float air_temp_mix_corr[TABLE_TEMPERATURES_MAX][TABLE_ROTATES_MAX];

    int32_t idle_rotates_count;
    float idle_rotates[TABLE_ROTATES_MAX];
    float idle_wish_rotates[TABLE_TEMPERATURES_MAX];
    float idle_wish_massair[TABLE_TEMPERATURES_MAX];
    float idle_wish_ignition[TABLE_ROTATES_MAX];
    float idle_fuel_mixture[TABLE_ROTATES_MAX];

    float idle_valve_to_massair_proporion;
    float idle_valve_to_massair_pid_p;
    float idle_valve_to_massair_pid_i;
    float idle_valve_to_massair_pid_d;

    float idle_ign_fan_corr;
    float idle_ign_to_rpm_pid_p;
    float idle_ign_to_rpm_pid_i;
    float idle_ign_to_rpm_pid_d;

    float idle_ign_deviation_max;
    float idle_ign_deviation_min;

    int32_t idle_speeds_shift_count;
    float idle_rpm_shift_speeds[TABLE_SPEEDS_MAX];
    float idle_rpm_shift[TABLE_SPEEDS_MAX];

    int32_t Reserved[1024];
}sEcuTable __attribute__((aligned(32)));

typedef struct {
    int32_t isCutoffEnabled;
    int32_t isSwitchByExternal;
    int32_t isForceTable;
    int32_t forceTableNumber;
    int32_t switchPos1Table;
    int32_t switchPos0Table;
    int32_t switchPos2Table;
    int32_t switchTime;

    float cutoffRPM;
    int32_t cutoffMode;
    float cutoffAngle;
    float cutoffMixture;
    float engineVolume;

    float speedCorrection;

    int32_t Reserved32[51];
}sEcuParams;

typedef struct {
    int32_t tables_count;
    sEcuParams params;
    sEcuTable tables[TABLE_SETUPS_MAX];
    uint32_t version;
    uint16_t crc;
}sEcuConfig;

typedef struct {
    float AdcKnockSensor;
    float AdcAirTemp;
    float AdcEngineTemp;
    float AdcManifoldAirPressure;
    float AdcThrottlePosition;
    float AdcPowerVoltage;
    float AdcLambdaUR;
    float AdcLambdaUA;

    float KnockSensor;
    float AirTemp;
    float EngineTemp;
    float ManifoldAirPressure;
    float ThrottlePosition;
    float PowerVoltage;
    float FuelRatio;

    int32_t IdleFlag;
    float RPM;
    float Speed;
    float MassAirFlow;
    float CyclicAirFlow;
    float CyclicFilling;
    float EngineLoad;
    float WishFuelRatio;
    float IdleValvePosition;
    float WishIdleRPM;
    float WishIdleMassAirFlow;
    float WishIdleValvePosition;
    float WishIdleIgnitionAngle;
    float IgnitionAngle;
    float InjectionPhase;
    float IgnitionTime;

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
}sParameters;

typedef struct {
    uint32_t Enable;
    float IgnitionAngle;
    float InjectionPhase;
    float IgnitionOctane;
    float WishFuelRatio;
    float WishIdleRPM;
    float WishIdleValvePosition;
    float WishIdleIgnitionAngle;
    float InjectionTime[4];
    float IgnitionTime[4];
    float IgnitionTimePP[2];
    float FanRelay;
    float FuelPumpRelay;
}sForceParameters;

#endif /* STRUCTS_H_ */
