/*
 * config.c
 *
 *  Created on: 7 мар. 2022 г.
 *      Author: VHEMaster
 */

#include "config.h"
#include "structs.h"
#include "flash.h"
#include "defines.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#define CONFIG_OFFSET_CORRECTIONS 0
#define CONFIG_OFFSET_CRITICALS 1920

static sEcuCorrectionsBackup tableBackupCorrections = {0};

static const float default_pressures[TABLE_PRESSURES_MAX] = {
    10000, 16200, 22400, 28600, 34800, 41000, 47200, 53400,
    59600, 65800, 72000, 78200, 84400, 90600, 96800, 103000
};

static const float default_rotates[TABLE_ROTATES_MAX] = {
    600, 740, 870, 1050, 1250, 1490, 1800, 2150,
    2560, 3040, 3590, 4310, 5100, 6060, 7190, 8500
};

static const float default_throttles[TABLE_THROTTLES_MAX] = {
    0.0f, 1.73f, 3.47f, 5.65f, 8.26f, 11.30f, 15.22f, 19.56f,
    24.78f, 30.87f, 37.83f, 46.96f, 56.96f, 69.13f, 83.48f, 100.0f
};

static const float default_fillings[TABLE_FILLING_MAX] = {
    33, 65, 98, 130, 163, 195, 228, 260,
    293, 325, 358, 390, 423, 455, 488, 520,

};

static const float default_ignitions[TABLE_FILLING_MAX][TABLE_ROTATES_MAX] = {
    { 16.1f, 18.0f, 20.1f, 23.1f, 26.6f, 31.0f, 38.1f, 41.0f, 42.6f, 43.4f, 44.0f, 44.4f, 44.3f, 44.3f, 44.8f, 45.0f },
    { 17.8f, 19.7f, 21.5f, 24.2f, 27.6f, 31.7f, 38.3f, 41.1f, 42.6f, 43.4f, 44.0f, 44.4f, 44.3f, 44.3f, 44.8f, 45.0f },
    { 18.7f, 20.5f, 22.2f, 24.6f, 27.7f, 31.8f, 38.2f, 40.9f, 42.4f, 43.2f, 43.7f, 44.1f, 44.0f, 44.1f, 44.7f, 44.9f },
    { 18.7f, 20.4f, 21.9f, 24.3f, 27.3f, 31.3f, 37.6f, 40.3f, 41.7f, 42.4f, 42.9f, 43.2f, 43.1f, 43.3f, 43.8f, 44.2f },
    { 17.4f, 19.4f, 20.9f, 23.2f, 26.2f, 30.1f, 36.3f, 39.1f, 40.6f, 41.3f, 41.9f, 42.2f, 42.1f, 42.3f, 42.9f, 43.1f },
    { 14.9f, 17.2f, 18.7f, 21.1f, 24.3f, 28.4f, 34.3f, 37.3f, 38.9f, 39.8f, 40.6f, 41.0f, 41.0f, 41.3f, 41.8f, 42.0f },
    { 12.2f, 14.5f, 15.9f, 17.9f, 21.5f, 26.0f, 31.9f, 34.0f, 36.8f, 37.2f, 38.9f, 39.5f, 39.9f, 40.2f, 40.8f, 41.0f },
    { 10.2f, 11.9f, 13.1f, 14.9f, 18.3f, 22.8f, 28.4f, 31.9f, 34.2f, 35.4f, 37.2f, 38.1f, 38.7f, 39.1f, 39.7f, 39.9f },
    { 8.5f, 9.7f, 10.8f, 12.4f, 15.3f, 19.2f, 24.0f, 27.9f, 30.0f, 33.2f, 35.4f, 36.5f, 36.7f, 37.0f, 37.5f, 37.7f },
    { 7.1f, 8.2f, 9.2f, 10.6f, 12.9f, 16.1f, 20.8f, 24.0f, 28.3f, 30.8f, 33.4f, 34.2f, 33.7f, 33.9f, 34.2f, 34.3f },
    { 6.3f, 7.2f, 8.1f, 9.3f, 11.1f, 14.0f, 18.0f, 23.3f, 26.9f, 29.1f, 31.6f, 32.0f, 31.0f, 31.3f, 31.7f, 31.8f },
    { 6.0f, 6.5f, 7.1f, 7.9f, 9.1f, 11.3f, 15.8f, 20.3f, 24.4f, 26.6f, 28.7f, 28.8f, 27.6f, 27.9f, 28.6f, 28.8f },
    { 6.0f, 6.5f, 7.0f, 7.7f, 8.7f, 10.4f, 13.8f, 17.6f, 21.5f, 24.2f, 26.3f, 26.7f, 25.3f, 25.6f, 26.0f, 26.4f },
    { 6.0f, 6.4f, 7.0f, 7.6f, 8.5f, 9.9f, 12.3f, 15.7f, 19.2f, 22.2f, 24.3f, 24.4f, 23.3f, 23.6f, 23.9f, 24.3f },
    { 6.0f, 6.2f, 6.7f, 7.3f, 8.2f, 9.3f, 11.0f, 14.2f, 18.0f, 20.8f, 22.8f, 22.9f, 21.8f, 22.1f, 22.5f, 22.8f },
    { 6.0f, 6.0f, 6.4f, 7.0f, 8.0f, 8.9f, 10.4f, 13.1f, 16.7f, 19.6f, 21.5f, 21.5f, 20.4f, 20.6f, 20.9f, 21.2f },

};

static const float default_filling_by_map[TABLE_PRESSURES_MAX][TABLE_ROTATES_MAX] = {
	{ 0.600f, 0.600f, 0.600f, 0.600f, 0.600f, 0.600f, 0.600f, 0.700f, 0.800f, 0.900f, 1.000f, 1.000f, 0.900f, 0.800f, 0.800f, 0.700f, },
	{ 0.600f, 0.600f, 0.600f, 0.600f, 0.600f, 0.598f, 0.575f, 0.666f, 0.793f, 0.893f, 0.989f, 1.000f, 0.900f, 0.800f, 0.800f, 0.700f, },
	{ 0.600f, 0.600f, 0.600f, 0.592f, 0.589f, 0.605f, 0.561f, 0.568f, 0.722f, 0.761f, 0.873f, 1.000f, 0.900f, 0.800f, 0.800f, 0.700f, },
	{ 0.591f, 0.539f, 0.532f, 0.507f, 0.522f, 0.548f, 0.598f, 0.627f, 0.637f, 0.592f, 0.772f, 1.000f, 0.900f, 0.800f, 0.800f, 0.700f, },
	{ 0.589f, 0.547f, 0.618f, 0.583f, 0.576f, 0.638f, 0.640f, 0.705f, 0.643f, 0.755f, 0.967f, 1.000f, 0.900f, 0.800f, 0.800f, 0.700f, },
	{ 0.599f, 0.616f, 0.626f, 0.630f, 0.627f, 0.647f, 0.668f, 0.727f, 0.794f, 0.868f, 0.998f, 1.000f, 0.900f, 0.800f, 0.800f, 0.700f, },
	{ 0.609f, 0.637f, 0.644f, 0.636f, 0.680f, 0.706f, 0.676f, 0.718f, 0.807f, 0.899f, 1.000f, 1.000f, 0.900f, 0.800f, 0.800f, 0.700f, },
	{ 0.592f, 0.600f, 0.600f, 0.602f, 0.606f, 0.623f, 0.624f, 0.701f, 0.801f, 0.900f, 1.000f, 1.000f, 0.900f, 0.800f, 0.800f, 0.700f, },
	{ 0.700f, 0.700f, 0.700f, 0.700f, 0.700f, 0.700f, 0.700f, 0.700f, 0.800f, 0.900f, 1.000f, 1.000f, 0.900f, 0.800f, 0.800f, 0.700f, },
	{ 0.700f, 0.700f, 0.700f, 0.700f, 0.700f, 0.700f, 0.700f, 0.700f, 0.800f, 0.900f, 1.000f, 1.000f, 0.900f, 0.800f, 0.800f, 0.700f, },
	{ 0.700f, 0.700f, 0.700f, 0.700f, 0.700f, 0.700f, 0.700f, 0.800f, 0.800f, 0.900f, 1.000f, 1.000f, 0.900f, 0.800f, 0.800f, 0.700f, },
	{ 0.800f, 0.800f, 0.800f, 0.800f, 0.800f, 0.800f, 0.800f, 0.800f, 0.800f, 0.900f, 1.000f, 1.000f, 0.900f, 0.800f, 0.800f, 0.700f, },
	{ 0.900f, 0.900f, 0.900f, 0.900f, 0.900f, 0.900f, 0.900f, 0.900f, 0.800f, 0.900f, 1.000f, 1.000f, 0.900f, 0.800f, 0.800f, 0.700f, },
	{ 0.900f, 0.900f, 0.900f, 0.900f, 0.900f, 0.900f, 0.900f, 0.900f, 0.800f, 0.900f, 1.000f, 1.000f, 0.900f, 0.800f, 0.800f, 0.700f, },
	{ 0.900f, 0.900f, 0.900f, 0.900f, 0.900f, 0.900f, 0.900f, 0.900f, 0.900f, 0.900f, 1.000f, 1.000f, 0.900f, 0.800f, 0.800f, 0.700f, },
	{ 0.900f, 0.900f, 0.900f, 0.900f, 0.900f, 0.900f, 0.900f, 0.900f, 0.900f, 0.900f, 1.000f, 1.000f, 0.900f, 0.800f, 0.800f, 0.700f, },



};

static const float default_map_by_thr[TABLE_THROTTLES_MAX][TABLE_ROTATES_MAX] = {
    { 50000, 40000, 20000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000 },
    { 50000, 40000, 25000, 13000, 13000, 13000, 13000, 13000, 13000, 13000, 13000, 13000, 13000, 13000, 13000, 13000 },
    { 50000, 40000, 30000, 19000, 19000, 19000, 19000, 19000, 19000, 19000, 19000, 19000, 19000, 19000, 19000, 19000 },
    { 50000, 40000, 35000, 25000, 25000, 25000, 25000, 25000, 25000, 25000, 25000, 25000, 25000, 25000, 25000, 25000 },
    { 55000, 50000, 40000, 32000, 32000, 32000, 32000, 32000, 32000, 32000, 32000, 32000, 32000, 32000, 32000, 32000 },
    { 60000, 50000, 45000, 38000, 38000, 38000, 38000, 38000, 38000, 38000, 38000, 38000, 38000, 38000, 38000, 38000 },
    { 65000, 60000, 50000, 44000, 44000, 44000, 44000, 44000, 44000, 44000, 44000, 44000, 44000, 44000, 44000, 44000 },
    { 70000, 65000, 55000, 51000, 51000, 51000, 51000, 51000, 51000, 51000, 51000, 51000, 51000, 51000, 51000, 51000 },
    { 80000, 70000, 60000, 57000, 57000, 57000, 57000, 57000, 57000, 57000, 57000, 57000, 57000, 57000, 57000, 57000 },
    { 90000, 80000, 70000, 63000, 63000, 63000, 63000, 63000, 63000, 63000, 63000, 63000, 63000, 63000, 63000, 63000 },
    { 100000, 90000, 75000, 70000, 70000, 70000, 70000, 70000, 70000, 70000, 70000, 70000, 70000, 70000, 70000, 70000 },
    { 100000, 90000, 80000, 76000, 76000, 76000, 76000, 76000, 76000, 76000, 76000, 76000, 76000, 76000, 76000, 76000 },
    { 100000, 95000, 90000, 82000, 82000, 82000, 82000, 82000, 82000, 82000, 82000, 82000, 82000, 82000, 82000, 82000 },
    { 100000, 97000, 95000, 89000, 89000, 89000, 89000, 89000, 89000, 89000, 89000, 89000, 89000, 89000, 89000, 89000 },
    { 100000, 100000, 100000, 95000, 95000, 95000, 95000, 95000, 95000, 95000, 95000, 95000, 95000, 95000, 95000, 95000 },
    { 101000, 101000, 101000, 101000, 101000, 101000, 101000, 101000, 101000, 101000, 101000, 101000, 101000, 101000, 101000, 101000 }
};

static const float default_fuel_mixtures[TABLE_FILLING_MAX][TABLE_ROTATES_MAX] = {
    { 13.6f, 13.8f, 14.0f, 14.0f, 14.0f, 14.0f, 14.1f, 14.2f, 14.3f, 14.4f, 14.5f, 14.6f, 14.6f, 14.7f, 14.7f, 14.7f },
    { 13.6f, 13.8f, 14.0f, 14.0f, 14.0f, 14.0f, 14.1f, 14.2f, 14.3f, 14.4f, 14.5f, 14.6f, 14.6f, 14.7f, 14.7f, 14.7f },
    { 13.6f, 13.8f, 14.0f, 14.0f, 14.0f, 14.0f, 14.1f, 14.2f, 14.3f, 14.4f, 14.5f, 14.6f, 14.7f, 14.7f, 14.7f, 14.7f },
    { 13.6f, 13.8f, 14.0f, 14.0f, 14.0f, 14.0f, 14.1f, 14.2f, 14.3f, 14.4f, 14.6f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f },
    { 13.6f, 13.8f, 14.0f, 14.0f, 14.0f, 14.0f, 14.1f, 14.2f, 14.3f, 14.4f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f },
    { 13.6f, 13.8f, 14.0f, 14.0f, 14.0f, 14.1f, 14.2f, 14.3f, 14.4f, 14.5f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f },
    { 13.6f, 13.8f, 14.0f, 14.0f, 14.1f, 14.2f, 14.3f, 14.4f, 14.5f, 14.6f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f },
    { 13.6f, 13.8f, 14.0f, 14.1f, 14.2f, 14.3f, 14.4f, 14.5f, 14.6f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f },
    { 13.6f, 13.8f, 14.2f, 14.3f, 14.4f, 14.5f, 14.6f, 14.6f, 14.6f, 14.6f, 14.6f, 14.6f, 14.6f, 14.6f, 14.6f, 14.6f },
    { 13.4f, 13.6f, 14.1f, 14.1f, 14.2f, 14.2f, 14.3f, 14.3f, 14.3f, 14.3f, 14.3f, 14.3f, 14.3f, 14.3f, 14.3f, 14.3f },
    { 13.3f, 13.5f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f },
    { 13.2f, 13.3f, 13.6f, 13.6f, 13.6f, 13.6f, 13.6f, 13.6f, 13.6f, 13.6f, 13.6f, 13.6f, 13.6f, 13.6f, 13.6f, 13.6f },
    { 13.0f, 13.1f, 13.2f, 13.3f, 13.4f, 13.4f, 13.4f, 13.4f, 13.4f, 13.4f, 13.4f, 13.4f, 13.4f, 13.4f, 13.4f, 13.4f },
    { 13.0f, 13.1f, 13.2f, 13.3f, 13.4f, 13.4f, 13.4f, 13.4f, 13.4f, 13.4f, 13.4f, 13.4f, 13.4f, 13.4f, 13.4f, 13.4f },
    { 13.0f, 13.1f, 13.2f, 13.3f, 13.3f, 13.3f, 13.3f, 13.3f, 13.3f, 13.3f, 13.3f, 13.3f, 13.3f, 13.3f, 13.3f, 13.3f },
    { 13.0f, 13.1f, 13.2f, 13.3f, 13.3f, 13.3f, 13.3f, 13.3f, 13.3f, 13.3f, 13.3f, 13.3f, 13.3f, 13.3f, 13.3f, 13.3f },
};

static const float default_injection_phase[TABLE_FILLING_MAX][TABLE_ROTATES_MAX] = {
    { 60, 80, 100, 100, 100, 100, 120, 140, 170, 190, 200, 200, 150, 100, 50, 0 },
    { 60, 80, 100, 100, 120, 120, 140, 170, 190, 190, 200, 200, 180, 140, 50, 10 },
    { 60, 80, 100, 100, 120, 140, 160, 190, 200, 210, 230, 220, 190, 150, 60, 20 },
    { 60, 80, 100, 100, 120, 150, 170, 190, 210, 230, 260, 240, 200, 160, 70, 30 },
    { 60, 80, 100, 100, 120, 140, 180, 200, 220, 250, 280, 260, 210, 170, 100, 40 },
    { 60, 80, 100, 100, 120, 160, 180, 200, 220, 250, 300, 280, 220, 180, 110, 60 },
    { 00, 80, 100, 100, 120, 160, 180, 200, 220, 250, 300, 300, 230, 190, 120, 90 },
    { 60, 80, 100, 100, 120, 160, 180, 200, 220, 250, 300, 300, 240, 200, 150, 130 },
    { 06, 80, 100, 100, 120, 160, 180, 200, 220, 250, 300, 300, 250, 220, 200, 170 },
    { 60, 80, 100, 100, 120, 160, 180, 200, 220, 250, 300, 300, 250, 240, 220, 210 },
    { 60, 80, 100, 100, 120, 160, 180, 200, 220, 250, 300, 300, 260, 270, 260, 300 },
    { 60, 80, 100, 100, 120, 160, 180, 200, 220, 250, 300, 300, 280, 290, 300, 350 },
    { 60, 80, 100, 100, 120, 160, 180, 200, 220, 250, 300, 300, 290, 320, 350, 370 },
    { 60, 80, 100, 100, 120, 160, 180, 210, 230, 250, 300, 310, 330, 350, 360, 390 },
    { 60, 80, 100, 100, 120, 160, 180, 210, 240, 260, 300, 310, 340, 360, 370, 400 },
    { 60, 80, 100, 100, 120, 160, 190, 220, 250, 280, 300, 320, 350, 370, 390, 410 }
};

static const float default_injection_phase_lpf[TABLE_ROTATES_MAX] = {
    0.100f, 0.100f, 0.100f, 0.100f, 0.100f, 0.100f, 0.100f, 0.100f,
    0.100f, 0.100f, 0.100f, 0.100f, 0.100f, 0.100f, 0.100f, 0.100f,
};

static const float default_enrichment_by_map_sens[TABLE_PRESSURES_MAX] = {
	0.150f, 0.300f, 0.400f, 0.400f, 0.400f, 0.400f, 0.400f, 0.400f,
	0.400f, 0.400f, 0.400f, 0.400f, 0.400f, 0.400f, 0.400f, 0.400f,

};

static const float default_enrichment_by_map_hpf[TABLE_ROTATES_MAX] = {
    0.400f, 0.400f, 0.400f, 0.370f, 0.340f, 0.310f, 0.290f, 0.270f,
    0.260f, 0.250f, 0.250f, 0.250f, 0.250f, 0.250f, 0.250f, 0.250f,
};

static const float default_enrichment_by_thr_sens[TABLE_THROTTLES_MAX] = {
    0.000f, 0.200f, 0.500f, 0.600f, 0.600f, 0.600f, 0.600f, 0.600f,
    0.600f, 0.600f, 0.600f, 0.600f, 0.600f, 0.600f, 0.600f, 0.600f,
};

static const float default_enrichment_by_thr_hpf[TABLE_ROTATES_MAX] = {
    0.400f, 0.400f, 0.400f, 0.370f, 0.340f, 0.310f, 0.290f, 0.270f,
    0.260f, 0.250f, 0.250f, 0.250f, 0.250f, 0.250f, 0.250f, 0.250f,
};

static const float default_enrichment_temp_mult[TABLE_TEMPERATURES_MAX] = {
    0.50f, 0.45f, 0.40f, 0.35f, 0.30f, 0.25f, 0.20f, 0.15f,
    0.10f, 0.05f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f
};

static const float default_ignition_time_rpm_mult[TABLE_ROTATES_MAX] = {
    4.0f, 1.7f, 1.1f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
};

static const float default_voltages[8] = {
    0.0f, 6.0f, 8.0f, 10.0f, 12.0f, 14.0f, 16.0f, 25.0f
};

static const float default_ignition_time[TABLE_VOLTAGES_MAX] = {
    5700, 4500, 3200, 2500, 2100, 1800, 1600, 1000
};

static const float default_injector_lag[TABLE_VOLTAGES_MAX] = {
    3.33f, 3.33f, 1.41f, 0.86f, 0.58f, 0.38f, 0.26f, 0.03f
};

static const float default_engine_temps[TABLE_TEMPERATURES_MAX] = {
    -20, -10, 0, 10, 20, 30, 40, 50,
    60, 70, 80, 90, 100, 110, 120, 130
};

static const float default_warmup_mixtures[TABLE_TEMPERATURES_MAX] = {
    13.4f, 13.5f, 13.6f, 13.7f, 13.8f, 13.9f, 14.4f, 14.7f,
    14.7f, 14.6f, 14.6f, 14.6f, 14.6f, 14.0f, 13.6f, 13.0f
};

static const float default_warmup_mix_koffs[TABLE_TEMPERATURES_MAX] = {
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.9f, 0.8f, 0.7f,
    0.6f, 0.4f, 0.2f, 0.0f, 0.0f, 0.7f, 1.0f, 1.0f
};

static const float default_warmup_mix_corrs[TABLE_TEMPERATURES_MAX] = {
    0.20f, 0.15f, 0.10f, 0.05f, 0.04f, 0.03f, 0.02f, 0.02f,
    0.01f, 0.01f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f,
};

static const float default_cold_start_idle_corrs[TABLE_TEMPERATURES_MAX] = {
	0.35f, 0.30f, 0.25f, 0.21f, 0.17f, 0.11f, 0.07f, 0.05f,
	0.04f, 0.02f, 0.03f, 0.02f, 0.01f, 0.01f, 0.01f, 0.01f,

};

static const float default_cold_start_idle_times[TABLE_TEMPERATURES_MAX] = {
	40, 35, 30, 27, 25, 18, 9, 4,
	3, 3, 3, 3, 3, 3, 3, 3,

};

static const float default_start_tps_corrs[TABLE_THROTTLES_MAX] = {
    1.00f, 1.00f, 1.00f, 1.00f, 0.95f, 0.90f, 0.85f, 0.80f,
    0.75f, 0.60f, 0.55f, 0.50f, 0.45f, 0.40f, 0.00f, 0.00f
};

static const float default_start_async_filling[TABLE_TEMPERATURES_MAX] = {
    //1600, 1400, 1200, 1000, 900, 800, 750, 700,
    //650, 600, 600, 600, 600, 600, 600, 600
    600, 520, 450, 400, 350, 300, 270, 250,
    220, 200, 200, 200, 200, 200, 200, 200
};

static const float default_start_large_filling[TABLE_TEMPERATURES_MAX] = {
    1400, 1100, 1000, 800, 650, 500, 450, 400,
    360, 350, 350, 350, 350, 350, 350, 350
};

static const float default_start_small_filling[TABLE_TEMPERATURES_MAX] = {
    1200, 1000, 800, 600, 440, 380, 340, 280,
    260, 240, 200, 200, 200, 200, 200, 200
};

static const float default_start_filling_time[TABLE_TEMPERATURES_MAX] = {
    5.0f, 4.5f, 4.0f, 3.5f, 3.0f, 2.5f, 2.0f, 1.5f,
    1.0f, 0.6f, 0.3f, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f
};

static const float default_start_injection_phase[TABLE_TEMPERATURES_MAX] = {
    50, 50, 60, 70, 80, 90, 100, 100,
    100, 100, 100, 100, 100, 100, 100, 100
};

static const float default_start_ignition[TABLE_TEMPERATURES_MAX] = {
    10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f,
    10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f
};

static const float default_start_idle_valve_pos[TABLE_TEMPERATURES_MAX] = {
    140, 140, 140, 130, 120, 110, 90, 70,
    60, 60, 60, 60, 60, 60, 60, 60,
};

static const float default_idle_wish_rotates[TABLE_TEMPERATURES_MAX] = {
    2050, 1970, 1780, 1580, 1460, 1390, 1320, 1240,
    1100, 1000, 860, 860, 860, 1100, 1200, 1250
};

static const float default_idle_valve_to_rpm[TABLE_TEMPERATURES_MAX][TABLE_ROTATES_MAX] = {
	{ 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, },
	{ 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, },
	{ 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, },
	{ 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, },
	{ 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, },
	{ 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, },
	{ 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, },
	{ 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, },
	{ 45, 45, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, },
	{ 45, 45, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, },
	{ 43, 40, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, },
	{ 45, 40, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, },
	{ 45, 40, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, },
	{ 47, 47, 47, 47, 47, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, },
	{ 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 30, 30, 30, },
	{ 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 30, 30, },

};

static const float default_idle_wish_massair[TABLE_TEMPERATURES_MAX] = {
	27.0f, 25.0f, 23.0f, 20.0f, 17.0f, 15.6f, 15.0f, 13.0f,
	11.5f, 11.2f, 9.3f, 9.3f, 9.5f, 11.4f, 12.3f, 13.2f,

};

static const float default_idle_wish_ignition_static[TABLE_ROTATES_MAX] = {
    25.0f, 23.0f, 18.0f, 16.0f, 14.0f, 14.0f, 14.0f, 15.0f,
    16.0f, 18.5f, 21.0f, 24.0f, 27.0f, 30.0f, 33.0f, 35.0f
};

static const float default_idle_wish_ignition[TABLE_TEMPERATURES_MAX] = {
    25.0f, 25.0f, 25.0f, 25.0f, 25.0f, 25.0f, 25.0f, 25.0f,
    18.0f, 15.0f, 15.0f, 15.0f, 15.0f, 18.0f, 19.0f, 20.0f
};

static const float default_idle_rpm_shift_speeds[TABLE_SPEEDS_MAX] = {
    0, 3, 10, 20, 30, 40, 50, 60,
    70, 80, 90, 100, 110, 120, 130, 140
};

static const float default_idle_rpm_shift[TABLE_SPEEDS_MAX] = {
    0, 50, 80, 100, 100, 120, 150, 150,
    150, 150, 180, 200, 200, 200, 200, 200
};

static const float default_knock_noise_level[TABLE_ROTATES_MAX] = {
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f
};

static const float default_knock_threshold[TABLE_ROTATES_MAX] = {
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f
};

static const float default_knock_filter_frequency[TABLE_ROTATES_MAX] = {
    42, 42, 42, 42, 42, 42, 42, 42,
    42, 42, 42, 42, 42, 42, 42, 42,
};

static const float default_knock_gain[TABLE_ROTATES_MAX] = {
    14, 14, 14, 14, 14, 14, 14, 14,
    14, 14, 14, 14, 14, 14, 14, 14
};

static const float default_knock_zone[TABLE_FILLING_MAX][TABLE_ROTATES_MAX] = {
    { 0.20f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f },
    { 0.40f, 0.20f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f },
    { 0.60f, 0.40f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f },
    { 0.60f, 0.50f, 0.40f, 0.40f, 0.40f, 0.40f, 0.50f, 0.60f, 0.60f, 0.60f, 0.60f, 0.60f, 0.50f, 0.50f, 0.50f, 0.50f },
    { 0.60f, 0.60f, 0.60f, 0.60f, 0.60f, 0.60f, 0.70f, 0.70f, 0.70f, 0.70f, 0.70f, 0.70f, 0.70f, 0.70f, 0.70f, 0.70f },
    { 0.70f, 0.80f, 0.80f, 0.80f, 0.80f, 0.80f, 0.80f, 0.80f, 0.80f, 0.80f, 0.80f, 0.80f, 0.80f, 0.80f, 0.80f, 0.80f },
    { 0.80f, 0.90f, 0.90f, 0.90f, 0.90f, 0.90f, 0.90f, 0.90f, 0.90f, 0.90f, 0.90f, 0.90f, 0.90f, 0.90f, 0.90f, 0.90f },
    { 0.90f, 0.95f, 0.95f, 0.95f, 0.95f, 0.95f, 0.95f, 0.95f, 0.95f, 0.95f, 0.95f, 0.95f, 0.95f, 0.95f, 0.95f, 0.95f },
    { 0.95f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f },
    { 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f },
    { 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f },
    { 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f },
    { 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f },
    { 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f },
    { 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f },
    { 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f },
};

static const float default_air_temps[TABLE_TEMPERATURES_MAX] = {
    -40, -30, -20, -10, -5, 0, 5, 10,
    15, 20, 30, 40, 50, 60, 70, 80
};

static const float default_air_temp_mix_corrs[TABLE_TEMPERATURES_MAX][TABLE_ROTATES_MAX] = {
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
};

static const float default_air_temp_ign_corrs[TABLE_TEMPERATURES_MAX][TABLE_ROTATES_MAX] = {
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,-1.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,-1.0f,-2.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,-1.0f,-1.0f,-2.0f,-2.0f,-2.0f,-2.0f,-3.0f,-3.0f,-3.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,-1.0f,-3.0f,-4.0f,-5.0f,-6.0f,-6.0f,-6.0f,-7.0f,-7.0f,-7.0f,-7.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f,-1.0f,-2.0f,-4.0f,-6.0f,-8.0f,-9.0f,-9.0f,-9.0f,-9.0f,-9.0f,-9.0f,-9.0f },
};

static const float default_idle_valve_to_massair_pid_p[TABLE_ROTATES_MAX] = {
    3.000f, 3.000f, 3.000f, 3.000f, 3.000f, 3.000f, 3.000f, 3.000f,
    3.000f, 3.000f, 3.000f, 3.000f, 3.000f, 3.000f, 3.000f, 3.000f
};

static const float default_idle_valve_to_massair_pid_i[TABLE_ROTATES_MAX] = {
    5.000f, 5.000f, 5.000f, 5.000f, 5.000f, 5.000f, 5.000f, 5.000f,
    5.000f, 5.000f, 5.000f, 5.000f, 5.000f, 5.000f, 5.000f, 5.000f
};

static const float default_idle_valve_to_massair_pid_d[TABLE_ROTATES_MAX] = {
    0.060f, 0.060f, 0.050f, 0.040f, 0.030f, 0.020f, 0.010f, 0.005f,
    0.003f, 0.002f, 0.001f, 0.001f, 0.001f, 0.001f, 0.001f, 0.001f
};

static const float default_idle_ign_to_rpm_pid_p[TABLE_ROTATES_MAX] = {
    0.0200f, 0.0200f, 0.0200f, 0.0200f, 0.0200f, 0.0200f, 0.0200f, 0.0200f,
    0.0200f, 0.0200f, 0.0200f, 0.0200f, 0.0200f, 0.0200f, 0.0200f, 0.0200f
};

static const float default_idle_ign_to_rpm_pid_i[TABLE_ROTATES_MAX] = {
    0.0200f, 0.0200f, 0.0200f, 0.0200f, 0.0200f, 0.0200f, 0.0200f, 0.0200f,
    0.0200f, 0.0200f, 0.0200f, 0.0200f, 0.0200f, 0.0200f, 0.0200f, 0.0200f
};

static const float default_idle_ign_to_rpm_pid_d[TABLE_ROTATES_MAX] = {
    0.0001f, 0.0001f, 0.0001f, 0.0001f, 0.0001f, 0.0001f, 0.0001f, 0.0001f,
    0.0001f, 0.0001f, 0.0001f, 0.0001f, 0.0001f, 0.0001f, 0.0001f, 0.0001f
};

static const float default_idle_rpm_pid_act_1[TABLE_TEMPERATURES_MAX] = {
    0.50f, 0.50f, 0.50f, 0.50f, 0.50f, 0.50f, 0.50f, 0.50f,
    0.40f, 0.30f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f
};

static const float default_idle_rpm_pid_act_2[TABLE_TEMPERATURES_MAX] = {
    0.60f, 0.60f, 0.60f, 0.60f, 0.60f, 0.60f, 0.60f, 0.55f,
    0.47f, 0.37f, 0.32f, 0.32f, 0.32f, 0.32f, 0.32f, 0.32f
};

void config_default_table(sEcuTable *table, uint8_t number)
{
  memset(table, 0, sizeof(sEcuTable));

  sprintf(table->name, "Default %d", number + 1);

  table->inj_channel = InjectorChannel1;

  //Bosch 0280156095 - 315 cc/min
  //BMW 03762FA - 180 cc/min
  table->injector_performance = 130.0f;

  table->is_fuel_phase_by_end = 1;
  table->is_fuel_pressure_const = 0;
  table->enrichment_ph_sync_enabled = 1;
  table->enrichment_ph_async_enabled = 1;
  table->enrichment_pp_sync_enabled = 1;
  table->enrichment_pp_async_enabled = 1;
  table->fuel_pressure = 3.0f;
  table->fuel_mass_per_cc = 0.75f;
  table->fuel_afr = 14.7f;

  table->voltages_count = ITEMSOF(default_voltages);
  memcpy(table->voltages, default_voltages, sizeof(default_voltages));

  table->pressures_count = ITEMSOF(default_pressures);
  memcpy(table->pressures, default_pressures, sizeof(default_pressures));

  table->rotates_count = ITEMSOF(default_rotates);
  memcpy(table->rotates, default_rotates, sizeof(default_rotates));

  table->throttles_count = ITEMSOF(default_throttles);
  memcpy(table->throttles, default_throttles, sizeof(default_throttles));

  table->enrichment_proportion_map_vs_thr = 0.9f;
  memcpy(table->fill_by_map, default_filling_by_map, sizeof(default_filling_by_map));
  memcpy(table->map_by_thr, default_map_by_thr, sizeof(default_map_by_thr));

  memcpy(table->enrichment_by_map_sens, default_enrichment_by_map_sens, sizeof(default_enrichment_by_map_sens));
  memcpy(table->enrichment_by_map_hpf, default_enrichment_by_map_hpf, sizeof(default_enrichment_by_map_hpf));
  memcpy(table->enrichment_by_thr_sens, default_enrichment_by_thr_sens, sizeof(default_enrichment_by_thr_sens));
  memcpy(table->enrichment_by_thr_hpf, default_enrichment_by_thr_hpf, sizeof(default_enrichment_by_thr_hpf));
  memcpy(table->enrichment_temp_mult, default_enrichment_temp_mult, sizeof(default_enrichment_temp_mult));

  table->fillings_count = ITEMSOF(default_fillings);
  memcpy(table->fillings, default_fillings, sizeof(default_fillings));
  memcpy(table->ignitions, default_ignitions, sizeof(default_ignitions));
  memcpy(table->fuel_mixtures, default_fuel_mixtures, sizeof(default_fuel_mixtures));
  memcpy(table->injection_phase, default_injection_phase, sizeof(default_injection_phase));
  memcpy(table->injection_phase_lpf, default_injection_phase_lpf, sizeof(default_injection_phase_lpf));

  memcpy(table->ignition_time_rpm_mult, default_ignition_time_rpm_mult, sizeof(default_ignition_time_rpm_mult));
  memcpy(table->ignition_time, default_ignition_time, sizeof(default_ignition_time));
  memcpy(table->injector_lag, default_injector_lag, sizeof(default_injector_lag));

  table->engine_temp_count = ITEMSOF(default_engine_temps);
  memcpy(table->engine_temps, default_engine_temps, sizeof(default_engine_temps));

  table->air_temp_count = ITEMSOF(default_air_temps);
  memcpy(table->air_temps, default_air_temps, sizeof(default_air_temps));
  memcpy(table->air_temp_mix_corr, default_air_temp_mix_corrs, sizeof(default_air_temp_mix_corrs));
  memcpy(table->air_temp_ign_corr, default_air_temp_ign_corrs, sizeof(default_air_temp_ign_corrs));

  memcpy(table->idle_wish_rotates, default_idle_wish_rotates, sizeof(default_idle_wish_rotates));
  memcpy(table->idle_wish_massair, default_idle_wish_massair, sizeof(default_idle_wish_massair));
  memcpy(table->idle_wish_ignition_static, default_idle_wish_ignition_static, sizeof(default_idle_wish_ignition_static));
  memcpy(table->idle_wish_ignition, default_idle_wish_ignition, sizeof(default_idle_wish_ignition));
  memcpy(table->idle_valve_to_rpm, default_idle_valve_to_rpm, sizeof(default_idle_valve_to_rpm));

  memcpy(table->warmup_mixtures, default_warmup_mixtures, sizeof(default_warmup_mixtures));
  memcpy(table->warmup_mix_koffs, default_warmup_mix_koffs, sizeof(default_warmup_mix_koffs));
  memcpy(table->warmup_mix_corrs, default_warmup_mix_corrs, sizeof(default_warmup_mix_corrs));

  memcpy(table->cold_start_idle_corrs, default_cold_start_idle_corrs, sizeof(default_cold_start_idle_corrs));
  memcpy(table->cold_start_idle_times, default_cold_start_idle_times, sizeof(default_cold_start_idle_times));
  memcpy(table->start_tps_corrs, default_start_tps_corrs, sizeof(default_start_tps_corrs));
  memcpy(table->start_async_filling, default_start_async_filling, sizeof(default_start_async_filling));
  memcpy(table->start_large_filling, default_start_large_filling, sizeof(default_start_large_filling));
  memcpy(table->start_small_filling, default_start_small_filling, sizeof(default_start_small_filling));
  memcpy(table->start_filling_time, default_start_filling_time, sizeof(default_start_filling_time));
  memcpy(table->start_ignition, default_start_ignition, sizeof(default_start_ignition));
  memcpy(table->start_injection_phase, default_start_injection_phase, sizeof(default_start_injection_phase));
  memcpy(table->start_idle_valve_pos, default_start_idle_valve_pos, sizeof(default_start_idle_valve_pos));
  table->start_large_count = 12;

  memcpy(table->idle_valve_to_massair_pid_p, default_idle_valve_to_massair_pid_p, sizeof(default_idle_valve_to_massair_pid_p));
  memcpy(table->idle_valve_to_massair_pid_i, default_idle_valve_to_massair_pid_i, sizeof(default_idle_valve_to_massair_pid_i));
  memcpy(table->idle_valve_to_massair_pid_d, default_idle_valve_to_massair_pid_d, sizeof(default_idle_valve_to_massair_pid_d));

  memcpy(table->idle_ign_to_rpm_pid_p, default_idle_ign_to_rpm_pid_p, sizeof(default_idle_ign_to_rpm_pid_p));
  memcpy(table->idle_ign_to_rpm_pid_i, default_idle_ign_to_rpm_pid_i, sizeof(default_idle_ign_to_rpm_pid_i));
  memcpy(table->idle_ign_to_rpm_pid_d, default_idle_ign_to_rpm_pid_d, sizeof(default_idle_ign_to_rpm_pid_d));

  memcpy(table->idle_rpm_pid_act_1, default_idle_rpm_pid_act_1, sizeof(default_idle_rpm_pid_act_1));
  memcpy(table->idle_rpm_pid_act_2, default_idle_rpm_pid_act_2, sizeof(default_idle_rpm_pid_act_2));

  table->short_term_corr_pid_p = 0.0000f;
  table->short_term_corr_pid_i = 0.0000f;
  table->short_term_corr_pid_d = 0.0000f;

  table->idle_ign_deviation_min = -14.0f;
  table->idle_ign_deviation_max = 14.0f;

  table->idle_ign_fan_low_corr = 5.0f;
  table->idle_ign_fan_high_corr = 10.0f;

  table->idle_speeds_shift_count = ITEMSOF(default_idle_rpm_shift_speeds);
  memcpy(table->idle_rpm_shift_speeds, default_idle_rpm_shift_speeds, sizeof(default_idle_rpm_shift_speeds));
  memcpy(table->idle_rpm_shift, default_idle_rpm_shift, sizeof(default_idle_rpm_shift));

  table->knock_ign_corr_max = -10.0f;
  table->knock_inj_corr_max = 0.07f;
  memcpy(table->knock_noise_level, default_knock_noise_level, sizeof(default_knock_noise_level));
  memcpy(table->knock_threshold, default_knock_threshold, sizeof(default_knock_threshold));
  memcpy(table->knock_zone, default_knock_zone, sizeof(default_knock_zone));
  memcpy(table->knock_gain, default_knock_gain, sizeof(default_knock_gain));
  memcpy(table->knock_filter_frequency, default_knock_filter_frequency, sizeof(default_knock_filter_frequency));

  for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
    table->cy_corr_injection[i] = 0;
    table->cy_corr_ignition[i] = 0;
  }
}

void config_default_params(sEcuParams *table)
{
  table->engineVolume = 1550.0f;

  table->isSwitchByExternal = 0;
  table->startupTableNumber = 0;
  table->switchPos1Table = 0;
  table->switchPos0Table = 0;
  table->switchPos2Table = 0;

  table->cutoffRPM = 5500;
  table->cutoffMode = 5;
  table->cutoffAngle = 3.0f;
  table->cutoffMixture = 12.1f;
  table->oilPressureCutoffRPM = 2000;
  table->speedInputCorrection = 1.0f;
  table->speedOutputCorrection = 1.0f;

  table->useLambdaSensor = 1;
  table->useTSPS = 1;
  table->useKnockSensor = 1;
  table->performAdaptation = 0;
  table->isSingleCoil = 0;
  table->isIndividualCoils = 0;
  table->isEconEnabled = 0;

  table->fanHighTemperature = 96;
  table->fanMidTemperature = 93;
  table->fanLowTemperature = 90;

  table->isBluetoothEnabled = 1;
  table->bluetoothPin = 1902;
  strcpy(table->bluetoothName, "BT_ECU");

  table->shiftMode = 0;
  table->shiftThrThr = 90;
  table->shiftRpmThr = 5000;
  table->shiftRpmTill = 2500;
  table->shiftAngle = 3.0f;
  table->shiftMixture = 12.1f;

  table->tspsRelPos = -81.6f;
  table->tspsDesyncThr = 3.0f;

  table->useShortTermCorr = 0;
  table->useLongTermCorr = 0;

  table->knockIntegratorTime = 22;

}

void config_default_corrections(sEcuCorrections *table)
{
  table->long_term_correction = 0;
  table->idle_correction = 0;

  for(int i = 0; i < TABLE_FILLING_MAX; i++)
    for(int j = 0; j < TABLE_ROTATES_MAX; j++)
      table->ignitions[i][j] = 0;

  for(int i = 0; i < TABLE_PRESSURES_MAX; i++)
    for(int j = 0; j < TABLE_ROTATES_MAX; j++)
      table->fill_by_map[i][j] = 0.0f;

  for(int i = 0; i < TABLE_THROTTLES_MAX; i++)
    for(int j = 0; j < TABLE_ROTATES_MAX; j++)
      table->map_by_thr[i][j] = 0.0f;

  for(int i = 0; i < TABLE_TEMPERATURES_MAX; i++)
    for(int j = 0; j < TABLE_ROTATES_MAX; j++)
      table->idle_valve_to_rpm[i][j] = 0.0f;
}

void config_default_critical_backup(sEcuCriticalBackup *table)
{
  table->km_driven = 0;
  table->fuel_consumed = 0;
  table->idle_valve_position = 0;
}


HAL_StatusTypeDef config_init(void)
{
  HAL_StatusTypeDef status = HAL_OK;
  status = flash_checkchip();
  return status;
}


int8_t config_load_table(sEcuTable *table, uint8_t number)
{
  if(number >= TABLE_SETUPS_MAX)
    return -1;
  return flash_page_load(table, sizeof(sEcuTable), number);
}

int8_t config_save_table(const sEcuTable *table, uint8_t number)
{
  if(number >= TABLE_SETUPS_MAX)
    return -1;
  return flash_page_save(table, sizeof(sEcuTable), number);
}


int8_t config_load_params(sEcuParams *params)
{
  return flash_page_load(params, sizeof(sEcuParams), TABLE_SETUPS_MAX);
}

int8_t config_save_params(const sEcuParams *params)
{
  return flash_page_save(params, sizeof(sEcuParams), TABLE_SETUPS_MAX);
}

static int8_t corr_to_backup(sEcuCorrectionsBackup *backup, const sEcuCorrections *corr)
{
  static uint8_t state = 0;

  if(state == 0) {
    backup->long_term_correction = corr->long_term_correction;
    backup->idle_correction = corr->idle_correction;
    for(int i = 0; i < TABLE_FILLING_MAX; i++) {
      for(int j = 0; j < TABLE_ROTATES_MAX; j++) {
        backup->ignitions[i][j] = CLAMP(roundf(corr->ignitions[i][j] * 5.0f), -128, 127);
      }
    }
    state++;
  } else if(state == 1) {
    for(int i = 0; i < TABLE_PRESSURES_MAX; i++) {
      for(int j = 0; j < TABLE_ROTATES_MAX; j++) {
        backup->fill_by_map[i][j] = CLAMP(roundf(corr->fill_by_map[i][j] * 125.0f), -128, 127);
      }
    }
    state++;
  } else if(state == 2) {
    for(int i = 0; i < TABLE_THROTTLES_MAX; i++) {
      for(int j = 0; j < TABLE_ROTATES_MAX; j++) {
        backup->map_by_thr[i][j] = CLAMP(roundf(corr->map_by_thr[i][j] * 125.0f), -128, 127);
      }
    }
    state++;
  } else if(state == 3) {
    for(int i = 0; i < TABLE_TEMPERATURES_MAX; i++) {
      for(int j = 0; j < TABLE_ROTATES_MAX; j++) {
        backup->idle_valve_to_rpm[i][j] = CLAMP(roundf(corr->idle_valve_to_rpm[i][j] * 125.0f), -128, 127);
      }
    }
    state = 0;
    return 1;
  } else state = 0;

  return 0;
}

static int8_t backup_to_corr(sEcuCorrections *corr, const sEcuCorrectionsBackup *backup)
{
  static uint8_t state = 0;

  if(state == 0) {
    corr->long_term_correction = backup->long_term_correction;
    corr->idle_correction = backup->idle_correction;
    for(int i = 0; i < TABLE_FILLING_MAX; i++) {
      for(int j = 0; j < TABLE_ROTATES_MAX; j++) {
        corr->ignitions[i][j] = (float)backup->ignitions[i][j] * 0.2f;
      }
    }
    state++;
  } else if(state == 1) {
    for(int i = 0; i < TABLE_PRESSURES_MAX; i++) {
      for(int j = 0; j < TABLE_ROTATES_MAX; j++) {
        corr->fill_by_map[i][j] = (float)backup->fill_by_map[i][j] * 0.008f;
      }
    }
    state++;
  } else if(state == 2) {
    for(int i = 0; i < TABLE_THROTTLES_MAX; i++) {
      for(int j = 0; j < TABLE_ROTATES_MAX; j++) {
        corr->map_by_thr[i][j] = (float)backup->map_by_thr[i][j] * 0.008f;
      }
    }
    state++;
  } else if(state == 3) {
    for(int i = 0; i < TABLE_TEMPERATURES_MAX; i++) {
      for(int j = 0; j < TABLE_ROTATES_MAX; j++) {
        corr->idle_valve_to_rpm[i][j] = (float)backup->idle_valve_to_rpm[i][j] * 0.008f;
      }
    }
    state = 0;
    return 1;
  } else state = 0;

  return 0;
}

int8_t config_load_corrections(sEcuCorrections *table)
{
  static uint8_t state = 0;
  int8_t status;

  if(state == 0) {
    status = flash_bkpsram_load(&tableBackupCorrections, sizeof(sEcuCorrectionsBackup), CONFIG_OFFSET_CORRECTIONS);
    if(status) {
      if(status > 0) {
        state++;
      } else {
        state = 0;
        return -1;
      }
    }
  } else if(state == 1) {
    status = backup_to_corr(table, &tableBackupCorrections);
    if(status) {
      state = 0;
      return 1;
    }
  }
  return 0;
}

int8_t config_save_corrections(const sEcuCorrections *table)
{
  static uint8_t state = 0;
  int8_t status;

  if(state == 0) {
    status = corr_to_backup(&tableBackupCorrections, table);
    if(status) {
      state++;
    }
  } else if(state == 1) {
    status = flash_bkpsram_save(&tableBackupCorrections, sizeof(sEcuCorrectionsBackup), CONFIG_OFFSET_CORRECTIONS);
    if(status) {
      state = 0;
      if(status > 0) {
        return 1;
      } else {
        return -1;
      }
    }
  }
  return 0;

}


int8_t config_load_critical_backup(sEcuCriticalBackup *table)
{
  return flash_bkpsram_load(table, sizeof(sEcuCriticalBackup), CONFIG_OFFSET_CRITICALS);
}

int8_t config_save_critical_backup(const sEcuCriticalBackup *table)
{
  return flash_bkpsram_save(table, sizeof(sEcuCriticalBackup), CONFIG_OFFSET_CRITICALS);
}


int8_t config_load_all(sEcuParams *params, sEcuTable *tables, uint32_t tables_count)
{
  static uint8_t state = 0;
  static int8_t current_table = 0;
  static int8_t tables_left = 0;
  int8_t status;

  if(!tables_count)
	  return -1;

  switch(state) {
    case 0 :
      status = config_load_params(params);
      if(status) {
        if(status > 0) {
          current_table = 0;
          tables_left = tables_count;
          state++;
        } else {
          state = 0;
          return -1;
        }
      }
      break;
    case 1 :
      status = config_load_table(&tables[current_table], current_table);
      if(status) {
        if(status > 0) {
          tables_left--;
          if(tables_left) {
            current_table++;
          } else {
            current_table = 0;
            state = 0;
            return 1;
          }
        } else {
          state = 0;
          current_table = 0;
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

int8_t config_save_all(const sEcuParams *params, const sEcuTable *tables, uint32_t tables_count)
{
  static uint8_t state = 0;
  static int8_t current_table = 0;
  static int8_t tables_left = 0;

  if(!tables_count)
	  return -1;

  int8_t status;
  switch(state) {
    case 0 :
      status = config_save_params(params);
      if(status) {
        if(status > 0) {
          current_table = 0;
          tables_left = tables_count;
          state++;
        } else {
          state = 0;
          return -1;
        }
      }
      break;
    case 1 :
      status = config_save_table(&tables[current_table], current_table);
      if(status) {
        if(status > 0) {
          tables_left--;
          if(tables_left) {
            current_table++;
          } else {
            current_table = 0;
            state = 0;
            return 1;
          }
        } else {
          state = 0;
          current_table = 0;
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
