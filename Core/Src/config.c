/*
 * config.c
 *
 *  Created on: 7 мар. 2022 г.
 *      Author: VHEMaster
 */

#include "config.h"
#include "structs.h"
#include "flash.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#define CONFIG_OFFSET_CORRECTIONS 0
#define CONFIG_OFFSET_CRITICALS 1920

#define CLAMP(val,min,max) ((val) < (min) ? (min) : (val) > (max) ? (max) : (val))
static sEcuCorrectionsBackup tableBackupCorrections = {0};

static const float default_pressures[TABLE_PRESSURES_MAX] = {
    10000, 16200, 22400, 28600, 34800, 41000, 47200, 53400,
    59600, 65800, 72000, 78200, 84400, 90600, 96800, 103000
};

static const float default_rotates[TABLE_ROTATES_MAX] = {
    600, 720, 840, 990, 1170, 1380, 1650, 1950,
    2310, 2730, 3210, 3840, 4530, 5370, 6360, 7500
};

static const float default_throttles[TABLE_THROTTLES_MAX] = {
    0.0f, 6.67f, 13.33f, 20.0f, 26.67f, 33.33f, 40.00f, 46.67f,
    53.33f, 60.00f, 66.67f, 73.33f, 80.00f, 86.67f, 93.33f, 100.0f
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
    { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f }
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
    { 13.8f, 13.8f, 15.1f, 15.1f, 15.1f, 15.1f, 15.1f, 15.0f, 14.9f, 14.8f, 14.6f, 14.5f, 14.5f, 14.5f, 14.5f, 14.5f },
    { 13.8f, 13.8f, 15.0f, 15.0f, 15.0f, 15.0f, 15.0f, 14.9f, 14.8f, 14.7f, 14.5f, 14.4f, 14.4f, 14.4f, 14.4f, 14.4f },
    { 13.7f, 13.7f, 14.8f, 14.8f, 14.8f, 14.8f, 14.8f, 14.7f, 14.6f, 14.5f, 14.3f, 14.2f, 14.2f, 14.2f, 14.2f, 14.2f },
    { 13.7f, 13.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.6f, 14.5f, 14.4f, 14.2f, 14.1f, 14.1f, 14.1f, 14.1f, 14.1f },
    { 13.6f, 13.6f, 14.5f, 14.5f, 14.5f, 14.5f, 14.5f, 14.4f, 14.3f, 14.2f, 14.0f, 13.9f, 13.9f, 13.9f, 13.9f, 13.9f },
    { 13.4f, 13.4f, 14.2f, 14.2f, 14.2f, 14.2f, 14.2f, 14.1f, 14.0f, 13.9f, 13.7f, 13.6f, 13.6f, 13.6f, 13.6f, 13.6f },
    { 13.3f, 13.3f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 13.9f, 13.8f, 13.7f, 13.5f, 13.4f, 13.4f, 13.4f, 13.4f, 13.4f },
    { 13.3f, 13.3f, 13.9f, 13.9f, 13.9f, 13.9f, 13.9f, 13.8f, 13.7f, 13.6f, 13.4f, 13.3f, 13.3f, 13.3f, 13.3f, 13.3f },
    { 13.3f, 13.3f, 13.8f, 13.8f, 13.8f, 13.8f, 13.8f, 13.7f, 13.6f, 13.5f, 13.3f, 13.2f, 13.2f, 13.2f, 13.3f, 13.3f },
    { 13.3f, 13.3f, 13.7f, 13.7f, 13.7f, 13.7f, 13.7f, 13.6f, 13.5f, 13.4f, 13.3f, 13.2f, 13.2f, 13.2f, 13.2f, 13.2f },
    { 13.3f, 13.3f, 13.6f, 13.6f, 13.6f, 13.6f, 13.6f, 13.5f, 13.4f, 13.3f, 13.2f, 13.1f, 13.1f, 13.1f, 13.1f, 13.0f },
    { 13.2f, 13.2f, 13.4f, 13.4f, 13.4f, 13.4f, 13.4f, 13.3f, 13.2f, 13.1f, 13.0f, 12.9f, 12.9f, 12.9f, 12.9f, 12.9f },
    { 13.1f, 13.1f, 13.2f, 13.2f, 13.2f, 13.2f, 13.2f, 13.1f, 13.0f, 12.9f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f },
    { 13.0f, 13.0f, 13.0f, 13.0f, 13.0f, 12.9f, 12.9f, 12.8f, 12.7f, 12.6f, 12.6f, 12.6f, 12.6f, 12.6f, 12.6f, 12.6f },
    { 12.9f, 12.9f, 12.8f, 12.8f, 12.8f, 12.7f, 12.7f, 12.6f, 12.6f, 12.6f, 12.6f, 12.5f, 12.5f, 12.5f, 12.5f, 12.5f },
    { 12.8f, 12.8f, 12.6f, 12.6f, 12.6f, 12.6f, 12.6f, 12.5f, 12.5f, 12.5f, 12.5f, 12.5f, 12.5f, 12.5f, 12.5f, 12.5f }
};

static const float default_injection_phase[TABLE_FILLING_MAX][TABLE_ROTATES_MAX] = {
    { 250, 250, 250, 250, 260, 270, 270, 270, 280, 290, 300, 300, 250, 200, 150, 90 },
    { 250, 250, 250, 250, 260, 280, 280, 280, 290, 290, 300, 300, 280, 240, 150, 110 },
    { 250, 250, 250, 250, 260, 270, 280, 290, 300, 310, 330, 320, 290, 250, 160, 120 },
    { 250, 250, 250, 250, 260, 270, 280, 290, 310, 330, 360, 340, 300, 260, 170, 130 },
    { 260, 270, 270, 270, 270, 280, 290, 300, 320, 350, 380, 360, 310, 270, 200, 140 },
    { 270, 280, 280, 290, 280, 290, 300, 300, 320, 350, 400, 380, 320, 280, 210, 160 },
    { 280, 285, 290, 290, 290, 300, 300, 300, 320, 350, 400, 400, 330, 290, 220, 190 },
    { 285, 285, 300, 300, 300, 300, 300, 300, 320, 350, 400, 400, 340, 300, 250, 230 },
    { 285, 285, 300, 300, 300, 300, 300, 300, 320, 350, 400, 400, 350, 320, 300, 270 },
    { 285, 285, 300, 300, 300, 300, 300, 300, 320, 350, 400, 400, 350, 340, 320, 310 },
    { 285, 285, 300, 300, 300, 300, 300, 300, 320, 350, 400, 400, 360, 370, 360, 400 },
    { 285, 285, 300, 300, 300, 300, 300, 300, 320, 350, 400, 400, 380, 390, 400, 450 },
    { 285, 285, 300, 300, 300, 300, 300, 300, 320, 350, 400, 400, 390, 420, 450, 500 },
    { 285, 285, 300, 300, 300, 300, 300, 310, 330, 350, 400, 410, 430, 460, 510, 530 },
    { 285, 285, 300, 300, 300, 300, 300, 310, 340, 360, 400, 410, 440, 470, 520, 550 },
    { 285, 285, 300, 300, 300, 300, 300, 320, 350, 380, 400, 420, 460, 480, 530, 560 }
};

static const float default_enrichment_by_map_sens[TABLE_PRESSURES_MAX] = {
    0.000f, 0.013f, 0.030f, 0.047f, 0.058f, 0.071f, 0.087f, 0.100f,
    0.113f, 0.125f, 0.138f, 0.150f, 0.163f, 0.175f, 0.188f, 0.200f
};

static const float default_enrichment_by_map_hpf[TABLE_ROTATES_MAX] = {
    0.116f, 0.113f, 0.109f, 0.105f, 0.101f, 0.098f, 0.094f, 0.090f,
    0.086f, 0.083f, 0.079f, 0.075f, 0.071f, 0.068f, 0.064f, 0.060f,
};

static const float default_enrichment_by_thr_sens[TABLE_THROTTLES_MAX] = {
    0.000f, 0.013f, 0.030f, 0.047f, 0.058f, 0.071f, 0.087f, 0.100f,
    0.113f, 0.125f, 0.138f, 0.150f, 0.163f, 0.175f, 0.188f, 0.200f
};

static const float default_enrichment_by_thr_hpf[TABLE_ROTATES_MAX] = {
    0.116f, 0.113f, 0.109f, 0.105f, 0.101f, 0.098f, 0.094f, 0.090f,
    0.086f, 0.083f, 0.079f, 0.075f, 0.071f, 0.068f, 0.064f, 0.060f,
};

static const float default_ignition_time_rpm_mult[TABLE_ROTATES_MAX] = {
    4.0f, 1.7f, 1.4f, 1.3f, 1.2f, 1.1f, 1.0f, 1.0f,
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
    13.0f, 13.0f, 13.1f, 13.2f, 13.3f, 13.5f, 13.8f, 14.1f,
    14.3f, 14.4f, 14.5f, 14.6f, 14.7f, 14.7f, 14.7f, 14.7f
};

static const float default_warmup_mix_koffs[TABLE_TEMPERATURES_MAX] = {
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.9f, 0.8f, 0.7f,
    0.6f, 0.4f, 0.2f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

static const float default_warmup_mix_corrs[TABLE_TEMPERATURES_MAX] = {
    6.0f, 5.4f, 4.5f, 3.2f, 1.9f, 0.9f, 0.3f, 0.1f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

static const float default_start_mixtures[TABLE_TEMPERATURES_MAX] = {
    9.0f, 9.1f, 9.3, 9.6f, 10.0f, 10.5f, 11.1f, 11.8f,
    12.0f, 12.0f, 12.0f, 12.0f, 12.0f, 12.0f, 12.0f, 12.0f
};

static const float default_idle_wish_rotates[TABLE_TEMPERATURES_MAX] = {
    2050, 1970, 1780, 1580, 1460, 1390, 1320, 1240,
    1090, 1000, 1000, 1000, 1060, 1200, 1300, 1350
};

static const float default_idle_valve_to_rpm[TABLE_TEMPERATURES_MAX][TABLE_ROTATES_MAX] = {
    { 60.0f, 60.0f, 60.0f, 60.0f, 60.0f, 60.0f, 60.0f, 60.0f, 60.0f, 60.0f, 60.0f, 60.0f, 60.0f, 60.0f, 60.0f, 60.0f },
    { 60.0f, 60.0f, 60.0f, 60.0f, 60.0f, 60.0f, 60.0f, 60.0f, 60.0f, 60.0f, 60.0f, 60.0f, 60.0f, 60.0f, 60.0f, 60.0f },
    { 55.0f, 55.0f, 55.0f, 55.0f, 55.0f, 55.0f, 55.0f, 55.0f, 55.0f, 55.0f, 55.0f, 55.0f, 55.0f, 55.0f, 55.0f, 55.0f },
    { 50.0f, 50.0f, 50.0f, 50.0f, 50.0f, 50.0f, 50.0f, 50.0f, 50.0f, 50.0f, 50.0f, 50.0f, 50.0f, 50.0f, 50.0f, 50.0f },
    { 47.0f, 47.0f, 47.0f, 47.0f, 47.0f, 47.0f, 47.0f, 47.0f, 47.0f, 47.0f, 47.0f, 47.0f, 47.0f, 47.0f, 47.0f, 47.0f },
    { 42.0f, 42.0f, 42.0f, 42.0f, 42.0f, 42.0f, 42.0f, 42.0f, 42.0f, 42.0f, 42.0f, 42.0f, 42.0f, 42.0f, 42.0f, 42.0f },
    { 40.0f, 40.0f, 40.0f, 40.0f, 40.0f, 40.0f, 40.0f, 40.0f, 40.0f, 40.0f, 40.0f, 40.0f, 40.0f, 40.0f, 40.0f, 40.0f },
    { 43.0f, 43.0f, 43.0f, 43.0f, 43.0f, 43.0f, 43.0f, 43.0f, 43.0f, 43.0f, 43.0f, 43.0f, 43.0f, 43.0f, 43.0f, 43.0f },
    { 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f },
    { 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f },
    { 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f },
    { 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f },
    { 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f },
    { 47.0f, 47.0f, 47.0f, 47.0f, 47.0f, 47.0f, 47.0f, 47.0f, 47.0f, 47.0f, 47.0f, 47.0f, 47.0f, 47.0f, 47.0f, 47.0f },
    { 49.0f, 49.0f, 49.0f, 49.0f, 49.0f, 49.0f, 49.0f, 49.0f, 49.0f, 49.0f, 49.0f, 49.0f, 49.0f, 49.0f, 49.0f, 49.0f },
    { 50.0f, 50.0f, 50.0f, 50.0f, 50.0f, 50.0f, 50.0f, 50.0f, 50.0f, 50.0f, 50.0f, 50.0f, 50.0f, 50.0f, 50.0f, 50.0f }
};

static const float default_idle_wish_massair[TABLE_TEMPERATURES_MAX] = {
    20.0f, 18.0f, 17.0f, 15.0f, 14.2, 14.0f, 14.0f, 12.7f,
    11.0f, 11.0f, 11.0f, 11.0f, 11.0f, 11.2f, 11.8f, 12.0f
};

static const float default_idle_wish_ignition[TABLE_ROTATES_MAX] = {
    15.0f, 21.0f, 17.0f, 12.0f, 11.0f, 11.0f, 12.0f, 14.0f,
    16.0f, 18.5f, 21.0f, 24.0f, 27.0f, 30.0f, 33.0f, 35.0f
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
    1.0f, 1.0f, 1.0f, 1.1f, 1.1f, 1.1f, 1.2f, 1.2f

};

static const float default_knock_threshold[TABLE_ROTATES_MAX] = {
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f
};

static const float default_knock_filter_frequency[TABLE_ROTATES_MAX] = {
    42, 42, 42, 42, 42, 42, 42, 42,
    42, 42, 42, 42, 42, 42, 42, 42,
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
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
};

static const float default_ignition_initial[TABLE_TEMPERATURES_MAX] = {
    10, 10, 10, 10, 10, 10, 10, 10,
    10, 10, 10, 10, 10, 10, 10, 10
};

static const float default_idle_valve_initial[TABLE_TEMPERATURES_MAX] = {
    70, 65, 63, 62, 61, 60, 59, 58,
    55, 55, 55, 55, 55, 55, 55, 55
};

void config_default_table(sEcuTable *table, uint8_t number)
{
  memset(table, 0, sizeof(sEcuTable));

  sprintf(table->name, "Default %d", number + 1);

  table->inj_channel = InjectorChannel1;

  table->injector_performance = 180.0f;
  table->is_fuel_phase_by_end = 1;
  table->is_fuel_pressure_const = 0;
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

  table->enrichment_proportion_map_vs_thr = 0.1f;
  memcpy(table->fill_by_map, default_filling_by_map, sizeof(default_filling_by_map));
  memcpy(table->map_by_thr, default_map_by_thr, sizeof(default_map_by_thr));

  memcpy(table->enrichment_by_map_sens, default_enrichment_by_map_sens, sizeof(default_enrichment_by_map_sens));
  memcpy(table->enrichment_by_map_hpf, default_enrichment_by_map_hpf, sizeof(default_enrichment_by_map_hpf));
  memcpy(table->enrichment_by_thr_sens, default_enrichment_by_thr_sens, sizeof(default_enrichment_by_thr_sens));
  memcpy(table->enrichment_by_thr_hpf, default_enrichment_by_thr_hpf, sizeof(default_enrichment_by_thr_hpf));

  table->fillings_count = ITEMSOF(default_fillings);
  memcpy(table->fillings, default_fillings, sizeof(default_fillings));
  memcpy(table->ignitions, default_ignitions, sizeof(default_ignitions));
  memcpy(table->fuel_mixtures, default_fuel_mixtures, sizeof(default_fuel_mixtures));
  memcpy(table->injection_phase, default_injection_phase, sizeof(default_injection_phase));

  memcpy(table->ignition_time_rpm_mult, default_ignition_time_rpm_mult, sizeof(default_ignition_time_rpm_mult));
  memcpy(table->ignition_time, default_ignition_time, sizeof(default_ignition_time));
  memcpy(table->injector_lag, default_injector_lag, sizeof(default_injector_lag));

  table->engine_temp_count = ITEMSOF(default_engine_temps);
  memcpy(table->engine_temps, default_engine_temps, sizeof(default_engine_temps));

  memcpy(table->ignition_initial, default_ignition_initial, sizeof(default_ignition_initial));
  memcpy(table->idle_valve_initial, default_idle_valve_initial, sizeof(default_idle_valve_initial));

  table->air_temp_count = ITEMSOF(default_air_temps);
  memcpy(table->air_temps, default_air_temps, sizeof(default_air_temps));
  memcpy(table->air_temp_mix_corr, default_air_temp_mix_corrs, sizeof(default_air_temp_mix_corrs));
  memcpy(table->air_temp_ign_corr, default_air_temp_ign_corrs, sizeof(default_air_temp_ign_corrs));

  memcpy(table->idle_wish_rotates, default_idle_wish_rotates, sizeof(default_idle_wish_rotates));
  memcpy(table->idle_wish_massair, default_idle_wish_massair, sizeof(default_idle_wish_massair));
  memcpy(table->idle_wish_ignition, default_idle_wish_ignition, sizeof(default_idle_wish_ignition));
  memcpy(table->idle_valve_to_rpm, default_idle_valve_to_rpm, sizeof(default_idle_valve_to_rpm));

  memcpy(table->warmup_mixtures, default_warmup_mixtures, sizeof(default_warmup_mixtures));
  memcpy(table->warmup_mix_koffs, default_warmup_mix_koffs, sizeof(default_warmup_mix_koffs));
  memcpy(table->warmup_mix_corrs, default_warmup_mix_corrs, sizeof(default_warmup_mix_corrs));
  memcpy(table->start_mixtures, default_start_mixtures, sizeof(default_start_mixtures));

  table->idle_valve_to_massair_pid_p = 3.000f;
  table->idle_valve_to_massair_pid_i = 5.000f;
  table->idle_valve_to_massair_pid_d = 0.001f;

  table->idle_ign_to_rpm_pid_p = 0.0200f;
  table->idle_ign_to_rpm_pid_i = 0.0200f;
  table->idle_ign_to_rpm_pid_d = 0.0001f;

  table->short_term_corr_pid_p = 0.0000f;
  table->short_term_corr_pid_i = 0.0000f;
  table->short_term_corr_pid_d = 0.0000f;

  table->idle_ign_deviation_min = -14.0f;
  table->idle_ign_deviation_max = 14.0f;

  table->idle_ign_fan_corr = 10.0f;
  table->idle_rpm_pid_act = 0.6f;

  table->idle_speeds_shift_count = ITEMSOF(default_idle_rpm_shift_speeds);
  memcpy(table->idle_rpm_shift_speeds, default_idle_rpm_shift_speeds, sizeof(default_idle_rpm_shift_speeds));
  memcpy(table->idle_rpm_shift, default_idle_rpm_shift, sizeof(default_idle_rpm_shift));

  table->knock_ign_corr_max = -10.0f;
  table->knock_inj_corr_max = 0.07f;
  memcpy(table->knock_noise_level, default_knock_noise_level, sizeof(default_knock_noise_level));
  memcpy(table->knock_threshold, default_knock_threshold, sizeof(default_knock_threshold));
  memcpy(table->knock_zone, default_knock_zone, sizeof(default_knock_zone));
  memcpy(table->knock_filter_frequency, default_knock_filter_frequency, sizeof(default_knock_filter_frequency));

  for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
    table->cy_corr_injection[i] = 0;
    table->cy_corr_ignition[i] = 0;
  }
}

void config_default_params(sEcuParams *table)
{
  table->engineVolume = 1551.0f;

  table->isSwitchByExternal = 0;
  table->startupTableNumber = 0;
  table->switchPos1Table = 0;
  table->switchPos0Table = 0;
  table->switchPos2Table = 0;

  table->cutoffRPM = 6200;
  table->cutoffMode = 5;
  table->cutoffAngle = 5.0f;
  table->cutoffMixture = 12.1f;
  table->oilPressureCutoffRPM = 1500;
  table->speedCorrection = 1.16f;

  table->useLambdaSensor = 1;
  table->useTSPS = 1;
  table->useKnockSensor = 1;
  table->performAdaptation = 0;
  table->isSingleCoil = 0;
  table->isIndividualCoils = 1;

  table->fanHighTemperature = 101;
  table->fanLowTemperature = 97;

  table->isBluetoothEnabled = 1;
  table->bluetoothPin = 1902;
  strcpy(table->bluetoothName, "BT_ECU");

  table->shiftMode = 0;
  table->shiftThrThr = 90;
  table->shiftRpmThr = 5000;
  table->shiftRpmTill = 2500;
  table->shiftAngle = 5.0f;
  table->shiftMixture = 12.1f;

  table->tspsRelPos = -154.0f;
  table->tspsDesyncThr = 3.0f;

  table->useShortTermCorr = 1;
  table->useLongTermCorr = 1;

  table->knockGain = 14;
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
  int8_t status;
  switch(state) {
    case 0 :
      status = config_load_params(params);
      if(status) {
        if(status > 0) {
          current_table = 0;
          state++;
        } else {
          state = 0;
          return -1;
        }
      }
      break;
    case 1 :
      status = config_load_table(tables, current_table);
      if(status) {
        if(status > 0) {
          if(tables_count) {
            current_table++;
            tables_count--;
            tables++;
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
  int8_t status;
  switch(state) {
    case 0 :
      status = config_save_params(params);
      if(status) {
        if(status > 0) {
          current_table = 0;
          state++;
        } else {
          state = 0;
          return -1;
        }
      }
      break;
    case 1 :
      status = config_save_table(tables, current_table);
      if(status) {
        if(status > 0) {
          if(tables_count) {
            current_table++;
            tables_count--;
            tables++;
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
