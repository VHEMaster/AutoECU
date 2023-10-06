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

#define CONFIG_OFFSET_TABLES (0)
#define CONFIG_OFFSET_PARAMS (CONFIG_OFFSET_TABLES + TABLE_SETUPS_MAX)
#define CONFIG_OFFSET_CORRECTIONS (CONFIG_OFFSET_PARAMS + 1)
#define CONFIG_OFFSET_CRITICALS 1920

static const float default_cylinders[ECU_CYLINDERS_COUNT] = {
    1, 2, 3, 4
};

static const float default_pressures[TABLE_PRESSURES_MAX] = {
    10000, 16200, 22400, 28600, 34800, 41000, 47200, 53400,
    59600, 65800, 72000, 78200, 84400, 90600, 96800, 103000
};

static const float default_idle_rotates[TABLE_ROTATES_MAX] = {
    525, 556, 588, 625, 667, 714, 769, 833,
    909, 1000, 1111, 1250, 1429, 1667, 2000, 2500,
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

static const float default_filling_select_koff_tps[TABLE_FILLING_MAX] = {
    0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f,
    0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f,
};

static const float default_filling_gbc_map[TABLE_PRESSURES_MAX][TABLE_ROTATES_MAX] = {
    { 0.0520f, 0.0520f, 0.0519f, 0.0518f, 0.0519f, 0.0524f, 0.0539f, 0.0617f, 0.0620f, 0.0598f, 0.0650f, 0.0932f, 0.0599f, 0.0605f, 0.0606f, 0.0606f, },
    { 0.0849f, 0.0847f, 0.0844f, 0.0841f, 0.0841f, 0.0849f, 0.0871f, 0.1015f, 0.1042f, 0.0983f, 0.1278f, 0.1530f, 0.0943f, 0.0974f, 0.0990f, 0.0990f, },
    { 0.1183f, 0.1179f, 0.1171f, 0.1163f, 0.1161f, 0.1175f, 0.1242f, 0.1370f, 0.1419f, 0.1342f, 0.1291f, 0.1813f, 0.1287f, 0.1374f, 0.1380f, 0.1380f, },
    { 0.1525f, 0.1517f, 0.1505f, 0.1494f, 0.1500f, 0.1520f, 0.1507f, 0.1253f, 0.0733f, 0.1428f, 0.1735f, 0.2370f, 0.1583f, 0.1776f, 0.1776f, 0.1776f, },
    { 0.1877f, 0.1868f, 0.1859f, 0.1843f, 0.1865f, 0.2083f, 0.1725f, 0.1697f, 0.1952f, 0.1952f, 0.2540f, 0.2895f, 0.2123f, 0.2490f, 0.2179f, 0.2179f, },
    { 0.2245f, 0.2234f, 0.2216f, 0.2216f, 0.2219f, 0.2441f, 0.2256f, 0.1923f, 0.2833f, 0.2231f, 0.3015f, 0.3440f, 0.2763f, 0.2959f, 0.2959f, 0.2589f, },
    { 0.2620f, 0.2582f, 0.2466f, 0.2672f, 0.2212f, 0.2556f, 0.2586f, 0.2534f, 0.3312f, 0.2831f, 0.3359f, 0.4021f, 0.3634f, 0.3436f, 0.3436f, 0.3007f, },
    { 0.2952f, 0.2967f, 0.2864f, 0.3153f, 0.2893f, 0.2707f, 0.3310f, 0.2976f, 0.3766f, 0.3751f, 0.4065f, 0.4163f, 0.4148f, 0.4403f, 0.3918f, 0.3923f, },
    { 0.3320f, 0.3353f, 0.3447f, 0.3121f, 0.3242f, 0.3784f, 0.3342f, 0.3126f, 0.3226f, 0.4402f, 0.4419f, 0.5170f, 0.4750f, 0.4932f, 0.4955f, 0.4419f, },
    { 0.3718f, 0.3718f, 0.3478f, 0.3460f, 0.3903f, 0.4346f, 0.4525f, 0.4260f, 0.4531f, 0.4913f, 0.5091f, 0.5793f, 0.5190f, 0.5510f, 0.5528f, 0.5541f, },
    { 0.4136f, 0.4197f, 0.3803f, 0.4082f, 0.3850f, 0.4211f, 0.3762f, 0.5361f, 0.4871f, 0.4932f, 0.5504f, 0.6626f, 0.6048f, 0.6095f, 0.6116f, 0.6123f, },
    { 0.4763f, 0.4374f, 0.3680f, 0.5054f, 0.5270f, 0.5375f, 0.4367f, 0.4822f, 0.6330f, 0.6599f, 0.7390f, 0.6367f, 0.6688f, 0.6696f, 0.6711f, 0.6718f, },
    { 0.5841f, 0.4507f, 0.4865f, 0.5345f, 0.5467f, 0.6305f, 0.5995f, 0.6353f, 0.6793f, 0.8143f, 0.8119f, 0.7964f, 0.7338f, 0.7305f, 0.7313f, 0.7321f, },
    { 0.6227f, 0.6121f, 0.5574f, 0.5415f, 0.6235f, 0.3951f, 0.5900f, 0.6985f, 0.7329f, 0.7347f, 0.7602f, 0.8749f, 0.8696f, 0.8793f, 0.8802f, 0.8819f, },
    { 0.6244f, 0.7396f, 0.6806f, 0.4778f, 0.6473f, 0.5949f, 0.6558f, 0.8415f, 0.8424f, 0.8082f, 0.7672f, 0.9033f, 1.0109f, 0.9081f, 0.9186f, 0.9519f, },
    { 0.7858f, 0.8165f, 0.8062f, 0.7817f, 0.7909f, 0.8206f, 0.9147f, 0.9014f, 0.9147f, 0.9065f, 0.9423f, 0.9996f, 0.9638f, 0.9260f, 0.9781f, 1.0232f, },
};

static const float default_filling_gbc_tps[TABLE_THROTTLES_MAX][TABLE_ROTATES_MAX] = {
    { 0.595f, 0.603f, 0.579f, 0.584f, 0.586f, 0.608f, 0.637f, 0.474f, 0.420f, 0.653f, 0.788f, 1.026f, 0.697f, 0.789f, 0.700f, 0.700f, },
    { 0.603f, 0.604f, 0.568f, 0.593f, 0.585f, 0.589f, 0.571f, 0.592f, 0.492f, 0.608f, 0.727f, 1.000f, 0.700f, 0.700f, 0.700f, 0.700f, },
    { 0.601f, 0.601f, 0.609f, 0.590f, 0.586f, 0.636f, 0.630f, 0.744f, 0.730f, 0.631f, 0.734f, 1.000f, 0.700f, 0.715f, 0.700f, 0.700f, },
    { 0.600f, 0.608f, 0.674f, 0.593f, 0.603f, 0.609f, 0.651f, 0.699f, 0.717f, 0.647f, 0.771f, 1.000f, 0.700f, 0.788f, 0.700f, 0.700f, },
    { 0.598f, 0.603f, 0.669f, 0.750f, 0.743f, 0.740f, 0.713f, 0.726f, 0.763f, 0.722f, 0.782f, 1.000f, 0.725f, 0.800f, 0.700f, 0.700f, },
    { 0.600f, 0.602f, 0.791f, 0.681f, 0.753f, 0.743f, 0.748f, 0.770f, 0.750f, 0.745f, 0.767f, 1.002f, 0.773f, 0.800f, 0.797f, 0.700f, },
    { 0.598f, 0.602f, 0.577f, 0.693f, 0.774f, 0.776f, 0.758f, 0.782f, 0.788f, 0.800f, 0.802f, 1.001f, 0.881f, 0.827f, 0.800f, 0.700f, },
    { 0.600f, 0.601f, 0.590f, 0.579f, 0.708f, 0.696f, 0.685f, 0.802f, 0.857f, 0.900f, 0.877f, 1.000f, 0.900f, 0.900f, 0.803f, 0.761f, },
    { 0.645f, 0.612f, 0.608f, 0.583f, 0.613f, 0.750f, 0.779f, 0.812f, 0.900f, 0.900f, 0.899f, 1.000f, 0.900f, 0.900f, 0.900f, 0.800f, },
    { 0.700f, 0.647f, 0.570f, 0.579f, 0.613f, 0.742f, 0.777f, 0.889f, 0.895f, 0.900f, 0.900f, 1.000f, 0.900f, 0.900f, 0.900f, 0.855f, },
    { 0.739f, 0.700f, 0.599f, 0.594f, 0.601f, 0.684f, 0.762f, 0.894f, 0.897f, 0.900f, 0.900f, 1.000f, 0.982f, 0.901f, 0.900f, 0.900f, },
    { 0.739f, 0.700f, 0.642f, 0.664f, 0.670f, 0.729f, 0.765f, 0.798f, 0.899f, 0.900f, 0.900f, 1.000f, 1.000f, 0.901f, 0.900f, 0.900f, },
    { 0.739f, 0.740f, 0.700f, 0.694f, 0.730f, 0.749f, 0.784f, 0.891f, 0.897f, 0.900f, 0.900f, 1.000f, 1.000f, 1.000f, 0.900f, 0.900f, },
    { 0.739f, 0.758f, 0.748f, 0.738f, 0.752f, 0.767f, 0.788f, 0.900f, 0.893f, 0.882f, 0.899f, 1.000f, 1.000f, 0.999f, 0.989f, 0.974f, },
    { 0.739f, 0.776f, 0.780f, 0.762f, 0.770f, 0.782f, 0.798f, 0.900f, 0.895f, 0.885f, 0.899f, 1.000f, 1.001f, 0.998f, 1.000f, 1.000f, },
    { 0.751f, 0.781f, 0.784f, 0.780f, 0.785f, 0.795f, 0.807f, 0.900f, 0.900f, 0.900f, 0.900f, 1.000f, 1.001f, 0.982f, 1.000f, 1.000f, },
};

static const float default_ignitions[TABLE_FILLING_MAX][TABLE_ROTATES_MAX] = {
    { 14.7f, 16.6f, 18.8f, 22.4f, 26.8f, 33.1f, 39.4f, 41.8f, 43.1f, 43.8f, 44.2f, 44.3f, 44.3f, 44.6f, 44.9f, 45.3f, },
    { 16.5f, 18.8f, 21.0f, 24.5f, 28.5f, 34.0f, 39.6f, 41.9f, 43.1f, 43.8f, 44.2f, 44.3f, 44.3f, 44.6f, 44.9f, 45.3f, },
    { 18.2f, 20.3f, 22.3f, 25.5f, 29.2f, 34.4f, 39.6f, 41.8f, 43.0f, 43.7f, 44.1f, 44.2f, 44.2f, 44.6f, 44.9f, 45.3f, },
    { 18.7f, 20.7f, 22.5f, 25.4f, 29.0f, 34.1f, 39.2f, 41.4f, 42.4f, 43.1f, 43.4f, 43.5f, 43.6f, 44.1f, 44.4f, 44.8f, },
    { 17.7f, 19.7f, 21.6f, 24.5f, 28.0f, 32.9f, 38.0f, 40.2f, 41.3f, 41.9f, 42.3f, 42.4f, 42.5f, 43.0f, 43.3f, 43.7f, },
    { 15.2f, 17.7f, 19.4f, 22.4f, 26.1f, 31.0f, 36.0f, 38.4f, 39.6f, 40.5f, 41.0f, 41.1f, 41.3f, 41.8f, 42.1f, 42.4f, },
    { 14.0f, 15.9f, 18.9f, 20.6f, 23.8f, 28.2f, 33.2f, 35.8f, 37.3f, 38.4f, 39.2f, 39.7f, 40.0f, 40.5f, 40.9f, 41.2f, },
    { 12.5f, 14.2f, 16.2f, 18.6f, 21.1f, 25.0f, 29.3f, 32.4f, 34.5f, 36.3f, 37.4f, 38.1f, 38.5f, 39.1f, 39.4f, 39.7f, },
    { 12.2f, 13.8f, 15.3f, 17.2f, 19.0f, 22.0f, 25.4f, 28.5f, 31.3f, 33.8f, 35.2f, 35.5f, 35.7f, 36.1f, 36.3f, 36.7f, },
    { 11.7f, 13.4f, 14.4f, 15.6f, 17.8f, 19.9f, 23.0f, 26.0f, 28.9f, 31.5f, 32.8f, 32.5f, 32.4f, 32.8f, 32.9f, 33.3f, },
    { 11.0f, 12.8f, 13.5f, 14.6f, 16.2f, 18.7f, 21.2f, 24.5f, 27.4f, 29.7f, 30.7f, 30.1f, 29.9f, 30.3f, 30.6f, 31.0f, },
    { 10.8f, 11.6f, 12.3f, 13.4f, 14.9f, 17.5f, 19.7f, 22.8f, 25.9f, 28.2f, 29.0f, 28.2f, 28.0f, 28.6f, 28.9f, 29.3f, },
    { 10.4f, 10.7f, 11.4f, 12.2f, 14.0f, 15.6f, 18.5f, 20.3f, 23.4f, 25.9f, 26.9f, 26.2f, 26.0f, 26.5f, 26.9f, 27.2f, },
    { 10.2f, 10.8f, 11.2f, 11.6f, 13.3f, 14.6f, 16.9f, 18.7f, 21.1f, 23.9f, 25.0f, 24.4f, 24.2f, 24.6f, 24.9f, 25.3f, },
    { 10.0f, 10.4f, 10.9f, 11.2f, 12.8f, 14.2f, 15.2f, 16.8f, 19.6f, 21.9f, 23.1f, 22.5f, 22.4f, 22.8f, 22.9f, 23.5f, },
    { 10.0f, 10.2f, 10.4f, 11.1f, 12.4f, 13.6f, 14.3f, 15.6f, 18.1f, 20.1f, 21.4f, 20.9f, 20.7f, 21.0f, 21.1f, 21.7f, },
};

static const float default_fuel_mixtures[TABLE_FILLING_MAX][TABLE_ROTATES_MAX] = {
    { 13.8f, 13.8f, 13.9f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, },
    { 13.8f, 13.8f, 13.9f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, },
    { 13.8f, 13.8f, 13.9f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, },
    { 13.8f, 13.8f, 13.9f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, },
    { 13.8f, 13.8f, 13.9f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, },
    { 13.8f, 13.9f, 13.9f, 13.9f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, },
    { 13.8f, 13.8f, 13.9f, 13.9f, 13.9f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, 14.0f, },
    { 13.6f, 13.6f, 13.8f, 13.8f, 13.8f, 13.8f, 13.8f, 13.8f, 13.8f, 13.8f, 13.8f, 13.8f, 13.8f, 13.8f, 13.8f, 13.8f, },
    { 13.5f, 13.5f, 13.5f, 13.6f, 13.6f, 13.6f, 13.6f, 13.6f, 13.6f, 13.6f, 13.6f, 13.6f, 13.6f, 13.6f, 13.6f, 13.6f, },
    { 13.3f, 13.3f, 13.3f, 13.4f, 13.4f, 13.4f, 13.4f, 13.4f, 13.4f, 13.4f, 13.4f, 13.4f, 13.4f, 13.4f, 13.4f, 13.4f, },
    { 13.2f, 13.2f, 13.2f, 13.2f, 13.2f, 13.2f, 13.2f, 13.2f, 13.2f, 13.2f, 13.2f, 13.2f, 13.2f, 13.2f, 13.2f, 13.2f, },
    { 13.0f, 13.0f, 12.9f, 12.9f, 12.9f, 12.9f, 12.9f, 12.9f, 12.9f, 12.9f, 12.9f, 12.9f, 12.9f, 12.9f, 12.9f, 12.9f, },
    { 12.9f, 12.9f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, },
    { 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, },
    { 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, },
    { 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, 12.8f, },
};

static const float default_injection_phase[TABLE_FILLING_MAX][TABLE_ROTATES_MAX] = {
    { 350, 350, 350, 350, 350, 345, 340, 335, 330, 320, 320, 310, 300, 300, 300, 300, },
    { 350, 350, 350, 350, 350, 345, 340, 335, 330, 320, 320, 310, 300, 300, 300, 300, },
    { 350, 350, 350, 350, 350, 345, 340, 335, 330, 320, 320, 310, 300, 300, 300, 320, },
    { 350, 350, 350, 350, 350, 345, 340, 335, 330, 320, 325, 335, 320, 330, 340, 350, },
    { 350, 350, 350, 350, 350, 350, 350, 350, 350, 350, 350, 355, 350, 350, 380, 400, },
    { 350, 350, 350, 350, 350, 345, 340, 345, 340, 345, 360, 390, 400, 405, 430, 440, },
    { 345, 345, 345, 350, 350, 350, 350, 350, 350, 400, 420, 445, 450, 470, 490, 500, },
    { 340, 340, 340, 345, 350, 350, 350, 350, 350, 410, 460, 490, 520, 535, 555, 570, },
    { 335, 335, 335, 335, 345, 350, 345, 345, 350, 420, 500, 520, 540, 560, 570, 580, },
    { 330, 330, 330, 330, 340, 340, 340, 340, 350, 420, 500, 520, 540, 560, 570, 580, },
    { 325, 325, 325, 325, 330, 335, 340, 345, 350, 420, 500, 520, 540, 560, 570, 580, },
    { 320, 320, 320, 320, 320, 320, 330, 345, 350, 420, 500, 520, 540, 560, 570, 580, },
    { 310, 310, 310, 310, 310, 310, 310, 310, 345, 420, 500, 520, 540, 560, 570, 580, },
    { 300, 300, 300, 300, 300, 300, 300, 300, 340, 420, 500, 520, 540, 560, 570, 580, },
    { 290, 290, 290, 290, 290, 290, 290, 310, 340, 420, 500, 520, 540, 560, 570, 580, },
    { 280, 280, 280, 290, 290, 290, 295, 310, 330, 420, 500, 520, 540, 560, 570, 580, },


};

static const float default_injection_phase_lpf[TABLE_ROTATES_MAX] = {
    0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f,
    0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f,
};

static const float default_enrichment_rate_start_load[TABLE_ENRICHMENT_PERCENTS_MAX] = {
    0.0f, 5.0f, 10.0f, 15.0f, 25.0f, 40.0f, 60.0f, 80.0f
};

static const float default_enrichment_rate_load_derivative[TABLE_ENRICHMENT_PERCENTS_MAX] = {
    0, 500, 1200, 2000, 3200, 4500, 6000, 8000,
};

static const float default_enrichment_rate[TABLE_ENRICHMENT_PERCENTS_MAX][TABLE_ENRICHMENT_PERCENTS_MAX] = {
    { 0.00f, 0.31f, 0.53f, 0.70f, 0.80f, 0.85f, 0.95f, 1.00f, },
    { 0.00f, 0.23f, 0.46f, 0.60f, 0.70f, 0.75f, 0.80f, 0.90f, },
    { 0.00f, 0.15f, 0.34f, 0.45f, 0.50f, 0.55f, 0.60f, 0.75f, },
    { 0.00f, 0.09f, 0.21f, 0.34f, 0.40f, 0.46f, 0.53f, 0.65f, },
    { 0.00f, 0.00f, 0.04f, 0.20f, 0.28f, 0.35f, 0.40f, 0.45f, },
    { 0.00f, 0.00f, 0.00f, 0.08f, 0.16f, 0.21f, 0.24f, 0.27f, },
    { 0.00f, 0.00f, 0.00f, 0.01f, 0.04f, 0.06f, 0.08f, 0.09f, },
    { 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, },
};

static const float default_enrichment_sync_amount[TABLE_ROTATES_MAX] = {
    0.30f, 0.30f, 0.30f, 0.32f, 0.35f, 0.39f, 0.43f, 0.48f,
    0.51f, 0.53f, 0.55f, 0.58f, 0.65f, 0.76f, 0.80f, 0.80f,
};

static const float default_enrichment_async_amount[TABLE_ROTATES_MAX] = {
    0.30f, 0.30f, 0.30f, 0.32f, 0.35f, 0.39f, 0.43f, 0.48f,
    0.51f, 0.53f, 0.55f, 0.58f, 0.65f, 0.76f, 0.80f, 0.80f,
};

static const float default_enrichment_ign_corr[TABLE_ROTATES_MAX][TABLE_ENRICHMENT_PERCENTS_MAX] = {
    { -4.0f, -3.9f, -3.7f, -3.6f, -3.3f, -2.9f, -2.4f, -2.0f, },
    { -4.0f, -3.9f, -3.7f, -3.6f, -3.3f, -2.9f, -2.4f, -2.0f, },
    { -4.0f, -3.9f, -3.7f, -3.6f, -3.3f, -2.9f, -2.4f, -2.0f, },
    { -4.0f, -3.9f, -3.7f, -3.5f, -3.3f, -2.9f, -2.4f, -2.0f, },
    { -3.8f, -3.7f, -3.5f, -3.4f, -3.2f, -2.8f, -2.3f, -1.9f, },
    { -3.5f, -3.5f, -3.2f, -3.1f, -3.0f, -2.6f, -2.2f, -1.8f, },
    { -3.2f, -3.2f, -2.9f, -2.9f, -2.8f, -2.5f, -2.0f, -1.7f, },
    { -2.9f, -2.9f, -2.6f, -2.6f, -2.5f, -2.3f, -1.9f, -1.6f, },
    { -2.4f, -2.4f, -2.3f, -2.2f, -2.2f, -2.0f, -1.8f, -1.5f, },
    { -2.0f, -2.0f, -1.9f, -1.8f, -1.8f, -1.7f, -1.6f, -1.4f, },
    { -1.4f, -1.4f, -1.4f, -1.4f, -1.4f, -1.4f, -1.4f, -1.3f, },
    { -0.7f, -0.8f, -0.7f, -0.8f, -0.8f, -1.0f, -1.1f, -1.1f, },
    { 0.0f, -0.1f, -0.1f, -0.2f, -0.3f, -0.5f, -0.8f, -1.0f, },
    { 0.0f, -0.1f, -0.1f, -0.2f, -0.3f, -0.5f, -0.8f, -1.0f, },
    { 0.0f, -0.1f, -0.1f, -0.2f, -0.3f, -0.5f, -0.8f, -1.0f, },
    { 0.0f, -0.1f, -0.1f, -0.2f, -0.3f, -0.5f, -0.8f, -1.0f, },
};


static const float default_enrichment_temp_mult[TABLE_TEMPERATURES_MAX] = {
    3.60f, 3.40f, 3.20f, 2.60f, 1.80f, 1.00f, 0.70f, 0.50f,
    0.40f, 0.27f, 0.10f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f,
};

static const float default_enrichment_injection_phase[TABLE_ROTATES_MAX] = {
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
};

static const float default_ignition_time_rpm_mult[TABLE_ROTATES_MAX] = {
    4.0f, 1.7f, 1.1f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f
};

static const float default_voltages[8] = {
    0.0f, 6.0f, 8.0f, 10.0f, 12.0f, 14.0f, 16.0f, 25.0f
};

static const float default_ignition_time[TABLE_VOLTAGES_MAX] = {
    5700, 4500, 3200, 2500, 2100, 1800, 1600, 1000
};

static const float default_injector_lag[TABLE_VOLTAGES_MAX] = {
    3.00f, 1.86f, 1.08f, 0.77f, 0.46f, 0.29f, 0.11f, 0.03f
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
    0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f,
    0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f,
};

static const float default_warmup_mix_corrs[TABLE_TEMPERATURES_MAX] = {
    0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f,
    0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f,
};

static const float default_cold_start_idle_corrs[TABLE_TEMPERATURES_MAX] = {
    0.20f, 0.15f, 0.13f, 0.10f, 0.08f, 0.06f, 0.05f, 0.04f,
    0.03f, 0.02f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f,
};

static const float default_cold_start_idle_times[TABLE_TEMPERATURES_MAX] = {
    9, 8, 7, 6, 5, 4, 4, 3,
    2, 1, 1, 1, 1, 1, 1, 1,
};

static const float default_start_tps_corrs[TABLE_THROTTLES_MAX] = {
    1.00f, 1.00f, 1.00f, 1.00f, 0.95f, 0.90f, 0.85f, 0.80f,
    0.75f, 0.60f, 0.55f, 0.50f, 0.45f, 0.40f, 0.00f, 0.00f
};

static const float default_start_async_filling[TABLE_TEMPERATURES_MAX] = {
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
};

static const float default_start_large_filling[TABLE_TEMPERATURES_MAX] = {
    1380, 1150, 1010, 830, 690, 575, 520, 500,
    475, 460, 440, 430, 430, 430, 430, 430,
};

static const float default_start_small_filling[TABLE_TEMPERATURES_MAX] = {
    1380, 1030, 860, 660, 490, 410, 360, 330,
    300, 290, 290, 290, 290, 290, 290, 290,
};

static const float default_start_filling_time[TABLE_TEMPERATURES_MAX] = {
    5.0f, 4.5f, 4.0f, 3.5f, 3.0f, 2.5f, 2.0f, 1.5f,
    1.0f, 0.6f, 0.3f, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f,
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
    100.0f, 100.0f, 100.0f, 97.0f, 70.0f, 55.0f, 45.0f, 40.0f,
    35.0f, 32.0f, 30.0f, 30.0f, 30.0f, 30.0f, 30.0f, 30.0f,
};

static const float default_idle_wish_rotates[TABLE_TEMPERATURES_MAX] = {
    1700, 1700, 1700, 1700, 1350, 1300, 1200, 1150,
    1100, 1100, 1100, 1100, 1100, 1250, 1350, 1450,
};

static const float default_idle_valve_position[TABLE_TEMPERATURES_MAX] = {
    70, 70, 70, 70, 65, 60, 55, 55,
    53, 50, 50, 50, 50, 50, 50, 50,
};

static const float default_idle_valve_econ_position[TABLE_ROTATES_MAX] = {
    50, 50, 50, 50, 50, 50, 50, 50,
    50, 50, 50, 50, 50, 50, 50, 50,
};

static const float default_idle_wish_massair[TABLE_TEMPERATURES_MAX] = {
    35.0f, 35.0f, 35.0f, 35.0f, 33.1f, 30.9f, 29.2f, 26.7f,
    25.5f, 23.0f, 23.0f, 22.5f, 22.5f, 22.5f, 22.5f, 22.5f,
};

static const float default_idle_wish_ignition_static[TABLE_ROTATES_MAX] = {
    14.0f, 16.0f, 18.0f, 21.0f, 25.0f, 25.7f, 23.0f, 19.0f,
    16.0f, 13.0f, 10.0f, 10.0f, 11.1f, 13.0f, 15.0f, 18.0f,
};

static const float default_idle_wish_ignition[TABLE_TEMPERATURES_MAX] = {
    18.0f, 17.0f, 16.0f, 15.0f, 14.0f, 13.0f, 12.0f, 11.0f,
    10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f,
};

static const float default_speeds[TABLE_SPEEDS_MAX] = {
    0, 3, 10, 20, 30, 40, 50, 60,
    70, 80, 90, 100, 110, 120, 130, 140
};

static const float default_idle_rpm_shift[TABLE_SPEEDS_MAX] = {
    0, 20, 40, 40, 60, 70, 75, 80,
    85, 90, 90, 95, 100, 100, 100, 100,
};

static const float default_knock_noise_level[TABLE_ROTATES_MAX] = {
    1.00f, 1.10f, 1.20f, 1.30f, 1.38f, 1.45f, 1.50f, 1.50f,
    1.50f, 1.50f, 1.50f, 1.50f, 1.50f, 1.50f, 1.50f, 1.50f,
};

static const float default_knock_threshold[TABLE_ROTATES_MAX] = {
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f
};

static const float default_knock_filter_frequency[TABLE_ROTATES_MAX] = {
    41, 41, 41, 41, 41, 41, 41, 41,
    41, 41, 41, 41, 41, 41, 41, 41,
};

static const float default_knock_gain[TABLE_ROTATES_MAX] = {
    0, 0, 1, 3, 7, 8, 13, 14,
    15, 16, 26, 30, 41, 46, 50, 53,
};

static const float default_knock_zone[TABLE_FILLING_MAX][TABLE_ROTATES_MAX] = {
    { 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, },
    { 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, },
    { 1.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, },
    { 1.00f, 1.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, },
    { 1.00f, 1.00f, 1.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, },
    { 1.00f, 1.00f, 1.00f, 1.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, },
    { 1.00f, 1.00f, 1.00f, 1.00f, 0.50f, 0.50f, 0.50f, 0.50f, 0.50f, 0.50f, 0.50f, 0.50f, 0.50f, 0.50f, 0.50f, 0.50f, },
    { 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, },
    { 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, },
    { 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, },
    { 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, },
    { 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, },
    { 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, },
    { 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, },
    { 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, },
    { 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, },
};

static const float default_knock_cy_level_multiplier[ECU_CYLINDERS_COUNT][TABLE_ROTATES_MAX] = {
    { 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, },
    { 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, },
    { 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, },
    { 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, },
};

static const float default_air_temps[TABLE_TEMPERATURES_MAX] = {
    -40, -30, -20, -10, -5, 0, 5, 10,
    15, 20, 30, 40, 50, 60, 70, 80
};

static const float default_air_temp_mix_corrs[TABLE_TEMPERATURES_MAX][TABLE_FILLING_MAX] = {
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

static const float default_air_temp_ign_corrs[TABLE_TEMPERATURES_MAX][TABLE_FILLING_MAX] = {
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
};

static const float default_engine_temp_mix_corrs[TABLE_TEMPERATURES_MAX][TABLE_FILLING_MAX] = {
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

static const float default_engine_temp_ign_corrs[TABLE_TEMPERATURES_MAX][TABLE_FILLING_MAX] = {
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, },
};

static const float default_idle_pids_rpm_koffs[TABLE_ROTATES_MAX] = {
    0.45f, 0.53f, 0.61f, 0.69f, 0.77f, 0.85f, 0.93f, 1.00f,
    1.08f, 1.16f, 1.24f, 1.32f, 1.40f, 1.48f, 1.56f, 1.64f
};

static const float default_idle_valve_to_massair_pid_p[TABLE_ROTATES_MAX] = {
    2.000f, 2.000f, 1.900f, 1.800f, 1.700f, 1.600f, 1.400f, 1.200f,
    1.200f, 1.200f, 1.200f, 1.200f, 1.200f, 1.200f, 1.200f, 1.200f,
};

static const float default_idle_valve_to_massair_pid_i[TABLE_ROTATES_MAX] = {
    5.000f, 5.000f, 5.000f, 5.000f, 5.000f, 5.000f, 4.600f, 4.000f,
    3.600f, 2.000f, 1.200f, 0.800f, 0.700f, 0.700f, 0.700f, 0.700f,
};

static const float default_idle_valve_to_massair_pid_d[TABLE_ROTATES_MAX] = {
    0.0020f, 0.0020f, 0.0080f, 0.0130f, 0.0200f, 0.0150f, 0.0050f, 0.0010f,
    0.0010f, 0.0010f, 0.0010f, 0.0010f, 0.0010f, 0.0010f, 0.0010f, 0.0010f,
};

static const float default_idle_valve_to_rpm_pid_p[TABLE_ROTATES_MAX] = {
    0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f,
    0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f,
};

static const float default_idle_valve_to_rpm_pid_i[TABLE_ROTATES_MAX] = {
    0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f,
    0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f, 0.000f,
};

static const float default_idle_valve_to_rpm_pid_d[TABLE_ROTATES_MAX] = {
    0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f,
    0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f,
};

static const float default_idle_ign_to_rpm_pid_p[TABLE_ROTATES_MAX] = {
    0.020f, 0.020f, 0.020f, 0.020f, 0.020f, 0.020f, 0.020f, 0.020f,
    0.020f, 0.020f, 0.020f, 0.020f, 0.020f, 0.020f, 0.020f, 0.020f,
};

static const float default_idle_ign_to_rpm_pid_i[TABLE_ROTATES_MAX] = {
    0.020f, 0.020f, 0.020f, 0.020f, 0.020f, 0.020f, 0.020f, 0.020f,
    0.018f, 0.014f, 0.008f, 0.004f, 0.002f, 0.002f, 0.002f, 0.002f,
};

static const float default_idle_ign_to_rpm_pid_d[TABLE_ROTATES_MAX] = {
    0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0050f, 0.0020f, 0.0015f,
    0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f,
};

static const float default_idle_rpm_pid_act_1[TABLE_TEMPERATURES_MAX] = {
    0.400f, 0.400f, 0.400f, 0.400f, 0.400f, 0.400f, 0.400f, 0.400f,
    0.380f, 0.330f, 0.300f, 0.300f, 0.300f, 0.300f, 0.300f, 0.300f,
};

static const float default_idle_rpm_pid_act_2[TABLE_TEMPERATURES_MAX] = {
    0.600f, 0.600f, 0.600f, 0.600f, 0.600f, 0.600f, 0.600f, 0.550f,
    0.470f, 0.370f, 0.340f, 0.340f, 0.340f, 0.340f, 0.340f, 0.340f,
};

static const float default_tsps_relative_pos[TABLE_ROTATES_MAX] = {
    -112.0f, -112.0f, -112.0f, -112.0f, -112.0f, -111.9f, -111.7f, -111.5f,
    -111.2f, -111.0f, -111.0f, -111.0f, -111.0f, -111.0f, -111.0f, -111.0f,
};

static const float default_tsps_desync_thr[TABLE_ROTATES_MAX] = {
    3.0f, 3.0f, 3.0f, 3.0f, 3.0f, 3.0f, 3.0f, 3.0f,
    3.0f, 3.0f, 3.0f, 3.0f, 3.0f, 3.0f, 3.0f, 3.0f,
};

static const float default_idle_ignition_time_by_tps[TABLE_THROTTLES_MAX] = {
    0.50f, 0.50f, 0.38f, 0.25f, 0.12f, 0.05f, 0.00f, 0.00f,
    0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f,
};

static const float default_idle_econ_delay[TABLE_TEMPERATURES_MAX] = {
    30.0f, 30.0f, 30.0f, 30.0f, 30.0f, 30.0f, 30.0f, 20.0f,
    10.0f, 3.0f, 1.0f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f,
};

static const float default_start_econ_delay[TABLE_TEMPERATURES_MAX] = {
    5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 4.8f, 4.5f,
    4.0f, 3.5f, 3.2f, 3.0f, 3.0f, 3.0f, 3.0f, 3.0f,
};

static const float default_fan_advance_control[TABLE_TEMPERATURES_MAX][TABLE_SPEEDS_MAX] = {
    { -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, },
    { -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, },
    { -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, },
    { -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, },
    { -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, },
    { -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, },
    { -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, },
    { -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, },
    { -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, },
    { -1.00f, -1.00f, -0.70f, -0.60f, -0.50f, -0.50f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, },
    { 0.00f, 0.00f, -0.15f, -0.20f, -0.30f, -0.30f, -0.40f, -0.70f, -0.90f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, -1.00f, },
    { 1.00f, 1.00f, 1.00f, 1.00f, 0.70f, 0.70f, 0.50f, 0.25f, 0.25f, 0.25f, 0.15f, -0.10f, -0.10f, -0.10f, -0.10f, -0.10f, },
    { 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 0.90f, 0.80f, 0.70f, 0.50f, 0.30f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f, },
    { 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, },
    { 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, },
    { 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, },
};

void config_default_table(sEcuTable *table, uint8_t number)
{
  memset(table, 0, sizeof(sEcuTable));

  sprintf(table->name, "Default %d", number + 1);

  table->inj_channel = InjectorChannel1;

  //Bosch 0280155968 - 420 cc/min
  //Bosch 0280156095 - 315 cc/min
  //Bosch 0280155746 - 200 cc/min
  //BMW 03762FA - 180 cc/min
  table->injector_performance = 200.0f;

  table->is_fuel_phase_by_end = 1;
  table->enrichment_ph_sync_enabled = 1;
  table->enrichment_ph_async_enabled = 0;
  table->enrichment_ph_post_injection_enabled = 1;
  table->enrichment_pp_sync_enabled = 1;
  table->enrichment_pp_async_enabled = 0;
  table->enrichment_pp_post_injection_enabled = 1;
  table->fuel_mass_per_cc = 0.75f;
  table->fuel_afr = 14.7f;

  table->enrichment_load_type = 0;
  table->enrichment_load_dead_band = 500.0f;
  table->enrichment_accel_dead_band = 100000.0f;
  table->enrichment_detect_duration = 100.0f;
  table->enrichment_ign_corr_decay_time = 200.0f;
  table->enrichment_injection_phase_decay_time = 200.0f;
  table->enrichment_async_pulses_divider = 5;
  table->enrichment_end_injection_final_phase = 540;
  table->enrichment_end_injection_final_amount = 1.0f;

  table->idle_valve_pos_min = -15;
  table->idle_valve_pos_max = 20;

  table->cylinders_count = ITEMSOF(default_cylinders);
  memcpy(table->cylinders, default_cylinders, sizeof(default_cylinders));

  table->voltages_count = ITEMSOF(default_voltages);
  memcpy(table->voltages, default_voltages, sizeof(default_voltages));

  table->pressures_count = ITEMSOF(default_pressures);
  memcpy(table->pressures, default_pressures, sizeof(default_pressures));

  table->rotates_count = ITEMSOF(default_rotates);
  memcpy(table->rotates, default_rotates, sizeof(default_rotates));

  table->idle_rotates_count = ITEMSOF(default_idle_rotates);
  memcpy(table->idle_rotates, default_idle_rotates, sizeof(default_idle_rotates));

  table->throttles_count = ITEMSOF(default_throttles);
  memcpy(table->throttles, default_throttles, sizeof(default_throttles));

  memcpy(table->filling_gbc_map, default_filling_gbc_map, sizeof(default_filling_gbc_map));
  memcpy(table->filling_gbc_tps, default_filling_gbc_tps, sizeof(default_filling_gbc_tps));
  memcpy(table->filling_select_koff_tps, default_filling_select_koff_tps, sizeof(default_filling_select_koff_tps));

  table->enrichment_rate_start_load_count = ITEMSOF(default_enrichment_rate_start_load);
  memcpy(table->enrichment_rate_start_load, default_enrichment_rate_start_load, sizeof(default_enrichment_rate_start_load));
  table->enrichment_rate_load_derivative_count = ITEMSOF(default_enrichment_rate_load_derivative);
  memcpy(table->enrichment_rate_load_derivative, default_enrichment_rate_load_derivative, sizeof(default_enrichment_rate_load_derivative));

  memcpy(table->enrichment_rate, default_enrichment_rate, sizeof(default_enrichment_rate));
  memcpy(table->enrichment_sync_amount, default_enrichment_sync_amount, sizeof(default_enrichment_sync_amount));
  memcpy(table->enrichment_async_amount, default_enrichment_async_amount, sizeof(default_enrichment_async_amount));
  memcpy(table->enrichment_ign_corr, default_enrichment_ign_corr, sizeof(default_enrichment_ign_corr));
  memcpy(table->enrichment_temp_mult, default_enrichment_temp_mult, sizeof(default_enrichment_temp_mult));
  memcpy(table->enrichment_injection_phase, default_enrichment_injection_phase, sizeof(default_enrichment_injection_phase));

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
  memcpy(table->engine_temp_mix_corr, default_engine_temp_mix_corrs, sizeof(default_engine_temp_mix_corrs));
  memcpy(table->engine_temp_ign_corr, default_engine_temp_ign_corrs, sizeof(default_engine_temp_ign_corrs));

  memcpy(table->idle_wish_rotates, default_idle_wish_rotates, sizeof(default_idle_wish_rotates));
  memcpy(table->idle_wish_massair, default_idle_wish_massair, sizeof(default_idle_wish_massair));
  memcpy(table->idle_wish_ignition_static, default_idle_wish_ignition_static, sizeof(default_idle_wish_ignition_static));
  memcpy(table->idle_wish_ignition, default_idle_wish_ignition, sizeof(default_idle_wish_ignition));
  memcpy(table->idle_valve_position, default_idle_valve_position, sizeof(default_idle_valve_position));
  memcpy(table->idle_valve_econ_position, default_idle_valve_econ_position, sizeof(default_idle_valve_econ_position));

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

  table->idle_pids_rpm_koffs_count = ITEMSOF(default_idle_pids_rpm_koffs);
  memcpy(table->idle_pids_rpm_koffs, default_idle_pids_rpm_koffs, sizeof(default_idle_pids_rpm_koffs));
  memcpy(table->idle_valve_to_massair_pid_p, default_idle_valve_to_massair_pid_p, sizeof(default_idle_valve_to_massair_pid_p));
  memcpy(table->idle_valve_to_massair_pid_i, default_idle_valve_to_massair_pid_i, sizeof(default_idle_valve_to_massair_pid_i));
  memcpy(table->idle_valve_to_massair_pid_d, default_idle_valve_to_massair_pid_d, sizeof(default_idle_valve_to_massair_pid_d));

  memcpy(table->idle_valve_to_rpm_pid_p, default_idle_valve_to_rpm_pid_p, sizeof(default_idle_valve_to_rpm_pid_p));
  memcpy(table->idle_valve_to_rpm_pid_i, default_idle_valve_to_rpm_pid_i, sizeof(default_idle_valve_to_rpm_pid_i));
  memcpy(table->idle_valve_to_rpm_pid_d, default_idle_valve_to_rpm_pid_d, sizeof(default_idle_valve_to_rpm_pid_d));

  memcpy(table->idle_ign_to_rpm_pid_p, default_idle_ign_to_rpm_pid_p, sizeof(default_idle_ign_to_rpm_pid_p));
  memcpy(table->idle_ign_to_rpm_pid_i, default_idle_ign_to_rpm_pid_i, sizeof(default_idle_ign_to_rpm_pid_i));
  memcpy(table->idle_ign_to_rpm_pid_d, default_idle_ign_to_rpm_pid_d, sizeof(default_idle_ign_to_rpm_pid_d));

  memcpy(table->idle_rpm_pid_act_1, default_idle_rpm_pid_act_1, sizeof(default_idle_rpm_pid_act_1));
  memcpy(table->idle_rpm_pid_act_2, default_idle_rpm_pid_act_2, sizeof(default_idle_rpm_pid_act_2));

  table->short_term_corr_pid_p = 0.0000f;
  table->short_term_corr_pid_i = 0.0000f;
  table->short_term_corr_pid_d = 0.0000f;

  table->idle_ign_deviation_min = -5.0f;
  table->idle_ign_deviation_max = 12.0f;

  table->idle_ign_fan_low_corr = 1.0f;
  table->idle_ign_fan_high_corr = 2.0f;
  table->idle_air_fan_low_corr = 0.5f;
  table->idle_air_fan_high_corr = 0.8f;

  table->speeds_count = ITEMSOF(default_speeds);
  memcpy(table->speeds, default_speeds, sizeof(default_speeds));
  memcpy(table->idle_rpm_shift, default_idle_rpm_shift, sizeof(default_idle_rpm_shift));

  table->knock_ign_corr_max = -5.0f;
  table->knock_inj_corr_max = 0.00f;
  memcpy(table->knock_noise_level, default_knock_noise_level, sizeof(default_knock_noise_level));
  memcpy(table->knock_threshold, default_knock_threshold, sizeof(default_knock_threshold));
  memcpy(table->knock_zone, default_knock_zone, sizeof(default_knock_zone));
  memcpy(table->knock_gain, default_knock_gain, sizeof(default_knock_gain));
  memcpy(table->knock_filter_frequency, default_knock_filter_frequency, sizeof(default_knock_filter_frequency));
  memcpy(table->knock_cy_level_multiplier, default_knock_cy_level_multiplier, sizeof(default_knock_cy_level_multiplier));

  memcpy(table->tsps_relative_pos, default_tsps_relative_pos, sizeof(default_tsps_relative_pos));
  memcpy(table->tsps_desync_thr, default_tsps_desync_thr, sizeof(default_tsps_desync_thr));
  memcpy(table->idle_ignition_time_by_tps, default_idle_ignition_time_by_tps, sizeof(default_idle_ignition_time_by_tps));
  memcpy(table->idle_econ_delay, default_idle_econ_delay, sizeof(default_idle_econ_delay));
  memcpy(table->start_econ_delay, default_start_econ_delay, sizeof(default_start_econ_delay));

  table->fan_advance_control_low  = -0.2f;
  table->fan_advance_control_mid  =  0.0f;
  table->fan_advance_control_high =  0.2f;
  memcpy(table->fan_advance_control, default_fan_advance_control, sizeof(default_fan_advance_control));

  for(int i = 0; i < ECU_CYLINDERS_COUNT; i++) {
    table->cy_corr_injection[i] = 0;
    table->cy_corr_ignition[i] = 0;
  }
}

void config_default_params(sEcuParams *table)
{
  table->engineVolume = 1770.0f;

  table->isSwitchByExternal = 0;
  table->startupTableNumber = 0;
  table->switchPos1Table = 0;
  table->switchPos0Table = 0;
  table->switchPos2Table = 0;

  table->cutoffRPM = 7000;
  table->cutoffMode = 5;
  table->cutoffAdvance = 10.0f;
  table->cutoffMixture = 12.6f;
  table->oilPressureCutoffRPM = 2000;
  table->speedInputCorrection = 0.91f;
  table->speedOutputCorrection = 0.89f;

  table->useLambdaSensor = 1;
  table->phasedMode = 2;
  table->useKnockSensor = 0;
  table->performAdaptation = 0;
  table->performIdleAdaptation = 0;
  table->isSingleCoil = 0;
  table->isIndividualCoils = 0;
  table->isEconEnabled = 0;

  table->fanHighTemperature = 93;
  table->fanMidTemperature = 91;
  table->fanLowTemperature = 89;

  table->isBluetoothEnabled = 0;
  table->bluetoothPin = 1902;
  strcpy(table->bluetoothName, "BT_ECU");

  table->shiftMode = 0;
  table->shiftThrThr = 80;
  table->shiftRpmThr = 5000;
  table->shiftRpmTill = 2500;
  table->shiftAdvance = 3.0f;
  table->shiftMixture = 12.1f;

  table->useIdleValve = 1;
  table->useShortTermCorr = 0;
  table->useLongTermCorr = 0;

  table->knockIntegratorTime = 22;
  table->learn_cycles_delay_mult = 2.0f;
  table->air_temp_corr_koff_min = 0.25f;
  table->air_temp_corr_koff_max = 0.9f;

  table->tps_voltage_low = 0.525f;
  table->tps_voltage_high = 4.641f;

  table->map_pressure_gain = 19000.0f;
  table->map_pressure_offset = 10000.0f;

}

void config_default_corrections(sEcuCorrections *table)
{
  table->long_term_correction = 0;
  table->idle_correction = 0;

  for(int i = 0; i < TABLE_FILLING_MAX; i++)
    for(int j = 0; j < TABLE_ROTATES_MAX; j++)
      table->ignitions[i][j] = 0.0f;

  for(int i = 0; i < TABLE_PRESSURES_MAX; i++)
    for(int j = 0; j < TABLE_ROTATES_MAX; j++)
      table->filling_gbc_map[i][j] = 0.0f;

  for(int i = 0; i < TABLE_THROTTLES_MAX; i++)
    for(int j = 0; j < TABLE_ROTATES_MAX; j++)
      table->filling_gbc_tps[i][j] = 0.0f;

  for(int i = 0; i < TABLE_TEMPERATURES_MAX; i++)
    table->idle_valve_position[i] = 0.0f;

  for(int i = 0; i < ECU_CYLINDERS_COUNT; i++)
    for(int j = 0; j < TABLE_ROTATES_MAX; j++)
      table->knock_cy_level_multiplier[i][j] = 0.0f;

  for(int i = 0; i < TABLE_FILLING_MAX; i++)
    for(int j = 0; j < TABLE_ROTATES_MAX; j++)
      table->knock_detonation_counter[i][j] = 0.0f;
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
  return flash_page_load(table, sizeof(sEcuTable), CONFIG_OFFSET_TABLES + number);
}

int8_t config_save_table(const sEcuTable *table, uint8_t number)
{
  if(number >= TABLE_SETUPS_MAX)
    return -1;
  return flash_page_save(table, sizeof(sEcuTable), CONFIG_OFFSET_TABLES + number);
}


int8_t config_load_params(sEcuParams *params)
{
  return flash_page_load(params, sizeof(sEcuParams), CONFIG_OFFSET_PARAMS);
}

int8_t config_save_params(const sEcuParams *params)
{
  return flash_page_save(params, sizeof(sEcuParams), CONFIG_OFFSET_PARAMS);
}

int8_t config_load_corrections(sEcuCorrections *table)
{
  return flash_page_load(table, sizeof(sEcuCorrections), CONFIG_OFFSET_CORRECTIONS);
}

int8_t config_save_corrections(const sEcuCorrections *table)
{
  return flash_page_save(table, sizeof(sEcuCorrections), CONFIG_OFFSET_CORRECTIONS);
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
