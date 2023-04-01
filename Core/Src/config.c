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

static const float default_filling_by_map[TABLE_PRESSURES_MAX][TABLE_ROTATES_MAX] = {
    { 0.600f, 0.600f, 0.600f, 0.600f, 0.600f, 0.600f, 0.600f, 0.700f, 0.800f, 0.903f, 0.978f, 0.804f, 0.837f, 0.846f, 0.800f, 0.780f, },
    { 0.600f, 0.600f, 0.600f, 0.600f, 0.600f, 0.597f, 0.571f, 0.666f, 0.785f, 0.884f, 0.680f, 0.709f, 0.564f, 0.462f, 0.500f, 0.500f, },
    { 0.600f, 0.599f, 0.598f, 0.593f, 0.584f, 0.565f, 0.536f, 0.572f, 0.635f, 0.566f, 0.651f, 0.665f, 0.519f, 0.478f, 0.500f, 0.500f, },
    { 0.600f, 0.600f, 0.600f, 0.500f, 0.482f, 0.558f, 0.562f, 0.629f, 0.567f, 0.544f, 0.761f, 0.858f, 0.549f, 0.488f, 0.500f, 0.500f, },
    { 0.600f, 0.600f, 0.615f, 0.456f, 0.445f, 0.533f, 0.554f, 0.674f, 0.625f, 0.623f, 0.759f, 0.772f, 0.669f, 0.501f, 0.510f, 0.500f, },
    { 0.612f, 0.612f, 0.631f, 0.495f, 0.527f, 0.652f, 0.549f, 0.709f, 0.658f, 0.646f, 0.782f, 0.828f, 0.690f, 0.576f, 0.550f, 0.520f, },
    { 0.629f, 0.632f, 0.597f, 0.423f, 0.570f, 0.628f, 0.596f, 0.695f, 0.674f, 0.671f, 0.791f, 0.792f, 0.755f, 0.627f, 0.590f, 0.560f, },
    { 0.603f, 0.636f, 0.499f, 0.573f, 0.659f, 0.647f, 0.619f, 0.739f, 0.707f, 0.691f, 0.800f, 0.881f, 0.837f, 0.693f, 0.650f, 0.590f, },
    { 0.596f, 0.552f, 0.467f, 0.550f, 0.637f, 0.679f, 0.711f, 0.787f, 0.728f, 0.703f, 0.793f, 0.913f, 0.853f, 0.718f, 0.700f, 0.650f, },
    { 0.583f, 0.546f, 0.545f, 0.650f, 0.594f, 0.726f, 0.662f, 0.766f, 0.742f, 0.724f, 0.835f, 0.945f, 0.880f, 0.740f, 0.710f, 0.680f, },
    { 0.584f, 0.557f, 0.666f, 0.603f, 0.602f, 0.753f, 0.668f, 0.759f, 0.797f, 0.745f, 0.848f, 0.954f, 0.901f, 0.756f, 0.720f, 0.700f, },
    { 0.570f, 0.668f, 0.680f, 0.558f, 0.853f, 0.822f, 0.672f, 0.806f, 0.810f, 0.763f, 0.856f, 0.971f, 0.899f, 0.758f, 0.730f, 0.710f, },
    { 0.462f, 0.654f, 0.544f, 0.810f, 0.780f, 0.775f, 0.696f, 0.769f, 0.881f, 0.764f, 0.880f, 1.068f, 0.933f, 0.776f, 0.782f, 0.750f, },
    { 0.500f, 0.781f, 0.847f, 0.742f, 0.763f, 0.728f, 0.697f, 0.821f, 0.912f, 0.794f, 0.885f, 0.947f, 1.017f, 0.937f, 0.849f, 0.790f, },
    { 0.700f, 0.887f, 0.771f, 0.806f, 0.797f, 0.815f, 0.790f, 0.746f, 0.923f, 0.817f, 0.898f, 0.927f, 1.007f, 0.970f, 0.876f, 0.810f, },
    { 0.800f, 0.902f, 0.800f, 0.825f, 0.810f, 0.822f, 0.842f, 0.883f, 0.887f, 0.864f, 1.037f, 1.076f, 0.995f, 0.929f, 0.900f, 0.830f, },
};

static const float default_map_by_thr[TABLE_THROTTLES_MAX][TABLE_ROTATES_MAX] = {
    { 74801, 73248, 56969, 52457, 48733, 38893, 35940, 29365, 30446, 25067, 18769, 19315, 20599, 36896, 18121, 6000, },
    { 52688, 47910, 64953, 64356, 59802, 50076, 48338, 35424, 35721, 31122, 24235, 20069, 18676, 20703, 14847, 13000, },
    { 71711, 64396, 76643, 69063, 58822, 54014, 50497, 41427, 37779, 34096, 25911, 22658, 21359, 27602, 21034, 19000, },
    { 70308, 68500, 85444, 72014, 71414, 58824, 53621, 46691, 41955, 39512, 29128, 25725, 23549, 31414, 27049, 25000, },
    { 64495, 66496, 77325, 85335, 72409, 69832, 66104, 58916, 50061, 47357, 36573, 29613, 34957, 38590, 34375, 32000, },
    { 60190, 60773, 99429, 77943, 85724, 77483, 67906, 68577, 59441, 57182, 45902, 37758, 39061, 41956, 40756, 38000, },
    { 65000, 60039, 58184, 76987, 93969, 92417, 72464, 76434, 71100, 68623, 56880, 47425, 46270, 47941, 46567, 44000, },
    { 70000, 65000, 55000, 62277, 75424, 79173, 85133, 82023, 79002, 77171, 66079, 57292, 54071, 53840, 53545, 51000, },
    { 80000, 70000, 60000, 58528, 66291, 84845, 91349, 87353, 86081, 84660, 76358, 70265, 59764, 58648, 59570, 57000, },
    { 90000, 80000, 70000, 63928, 65889, 67788, 85993, 96161, 89563, 89935, 83531, 78311, 66739, 63531, 65148, 63000, },
    { 100000, 90000, 75000, 70335, 71533, 70660, 77880, 94113, 93034, 92725, 89185, 86569, 78852, 70223, 72318, 70000, },
    { 100000, 90000, 80031, 77309, 76327, 76155, 79850, 86412, 98343, 96330, 91963, 92716, 87599, 78772, 77901, 76000, },
    { 100000, 95000, 90063, 82862, 82108, 82013, 86269, 94777, 96003, 96914, 95218, 94465, 92995, 90230, 83611, 82000, },
    { 100000, 97000, 95000, 89000, 89000, 89000, 90691, 98846, 97299, 97425, 94880, 95501, 94307, 93779, 89893, 89000, },
    { 100000, 100000, 100000, 95000, 95000, 95000, 95682, 96993, 97447, 96512, 96656, 95936, 95863, 94786, 94869, 95000, },
    { 101000, 101000, 101000, 101000, 101000, 100599, 97228, 96783, 96540, 96576, 96236, 96093, 95816, 95223, 96681, 101000, },
};

static const float default_part_load_ignitions[TABLE_FILLING_MAX][TABLE_ROTATES_MAX] = {
    { 14.7f, 16.6f, 18.8f, 22.4f, 26.8f, 33.1f, 39.4f, 41.8f, 43.1f, 43.8f, 44.2f, 44.3f, 44.3f, 44.6f, 44.9f, 45.3f, },
    { 16.5f, 18.8f, 21.0f, 24.5f, 28.5f, 34.0f, 39.6f, 41.9f, 43.1f, 43.8f, 44.2f, 44.3f, 44.3f, 44.6f, 44.9f, 45.3f, },
    { 18.2f, 20.3f, 22.3f, 25.5f, 29.2f, 34.4f, 39.6f, 41.8f, 43.0f, 43.7f, 44.1f, 44.2f, 44.2f, 44.6f, 44.9f, 45.3f, },
    { 18.7f, 20.7f, 22.5f, 25.4f, 29.0f, 34.1f, 39.2f, 41.4f, 42.4f, 43.1f, 43.4f, 43.5f, 43.6f, 44.1f, 44.4f, 44.8f, },
    { 17.7f, 19.7f, 21.6f, 24.5f, 28.0f, 32.9f, 38.0f, 40.2f, 41.3f, 41.9f, 42.3f, 42.4f, 42.5f, 43.0f, 43.3f, 43.7f, },
    { 15.2f, 17.7f, 19.4f, 22.4f, 26.1f, 31.0f, 36.0f, 38.4f, 39.6f, 40.5f, 41.0f, 41.1f, 41.3f, 41.8f, 42.1f, 42.4f, },
    { 12.1f, 14.5f, 16.1f, 18.9f, 23.0f, 28.2f, 33.2f, 35.8f, 37.3f, 38.4f, 39.2f, 39.7f, 40.0f, 40.5f, 40.9f, 41.2f, },
    { 9.8f, 11.6f, 13.0f, 15.5f, 19.3f, 24.2f, 29.3f, 32.4f, 34.5f, 36.3f, 37.4f, 38.1f, 38.5f, 39.1f, 39.4f, 39.7f, },
    { 8.0f, 9.3f, 10.5f, 12.6f, 15.7f, 19.9f, 24.8f, 28.5f, 31.3f, 33.8f, 35.2f, 35.5f, 35.7f, 36.1f, 36.3f, 36.7f, },
    { 6.7f, 7.8f, 8.9f, 10.6f, 13.1f, 17.0f, 22.0f, 26.0f, 28.9f, 31.5f, 32.8f, 32.5f, 32.4f, 32.8f, 32.9f, 33.3f, },
    { 6.2f, 7.0f, 7.8f, 9.2f, 11.3f, 15.0f, 20.2f, 24.5f, 27.4f, 29.7f, 30.7f, 30.1f, 29.9f, 30.3f, 30.6f, 31.0f, },
    { 6.0f, 6.6f, 7.3f, 8.4f, 10.1f, 13.3f, 18.3f, 22.8f, 25.9f, 28.2f, 29.0f, 28.2f, 28.0f, 28.6f, 28.9f, 29.3f, },
    { 6.0f, 6.6f, 7.1f, 8.0f, 9.4f, 11.9f, 16.1f, 20.3f, 23.4f, 25.9f, 26.9f, 26.2f, 26.0f, 26.5f, 26.9f, 27.2f, },
    { 6.0f, 6.6f, 7.1f, 7.9f, 9.0f, 10.9f, 14.1f, 17.9f, 21.1f, 23.9f, 25.0f, 24.4f, 24.2f, 24.6f, 24.9f, 25.3f, },
    { 5.9f, 6.5f, 7.0f, 7.8f, 8.6f, 10.0f, 12.5f, 16.1f, 19.6f, 21.9f, 23.1f, 22.5f, 22.4f, 22.8f, 22.9f, 23.5f, },
    { 5.8f, 6.4f, 6.9f, 7.5f, 8.2f, 9.2f, 11.1f, 14.4f, 18.1f, 20.1f, 21.4f, 20.9f, 20.7f, 21.0f, 21.1f, 21.7f, },
};

static const float default_part_load_fuel_mixtures[TABLE_FILLING_MAX][TABLE_ROTATES_MAX] = {
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

static const float default_part_load_injection_phase[TABLE_FILLING_MAX][TABLE_ROTATES_MAX] = {
    { 113, 116, 121, 128, 136, 144, 153, 160, 165, 168, 169, 167, 163, 159, 154, 152, },
    { 117, 120, 125, 132, 140, 148, 157, 164, 170, 173, 173, 172, 168, 164, 160, 157, },
    { 124, 127, 132, 139, 147, 155, 164, 171, 177, 181, 181, 180, 177, 172, 169, 166, },
    { 134, 137, 142, 149, 158, 166, 175, 183, 188, 192, 193, 192, 189, 185, 182, 179, },
    { 149, 152, 157, 164, 173, 181, 190, 198, 204, 208, 209, 208, 206, 202, 199, 197, },
    { 170, 173, 178, 185, 193, 201, 210, 218, 224, 228, 230, 229, 227, 224, 221, 219, },
    { 196, 199, 204, 211, 218, 227, 235, 242, 249, 253, 255, 255, 253, 250, 248, 246, },
    { 230, 232, 237, 243, 250, 258, 266, 273, 279, 283, 285, 285, 284, 282, 280, 278, },
    { 270, 273, 277, 282, 289, 296, 303, 309, 315, 318, 321, 321, 320, 319, 317, 316, },
    { 318, 320, 323, 328, 334, 340, 346, 351, 356, 359, 361, 362, 361, 360, 359, 358, },
    { 370, 372, 375, 379, 383, 388, 393, 398, 402, 404, 406, 407, 406, 405, 405, 404, },
    { 425, 426, 428, 431, 434, 438, 442, 445, 448, 451, 452, 452, 452, 452, 451, 451, },
    { 475, 476, 478, 480, 482, 485, 488, 490, 492, 494, 495, 495, 495, 495, 494, 494, },
    { 517, 518, 519, 520, 522, 524, 525, 527, 529, 530, 530, 531, 530, 530, 530, 530, },
    { 547, 548, 548, 549, 550, 551, 552, 554, 554, 555, 556, 556, 556, 556, 555, 555, },
    { 565, 565, 565, 566, 567, 567, 568, 569, 569, 570, 570, 570, 570, 570, 570, 570, },
};

static const float default_full_throttle_ignitions[TABLE_FILLING_MAX][TABLE_ROTATES_MAX] = {
    { 14.7f, 16.6f, 18.8f, 22.4f, 26.8f, 33.1f, 39.4f, 41.8f, 43.1f, 43.8f, 44.2f, 44.3f, 44.3f, 44.6f, 44.9f, 45.3f, },
    { 16.5f, 18.8f, 21.0f, 24.5f, 28.5f, 34.0f, 39.6f, 41.9f, 43.1f, 43.8f, 44.2f, 44.3f, 44.3f, 44.6f, 44.9f, 45.3f, },
    { 18.2f, 20.3f, 22.3f, 25.5f, 29.2f, 34.4f, 39.6f, 41.8f, 43.0f, 43.7f, 44.1f, 44.2f, 44.2f, 44.6f, 44.9f, 45.3f, },
    { 18.7f, 20.7f, 22.5f, 25.4f, 29.0f, 34.1f, 39.2f, 41.4f, 42.4f, 43.1f, 43.4f, 43.5f, 43.6f, 44.1f, 44.4f, 44.8f, },
    { 17.7f, 19.7f, 21.6f, 24.5f, 28.0f, 32.9f, 38.0f, 40.2f, 41.3f, 41.9f, 42.3f, 42.4f, 42.5f, 43.0f, 43.3f, 43.7f, },
    { 15.2f, 17.7f, 19.4f, 22.4f, 26.1f, 31.0f, 36.0f, 38.4f, 39.6f, 40.5f, 41.0f, 41.1f, 41.3f, 41.8f, 42.1f, 42.4f, },
    { 12.1f, 14.5f, 16.1f, 18.9f, 23.0f, 28.2f, 33.2f, 35.8f, 37.3f, 38.4f, 39.2f, 39.7f, 40.0f, 40.5f, 40.9f, 41.2f, },
    { 9.8f, 11.6f, 13.0f, 15.5f, 19.3f, 24.2f, 29.3f, 32.4f, 34.5f, 36.3f, 37.4f, 38.1f, 38.5f, 39.1f, 39.4f, 39.7f, },
    { 8.0f, 9.3f, 10.5f, 12.6f, 15.7f, 19.9f, 24.8f, 28.5f, 31.3f, 33.8f, 35.2f, 35.5f, 35.7f, 36.1f, 36.3f, 36.7f, },
    { 6.7f, 7.8f, 8.9f, 10.6f, 13.1f, 17.0f, 22.0f, 26.0f, 28.9f, 31.5f, 32.8f, 32.5f, 32.4f, 32.8f, 32.9f, 33.3f, },
    { 6.2f, 7.0f, 7.8f, 9.2f, 11.3f, 15.0f, 20.2f, 24.5f, 27.4f, 29.7f, 30.7f, 30.1f, 29.9f, 30.3f, 30.6f, 31.0f, },
    { 6.0f, 6.6f, 7.3f, 8.4f, 10.1f, 13.3f, 18.3f, 22.8f, 25.9f, 28.2f, 29.0f, 28.2f, 28.0f, 28.6f, 28.9f, 29.3f, },
    { 6.0f, 6.6f, 7.1f, 8.0f, 9.4f, 11.9f, 16.1f, 20.3f, 23.4f, 25.9f, 26.9f, 26.2f, 26.0f, 26.5f, 26.9f, 27.2f, },
    { 6.0f, 6.6f, 7.1f, 7.9f, 9.0f, 10.9f, 14.1f, 17.9f, 21.1f, 23.9f, 25.0f, 24.4f, 24.2f, 24.6f, 24.9f, 25.3f, },
    { 5.9f, 6.5f, 7.0f, 7.8f, 8.6f, 10.0f, 12.5f, 16.1f, 19.6f, 21.9f, 23.1f, 22.5f, 22.4f, 22.8f, 22.9f, 23.5f, },
    { 5.8f, 6.4f, 6.9f, 7.5f, 8.2f, 9.2f, 11.1f, 14.4f, 18.1f, 20.1f, 21.4f, 20.9f, 20.7f, 21.0f, 21.1f, 21.7f, },
};

static const float default_full_throttle_fuel_mixtures[TABLE_FILLING_MAX][TABLE_ROTATES_MAX] = {
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

static const float default_full_throttle_injection_phase[TABLE_FILLING_MAX][TABLE_ROTATES_MAX] = {
    { 113, 116, 121, 128, 136, 144, 153, 160, 165, 168, 169, 167, 163, 159, 154, 152, },
    { 117, 120, 125, 132, 140, 148, 157, 164, 170, 173, 173, 172, 168, 164, 160, 157, },
    { 124, 127, 132, 139, 147, 155, 164, 171, 177, 181, 181, 180, 177, 172, 169, 166, },
    { 134, 137, 142, 149, 158, 166, 175, 183, 188, 192, 193, 192, 189, 185, 182, 179, },
    { 149, 152, 157, 164, 173, 181, 190, 198, 204, 208, 209, 208, 206, 202, 199, 197, },
    { 170, 173, 178, 185, 193, 201, 210, 218, 224, 228, 230, 229, 227, 224, 221, 219, },
    { 196, 199, 204, 211, 218, 227, 235, 242, 249, 253, 255, 255, 253, 250, 248, 246, },
    { 230, 232, 237, 243, 250, 258, 266, 273, 279, 283, 285, 285, 284, 282, 280, 278, },
    { 270, 273, 277, 282, 289, 296, 303, 309, 315, 318, 321, 321, 320, 319, 317, 316, },
    { 318, 320, 323, 328, 334, 340, 346, 351, 356, 359, 361, 362, 361, 360, 359, 358, },
    { 370, 372, 375, 379, 383, 388, 393, 398, 402, 404, 406, 407, 406, 405, 405, 404, },
    { 425, 426, 428, 431, 434, 438, 442, 445, 448, 451, 452, 452, 452, 452, 451, 451, },
    { 475, 476, 478, 480, 482, 485, 488, 490, 492, 494, 495, 495, 495, 495, 494, 494, },
    { 517, 518, 519, 520, 522, 524, 525, 527, 529, 530, 530, 531, 530, 530, 530, 530, },
    { 547, 548, 548, 549, 550, 551, 552, 554, 554, 555, 556, 556, 556, 556, 555, 555, },
    { 565, 565, 565, 566, 567, 567, 568, 569, 569, 570, 570, 570, 570, 570, 570, 570, },
};

static const float default_part_load_fuel_mixtures_lpf[TABLE_ROTATES_MAX] = {
    0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f,
    0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f,
};

static const float default_switch_ign_lpf[TABLE_ROTATES_MAX] = {
    0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f,
    0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f,
};

static const float default_switch_phase_lpf[TABLE_ROTATES_MAX] = {
    0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f,
    0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f,
};

static const float default_injection_phase_lpf[TABLE_ROTATES_MAX] = {
    0.100f, 0.100f, 0.100f, 0.100f, 0.100f, 0.100f, 0.100f, 0.100f,
    0.100f, 0.100f, 0.100f, 0.100f, 0.100f, 0.100f, 0.100f, 0.100f,
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
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.9f, 0.8f, 0.7f,
    0.6f, 0.4f, 0.2f, 0.0f, 0.0f, 0.7f, 1.0f, 1.0f
};

static const float default_warmup_mix_corrs[TABLE_TEMPERATURES_MAX] = {
    0.20f, 0.15f, 0.10f, 0.05f, 0.04f, 0.03f, 0.02f, 0.02f,
    0.01f, 0.01f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f,
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
    //1600, 1400, 1200, 1000, 900, 800, 750, 700,
    //650, 600, 600, 600, 600, 600, 600, 600

    //600, 520, 450, 400, 350, 300, 270, 250,
    //220, 200, 200, 200, 200, 200, 200, 200

    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
};

static const float default_start_large_filling[TABLE_TEMPERATURES_MAX] = {
    1200, 1000, 880, 720, 600, 500, 450, 400,
    360, 340, 330, 320, 320, 320, 320, 320,
};

static const float default_start_small_filling[TABLE_TEMPERATURES_MAX] = {
    1200, 900, 750, 580, 430, 360, 320, 280,
    260, 240, 220, 200, 200, 200, 200, 200,
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
    100.0f, 100.0f, 100.0f, 97.0f, 90.0f, 80.0f, 70.0f, 63.0f,
    58.0f, 54.0f, 52.0f, 50.0f, 50.0f, 52.0f, 55.0f, 60.0f,
};

static const float default_idle_wish_rotates[TABLE_TEMPERATURES_MAX] = {
    1700, 1700, 1700, 1700, 1650, 1600, 1500, 1100,
    1100, 1100, 1100, 1100, 1100, 1250, 1350, 1450,
};

static const float default_idle_valve_to_rpm[TABLE_TEMPERATURES_MAX][TABLE_ROTATES_MAX] = {
    { 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, },
    { 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, },
    { 62, 80, 90, 90, 90, 90, 90, 80, 80, 80, 80, 55, 55, 55, 55, 55, },
    { 74, 90, 90, 90, 90, 90, 90, 90, 80, 80, 80, 50, 50, 50, 50, 50, },
    { 66, 46, 51, 57, 80, 40, 40, 40, 47, 47, 47, 47, 47, 47, 47, 47, },
    { 42, 42, 42, 42, 44, 43, 43, 42, 42, 42, 42, 42, 42, 42, 42, 42, },
    { 40, 40, 40, 38, 59, 29, 28, 28, 40, 40, 40, 40, 40, 40, 40, 40, },
    { 43, 43, 40, 40, 51, 51, 39, 39, 40, 40, 43, 43, 43, 43, 43, 43, },
    { 45, 40, 40, 40, 40, 41, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, },
    { 69, 42, 40, 40, 40, 40, 34, 40, 40, 40, 40, 40, 40, 40, 40, 35, },
    { 54, 40, 51, 43, 43, 33, 40, 40, 40, 40, 40, 40, 40, 40, 40, 30, },
    { 39, 40, 39, 38, 39, 35, 40, 40, 40, 40, 40, 40, 40, 40, 40, 30, },
    { 49, 40, 35, 35, 35, 28, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, },
    { 47, 47, 47, 47, 47, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, },
    { 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 30, 30, 30, },
    { 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 30, 30, },
};

static const float default_idle_wish_massair[TABLE_TEMPERATURES_MAX] = {
    35.0f, 35.0f, 35.0f, 35.0f, 33.0f, 31.0f, 24.0f, 18.0f,
    17.0f, 17.0f, 17.0f, 16.0f, 16.0f, 17.0f, 18.5f, 19.0f,
};

static const float default_idle_wish_ignition_static[TABLE_ROTATES_MAX] = {
    14.0f, 16.0f, 18.0f, 21.0f, 25.0f, 25.7f, 23.0f, 19.0f,
    16.0f, 15.0f, 14.0f, 13.0f, 13.5f, 15.6f, 18.0f, 21.0f,
};

static const float default_idle_wish_ignition[TABLE_TEMPERATURES_MAX] = {
    35.0f, 35.0f, 35.0f, 35.0f, 34.0f, 32.0f, 30.0f, 19.0f,
    17.0f, 14.0f, 10.0f, 10.0f, 10.0f, 15.0f, 18.0f, 19.0f,
};

static const float default_idle_rpm_shift_speeds[TABLE_SPEEDS_MAX] = {
    0, 3, 10, 20, 30, 40, 50, 60,
    70, 80, 90, 100, 110, 120, 130, 140
};

static const float default_idle_rpm_shift[TABLE_SPEEDS_MAX] = {
    0, 20, 40, 40, 60, 70, 75, 80,
    85, 90, 90, 95, 100, 100, 100, 100,
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

static const float default_idle_pids_rpm_koffs[TABLE_ROTATES_MAX] = {
    0.45f, 0.53f, 0.61f, 0.69f, 0.77f, 0.85f, 0.93f, 1.00f,
    1.08f, 1.16f, 1.24f, 1.32f, 1.40f, 1.48f, 1.56f, 1.64f
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
    0.0020f, 0.0020f, 0.0020f, 0.0020f, 0.0020f, 0.0020f, 0.0020f, 0.0020f,
    0.0020f, 0.0020f, 0.0020f, 0.0020f, 0.0020f, 0.0020f, 0.0020f, 0.0020f,
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
    0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f,
    0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f,
};

static const float default_idle_rpm_pid_act_1[TABLE_TEMPERATURES_MAX] = {
    0.400f, 0.400f, 0.400f, 0.400f, 0.400f, 0.400f, 0.400f, 0.400f,
    0.380f, 0.300f, 0.280f, 0.280f, 0.280f, 0.280f, 0.280f, 0.280f,
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

void config_default_table(sEcuTable *table, uint8_t number)
{
  memset(table, 0, sizeof(sEcuTable));

  sprintf(table->name, "Default %d", number + 1);

  table->inj_channel = InjectorChannel1;

  //Bosch 0280156095 - 315 cc/min
  //Bosch 0280155746 - 200 cc/min
  //BMW 03762FA - 180 cc/min
  table->injector_performance = 200.0f;

  table->is_fuel_phase_by_end = 1;
  table->is_fuel_pressure_const = 0;
  table->is_full_thr_used = 0;
  table->enrichment_ph_sync_enabled = 0;
  table->enrichment_ph_async_enabled = 0;
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

  table->idle_rotates_count = ITEMSOF(default_idle_rotates);
  memcpy(table->idle_rotates, default_idle_rotates, sizeof(default_idle_rotates));

  table->throttles_count = ITEMSOF(default_throttles);
  memcpy(table->throttles, default_throttles, sizeof(default_throttles));

  memcpy(table->fill_by_map, default_filling_by_map, sizeof(default_filling_by_map));
  memcpy(table->map_by_thr, default_map_by_thr, sizeof(default_map_by_thr));

  memcpy(table->enrichment_temp_mult, default_enrichment_temp_mult, sizeof(default_enrichment_temp_mult));

  table->fillings_count = ITEMSOF(default_fillings);
  memcpy(table->fillings, default_fillings, sizeof(default_fillings));

  memcpy(table->injection_phase_lpf, default_injection_phase_lpf, sizeof(default_injection_phase_lpf));

  memcpy(table->main.full_throttle.ignitions, default_full_throttle_ignitions, sizeof(default_full_throttle_ignitions));
  memcpy(table->main.full_throttle.fuel_mixtures, default_full_throttle_fuel_mixtures, sizeof(default_full_throttle_fuel_mixtures));
  memcpy(table->main.full_throttle.injection_phase, default_full_throttle_injection_phase, sizeof(default_full_throttle_injection_phase));

  memcpy(table->main.part_load.ignitions, default_part_load_ignitions, sizeof(default_part_load_ignitions));
  memcpy(table->main.part_load.fuel_mixtures, default_part_load_fuel_mixtures, sizeof(default_part_load_fuel_mixtures));
  memcpy(table->main.part_load.injection_phase, default_part_load_injection_phase, sizeof(default_part_load_injection_phase));

  memcpy(table->main.switch_mix_lpf, default_part_load_fuel_mixtures_lpf, sizeof(default_part_load_fuel_mixtures_lpf));
  memcpy(table->main.switch_ign_lpf, default_switch_ign_lpf, sizeof(default_switch_ign_lpf));
  memcpy(table->main.switch_phase_lpf, default_switch_phase_lpf, sizeof(default_switch_phase_lpf));

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

  table->idle_pids_rpm_koffs_count = ITEMSOF(default_idle_pids_rpm_koffs);
  memcpy(table->idle_pids_rpm_koffs, default_idle_pids_rpm_koffs, sizeof(default_idle_pids_rpm_koffs));
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

  table->idle_ign_fan_low_corr = 3.0f;
  table->idle_ign_fan_high_corr = 5.0f;
  table->idle_air_fan_low_corr = 1.0f;
  table->idle_air_fan_high_corr = 2.0f;

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

  memcpy(table->tsps_relative_pos, default_tsps_relative_pos, sizeof(default_tsps_relative_pos));
  memcpy(table->tsps_desync_thr, default_tsps_desync_thr, sizeof(default_tsps_desync_thr));
  memcpy(table->idle_ignition_time_by_tps, default_idle_ignition_time_by_tps, sizeof(default_idle_ignition_time_by_tps));
  memcpy(table->idle_econ_delay, default_idle_econ_delay, sizeof(default_idle_econ_delay));
  memcpy(table->start_econ_delay, default_start_econ_delay, sizeof(default_start_econ_delay));

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
  table->performIdleAdaptation = 0;
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
