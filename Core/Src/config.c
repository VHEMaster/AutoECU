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

static const float default_pressures[TABLE_PRESSURES_MAX] = {
    0, 7300, 14700, 22000, 29300, 36700, 44000, 51300,
    58600, 66000, 73300, 80600, 88000, 95300, 12700, 110000
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
    32, 62, 92, 122, 151, 181, 211, 241,
    271, 301, 331, 361, 390, 420, 450, 480,

};

static const float default_ignitions[TABLE_FILLING_MAX][TABLE_ROTATES_MAX] = {
    { 14.5f, 16.1f, 18.0f, 20.9f, 24.7f, 29.7f, 37.8f, 40.9f, 42.6f, 43.4f, 44.0f, 44.4f, 44.3f, 44.3f, 44.8f, 45.0f },
    { 16.1f, 18.0f, 20.1f, 23.1f, 26.6f, 31.0f, 38.1f, 41.0f, 42.6f, 43.4f, 44.0f, 44.4f, 44.3f, 44.3f, 44.8f, 45.0f },
    { 17.8f, 19.7f, 21.5f, 24.2f, 27.6f, 31.7f, 38.3f, 41.1f, 42.6f, 43.4f, 44.0f, 44.4f, 44.3f, 44.3f, 44.8f, 45.0f },
    { 18.7f, 20.5f, 22.2f, 24.6f, 27.7f, 31.8f, 38.2f, 40.9f, 42.4f, 43.2f, 43.7f, 44.1f, 44.0f, 44.1f, 44.7f, 44.9f },
    { 18.7f, 20.4f, 21.9f, 24.3f, 27.3f, 31.3f, 37.6f, 40.3f, 41.7f, 42.4f, 42.9f, 43.2f, 43.1f, 43.3f, 43.0f, 44.2f },
    { 17.4f, 19.4f, 20.9f, 23.2f, 26.2f, 30.1f, 36.3f, 39.1f, 40.6f, 41.3f, 41.9f, 42.2f, 42.1f, 42.3f, 42.9f, 43.1f },
    { 14.9f, 17.2f, 18.7f, 21.1f, 24.3f, 28.4f, 34.3f, 37.3f, 38.9f, 39.8f, 40.6f, 41.0f, 41.0f, 41.3f, 41.8f, 42.0f },
    { 12.2f, 14.5f, 15.9f, 17.9f, 21.5f, 26.0f, 31.9f, 34.0f, 36.8f, 37.2f, 38.9f, 39.5f, 39.9f, 40.2f, 40.8f, 41.0f },
    { 10.2f, 11.9f, 13.1f, 14.9f, 18.3f, 22.8f, 28.4f, 31.9f, 34.2f, 35.4f, 37.2f, 38.1f, 38.7f, 39.1f, 39.7f, 39.9f },
    { 8.5f, 9.7f, 10.8f, 12.4f, 15.3f, 19.2f, 24.0f, 27.9f, 30.0f, 33.2f, 35.4f, 36.5f, 36.7f, 37.0f, 37.5f, 37.7f },
    { 7.1f, 8.2f, 9.2f, 10.6f, 12.9f, 16.1f, 20.8f, 24.0f, 28.3f, 30.8f, 33.4f, 34.2f, 33.7f, 33.9f, 34.2f, 34.3f },
    { 6.3f, 7.2f, 8.1f, 9.3f, 11.1f, 14.0f, 18.0f, 23.3f, 26.9f, 29.1f, 31.6f, 32.0f, 31.0f, 31.3f, 31.7f, 31.8f },
    { 6.1f, 6.7f, 7.3f, 8.4f, 9.9f, 12.5f, 17.5f, 22.0f, 25.9f, 27.9f, 30.1f, 30.3f, 29.2f, 29.4f, 30.0f, 30.2f },
    { 6.0f, 6.5f, 7.1f, 7.9f, 9.1f, 11.3f, 15.8f, 20.3f, 24.4f, 26.6f, 28.7f, 28.8f, 27.6f, 27.9f, 28.6f, 28.8f },
    { 6.0f, 6.5f, 7.0f, 7.7f, 8.7f, 10.5f, 14.1f, 18.2f, 22.1f, 24.5f, 26.6f, 27.0f, 25.9f, 26.2f, 26.8f, 27.0f },
    { 6.0f, 6.5f, 7.0f, 7.6f, 8.5f, 9.9f, 12.3f, 15.8f, 19.5f, 22.2f, 24.8f, 25.1f, 24.0f, 24.3f, 24.8f, 25.0f },

};

static const float default_filling_by_map[TABLE_PRESSURES_MAX][TABLE_ROTATES_MAX] = {
    { 24.23f, 24.23f, 24.23f, 24.23f, 24.23f, 24.23f, 24.23f, 24.23f, 24.23f, 24.23f, 24.23f, 24.23f, 24.23f, 24.23f, 24.23f, 24.23 },
    { 48.47f, 48.47f, 48.47f, 48.47f, 48.47f, 48.47f, 48.47f, 48.47f, 48.47f, 48.47f, 48.47f, 48.47f, 48.47f, 48.47f, 48.47f, 48.47 },
    { 72.7f, 72.7f, 72.7f, 72.7f, 72.7f, 72.7f, 72.7f, 72.7f, 72.7f, 72.7f, 72.7f, 72.7f, 72.7f, 72.7f, 72.7f, 72.7 },
    { 96.94f, 96.94f, 96.94f, 96.94f, 96.94f, 96.94f, 96.94f, 96.94f, 96.94f, 96.94f, 96.94f, 96.94f, 96.94f, 96.94f, 96.94f, 96.94 },
    { 121.17f, 121.17f, 121.17f, 121.17f, 121.17f, 121.17f, 121.17f, 121.17f, 121.17f, 121.17f, 121.17f, 121.17f, 121.17f, 121.17f, 121.17f, 121.17 },
    { 145.41f, 145.41f, 145.41f, 145.41f, 145.41f, 145.41f, 145.41f, 145.41f, 145.41f, 145.41f, 145.41f, 145.41f, 145.41f, 145.41f, 145.41f, 145.41 },
    { 152.68f, 169.64f, 169.64f, 169.64f, 169.64f, 169.64f, 169.64f, 169.64f, 169.64f, 169.64f, 169.64f, 169.64f, 169.64f, 169.64f, 169.64f, 169.64 },
    { 155.1f, 174.49f, 193.88f, 193.88f, 193.88f, 193.88f, 193.88f, 193.88f, 193.88f, 193.88f, 193.88f, 193.88f, 193.88f, 193.88f, 193.88f, 193.88 },
    { 152.68f, 174.49f, 196.3f, 218.11f, 218.11f, 218.11f, 218.11f, 218.11f, 218.11f, 218.11f, 218.11f, 218.11f, 218.11f, 218.11f, 218.11f, 218.11 },
    { 145.4f, 169.64f, 193.87f, 242.34f, 242.34f, 242.34f, 242.34f, 242.34f, 242.34f, 242.34f, 242.34f, 242.34f, 242.34f, 242.34f, 242.34f, 242.34 },
    { 133.29f, 159.95f, 186.61f, 239.92f, 239.92f, 266.58f, 266.58f, 266.58f, 266.58f, 266.58f, 266.58f, 266.58f, 266.58f, 266.58f, 266.58f, 266.58 },
    { 116.32f, 145.41f, 174.49f, 232.65f, 261.73f, 290.81f, 290.81f, 290.81f, 290.81f, 290.81f, 290.81f, 290.81f, 290.81f, 290.81f, 290.81f, 290.81 },
    { 126.02f, 141.77f, 157.53f, 220.54f, 252.04f, 315.05f, 315.05f, 315.05f, 315.05f, 315.05f, 315.05f, 315.05f, 315.05f, 315.05f, 315.05f, 315.05 },
    { 135.71f, 135.71f, 135.71f, 203.57f, 271.42f, 339.28f, 339.28f, 339.28f, 339.28f, 339.28f, 339.28f, 339.28f, 339.28f, 339.28f, 339.28f, 339.28 },
    { 145.41f, 145.41f, 145.41f, 218.11f, 290.82f, 363.52f, 363.52f, 363.52f, 363.52f, 363.52f, 363.52f, 363.52f, 363.52f, 363.52f, 363.52f, 363.52 },
    { 155.1f, 155.1f, 155.1f, 232.65f, 310.2f, 387.75f, 387.75f, 387.75f, 387.75f, 387.75f, 387.75f, 387.75f, 387.75f, 387.75f, 387.75f, 387.75 }
};

static const float default_filling_by_thr[TABLE_THROTTLES_MAX][TABLE_ROTATES_MAX] = {
    { 24.23f, 24.23f, 24.23f, 24.23f, 24.23f, 24.23f, 24.23f, 24.23f, 24.23f, 24.23f, 24.23f, 24.23f, 24.23f, 24.23f, 24.23f, 24.23 },
    { 48.47f, 48.47f, 48.47f, 48.47f, 48.47f, 48.47f, 48.47f, 48.47f, 48.47f, 48.47f, 48.47f, 48.47f, 48.47f, 48.47f, 48.47f, 48.47 },
    { 72.7f, 72.7f, 72.7f, 72.7f, 72.7f, 72.7f, 72.7f, 72.7f, 72.7f, 72.7f, 72.7f, 72.7f, 72.7f, 72.7f, 72.7f, 72.7 },
    { 96.94f, 96.94f, 96.94f, 96.94f, 96.94f, 96.94f, 96.94f, 96.94f, 96.94f, 96.94f, 96.94f, 96.94f, 96.94f, 96.94f, 96.94f, 96.94 },
    { 121.17f, 121.17f, 121.17f, 121.17f, 121.17f, 121.17f, 121.17f, 121.17f, 121.17f, 121.17f, 121.17f, 121.17f, 121.17f, 121.17f, 121.17f, 121.17 },
    { 145.41f, 145.41f, 145.41f, 145.41f, 145.41f, 145.41f, 145.41f, 145.41f, 145.41f, 145.41f, 145.41f, 145.41f, 145.41f, 145.41f, 145.41f, 145.41 },
    { 152.68f, 169.64f, 169.64f, 169.64f, 169.64f, 169.64f, 169.64f, 169.64f, 169.64f, 169.64f, 169.64f, 169.64f, 169.64f, 169.64f, 169.64f, 169.64 },
    { 155.1f, 174.49f, 193.88f, 193.88f, 193.88f, 193.88f, 193.88f, 193.88f, 193.88f, 193.88f, 193.88f, 193.88f, 193.88f, 193.88f, 193.88f, 193.88 },
    { 152.68f, 174.49f, 196.3f, 218.11f, 218.11f, 218.11f, 218.11f, 218.11f, 218.11f, 218.11f, 218.11f, 218.11f, 218.11f, 218.11f, 218.11f, 218.11 },
    { 145.4f, 169.64f, 193.87f, 242.34f, 242.34f, 242.34f, 242.34f, 242.34f, 242.34f, 242.34f, 242.34f, 242.34f, 242.34f, 242.34f, 242.34f, 242.34 },
    { 133.29f, 159.95f, 186.61f, 239.92f, 239.92f, 266.58f, 266.58f, 266.58f, 266.58f, 266.58f, 266.58f, 266.58f, 266.58f, 266.58f, 266.58f, 266.58 },
    { 116.32f, 145.41f, 174.49f, 232.65f, 261.73f, 290.81f, 290.81f, 290.81f, 290.81f, 290.81f, 290.81f, 290.81f, 290.81f, 290.81f, 290.81f, 290.81 },
    { 126.02f, 141.77f, 157.53f, 220.54f, 252.04f, 315.05f, 315.05f, 315.05f, 315.05f, 315.05f, 315.05f, 315.05f, 315.05f, 315.05f, 315.05f, 315.05 },
    { 135.71f, 135.71f, 135.71f, 203.57f, 271.42f, 339.28f, 339.28f, 339.28f, 339.28f, 339.28f, 339.28f, 339.28f, 339.28f, 339.28f, 339.28f, 339.28 },
    { 145.41f, 145.41f, 145.41f, 218.11f, 290.82f, 363.52f, 363.52f, 363.52f, 363.52f, 363.52f, 363.52f, 363.52f, 363.52f, 363.52f, 363.52f, 363.52 },
    { 155.1f, 155.1f, 155.1f, 232.65f, 310.2f, 387.75f, 387.75f, 387.75f, 387.75f, 387.75f, 387.75f, 387.75f, 387.75f, 387.75f, 387.75f, 387.75 }
};

static const float default_fuel_mixtures[TABLE_FILLING_MAX][TABLE_ROTATES_MAX] = {
    { 14.1f, 14.1f, 14.1f, 14.1f, 14.1f, 14.1f, 14.1f, 14.1f, 14.1f, 14.1f, 14.1f, 14.1f, 14.1f, 14.1f, 14.1f, 14.1f },
    { 14.7f, 14.7f, 14.7f, 14.6f, 14.5f, 14.4f, 14.3f, 14.3f, 14.3f, 14.3f, 14.3f, 14.3f, 14.3f, 14.3f, 14.3f, 14.3f },
    { 14.7f, 14.7f, 14.7f, 14.7f, 14.6f, 14.5f, 14.4f, 14.4f, 14.3f, 14.3f, 14.3f, 14.3f, 14.3f, 14.5f, 14.5f, 14.5f },
    { 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.6f, 14.6f, 14.6f, 14.5f, 14.5f, 14.5f, 14.5f, 14.6f, 14.7f, 14.7f, 14.7f },
    { 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.6f, 14.6f, 14.6f, 14.6f, 14.7f, 14.7f, 14.7f, 14.6f },
    { 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.6f, 14.5f },
    { 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.6f, 14.6f, 14.5f, 14.4f },
    { 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.6f, 14.5f, 14.4f, 14.4f, 14.3f },
    { 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.6f, 14.6f, 14.5f, 14.4f, 14.3f, 14.2f, 14.1f },
    { 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.7f, 14.6f, 14.5f, 14.5f, 14.4f, 14.4f, 14.3f, 14.2f, 14.1f, 14.0f, 13.9f },
    { 14.7f, 14.7f, 14.7f, 14.7f, 14.5f, 14.4f, 14.4f, 14.3f, 14.3f, 14.2f, 14.2f, 14.1f, 14.0f, 13.9f, 13.8f, 13.7f },
    { 14.5f, 14.5f, 14.5f, 14.4f, 14.3f, 14.1f, 13.9f, 13.8f, 13.7f, 13.6f, 13.5f, 13.4f, 13.3f, 13.2f, 13.1f, 13.1f },
    { 14.2f, 14.2f, 14.2f, 14.2f, 14.0f, 13.7f, 13.5f, 13.3f, 13.0f, 12.9f, 12.7f, 12.5f, 12.5f, 12.5f, 12.5f, 12.5f },
    { 13.9f, 13.9f, 13.9f, 13.9f, 13.8f, 13.6f, 13.3f, 13.1f, 12.8f, 12.6f, 12.5f, 12.3f, 12.2f, 12.1f, 12.1f, 12.1f },
    { 13.5f, 13.5f, 13.4f, 13.3f, 13.2f, 13.2f, 13.2f, 13.1f, 12.8f, 12.6f, 12.5f, 12.3f, 12.2f, 12.1f, 12.1f, 12.1f },
    { 13.1f, 13.1f, 13.1f, 13.1f, 13.1f, 13.1f, 13.1f, 13.0f, 12.8f, 12.6f, 12.5f, 12.3f, 12.2f, 12.1f, 12.1f, 12.1f }
};

static const float default_enrichment_by_map_sens[TABLE_PRESSURES_MAX] = {
    0.013f, 0.025f, 0.038f, 0.050f, 0.063f, 0.075f, 0.088f, 0.100f,
    0.113f, 0.125f, 0.138f, 0.150f, 0.163f, 0.175f, 0.188f, 0.200f
};

static const float default_enrichment_by_map_hpf[TABLE_ROTATES_MAX] = {
    0.116f, 0.113f, 0.109f, 0.105f, 0.101f, 0.098f, 0.094f, 0.090f,
    0.086f, 0.083f, 0.079f, 0.075f, 0.071f, 0.068f, 0.064f,0.060f,
};

static const float default_enrichment_by_thr_sens[TABLE_THROTTLES_MAX] = {
    0.013f, 0.025f, 0.038f, 0.050f, 0.063f, 0.075f, 0.088f, 0.100f,
    0.113f, 0.125f, 0.138f, 0.150f, 0.163f, 0.175f, 0.188f, 0.200f
};

static const float default_enrichment_by_thr_hpf[TABLE_ROTATES_MAX] = {
    0.116f, 0.113f, 0.109f, 0.105f, 0.101f, 0.098f, 0.094f, 0.090f,
    0.086f, 0.083f, 0.079f, 0.075f, 0.071f, 0.068f, 0.064f,0.060f,
};

void config_default_table(sEcuTable *table, uint8_t number)
{
  sprintf(table->name, "Default %d", number);

  table->inj_channel = InjectorChannel1;

  //For BOSCH 0280150762 injectors
  table->ignition_initial = 10.0f;
  table->injector_performance = 214.2f;
  table->fuel_pressure = 3.0f;
  table->fuel_mass_per_cc = 0.75f;

  table->pressures_count = ITEMSOF(default_pressures);
  memcpy(table->pressures, default_pressures, sizeof(default_pressures));

  table->rotates_count = ITEMSOF(default_rotates);
  memcpy(table->rotates, default_rotates, sizeof(default_rotates));

  table->throttles_count = ITEMSOF(default_throttles);
  memcpy(table->throttles, default_throttles, sizeof(default_throttles));

  table->fill_proportion_map_vs_thr = 0.7f;
  table->enrichment_proportion_map_vs_thr = 0.1f;
  memcpy(table->fill_by_map, default_filling_by_thr, sizeof(default_filling_by_map));
  memcpy(table->fill_by_thr, default_filling_by_thr, sizeof(default_filling_by_thr));

  memcpy(table->enrichment_by_map_sens, default_enrichment_by_map_sens, sizeof(default_enrichment_by_map_sens));
  memcpy(table->enrichment_by_map_hpf, default_enrichment_by_map_hpf, sizeof(default_enrichment_by_map_hpf));
  memcpy(table->enrichment_by_thr_sens, default_enrichment_by_thr_sens, sizeof(default_enrichment_by_thr_sens));
  memcpy(table->enrichment_by_thr_hpf, default_enrichment_by_thr_hpf, sizeof(default_enrichment_by_thr_hpf));


  table->fillings_count = ITEMSOF(default_fillings);
  memcpy(table->fillings, default_fillings, sizeof(default_fillings));
  memcpy(table->ignitions, default_ignitions, sizeof(default_ignitions));
  memcpy(table->fuel_mixtures, default_fuel_mixtures, sizeof(default_fuel_mixtures));
}

void config_default_params(sEcuParams *table)
{

}

void config_default_correctives(sEcuCorrections *table)
{

}

HAL_StatusTypeDef config_init(void)
{
  HAL_StatusTypeDef status = HAL_OK;
  return status;
}

int8_t config_load_table(sEcuTable *table, uint8_t number)
{

  return 0;
}

int8_t config_save_table(const sEcuTable *table, uint8_t number)
{

  return 0;
}


int8_t config_load_params(sEcuParams *params)
{

  return 0;
}

int8_t config_save_params(const sEcuParams *params)
{

  return 0;
}


int8_t config_load_correctives(sEcuCorrections *table)
{

  return 0;
}

int8_t config_save_correctives(const sEcuCorrections *table)
{

  return 0;
}

