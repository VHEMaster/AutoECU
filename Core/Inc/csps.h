#include "main.h"

typedef struct
{
  uint32_t DelayCur;
  uint32_t DelayPrev;
  float AngleCur14;
  float AnglePrev14;
  float AngleCur23;
  float AnglePrev23;
  float RPM;
  float uSPA;
  float Period;
  uint32_t PhasedActive;
  float PhasedAngleCur;
  float PhasedAnglePrev;
}sCspsData;

typedef void (*csps_rotated_callback_t)(uint32_t period);

void csps_loop(void);
void csps_register_rotated_callback(csps_rotated_callback_t callback);
void csps_set_tsps_simulation_egde(int8_t edge);
void csps_init(volatile uint32_t *timebase, TIM_HandleTypeDef *_htim, uint32_t channel);
void csps_exti(uint32_t timestamp);
void csps_tsps_exti(uint32_t timestamp);
float csps_getangle14(sCspsData data);
float csps_getangle23from14(float angle);
float csps_getangle23(sCspsData data);
float csps_getrpm(sCspsData data);
float csps_getperiod(sCspsData data);
float csps_getuspa(sCspsData data);
uint8_t csps_isrotates(void);
uint8_t csps_isrunning(void);
uint8_t csps_isfound(void);
uint8_t csps_isphased(sCspsData data);
uint8_t csps_iserror(void);
uint32_t csps_getturns(void);
uint32_t csps_gethalfturns(void);
float csps_getphasedangle(sCspsData data);
float csps_getphasedangle_cy(sCspsData data, uint8_t cylinder, float angle);
sCspsData csps_data(void);
float csps_gettspsrelpos(void);
