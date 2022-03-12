#include "main.h"

void csps_loop(void);
void csps_init(volatile uint32_t *timebase, TIM_HandleTypeDef *_htim, uint32_t channel);
void csps_exti(uint32_t timestamp);
void csps_tsps_exti(uint32_t timestamp);
float csps_getangle14(void);
float csps_getangle23from14(float angle);
float csps_getangle23(void);
float csps_getrpm(void);
float csps_getperiod(void);
float csps_getuspa(void);
uint8_t csps_isrotates(void);
uint8_t csps_isrunning(void);
uint8_t csps_isfound(void);
uint8_t csps_isphased(void);
uint8_t csps_iserror(void);
uint32_t csps_getturns(void);
uint32_t csps_gethalfturns(void);
float csps_getphasedangle(void);
