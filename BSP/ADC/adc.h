#ifndef ADC_H
#define ADC_H

#include "bsp_common.h"

/*
 * ADC0: 7 路 — PA27,PA26,PA25,PA24, PB25,PB24, PA16
 * ADC1: 1 路 — PA15
 */

void adc0_init(void);
void adc1_init(void);

uint16_t adc0_read_channel(uint8_t channel);
uint16_t adc1_read_channel(uint8_t channel);

/* 全局数组 + 一次采样全部8路 */
extern uint16_t g_adc_raw[8];
void adc_sample_all(void);

#endif
