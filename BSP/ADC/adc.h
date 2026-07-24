#ifndef ADC_H
#define ADC_H

#include "bsp_common.h"

/*
 * ADC 初始化 (手动配置 — 未使用 SysConfig ADC 模块)
 *
 * ADC0: 7 路模拟输入
 *   CH0 → PA27, CH1 → PA26, CH2 → PA25, CH3 → PA24
 *   CH4 → PB25, CH5 → PB24, CH6 → PA16
 *
 * ADC1: 1 路模拟输入
 *   CH0 → PA15 (与 DAC_OUT 共享引脚)
 *
 * ADC 引脚默认即为模拟模式，无需 GPIO 数字功能配置。
 * DL_GPIO_reset() 后引脚处于高阻模拟态，直接可用作 ADC 输入。
 */

void adc0_init(void);
void adc1_init(void);

/* 单次转换指定通道，返回 12-bit 原始值 */
uint16_t adc0_read_channel(uint8_t channel);
uint16_t adc1_read_channel(uint8_t channel);

#endif
