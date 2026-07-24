#ifndef BSP_H
#define BSP_H

/*
 * BSP 聚合头文件（G3519 核心板测试）。
 * 应用层包含本文件；BSP 模块头文件只包含 bsp_common.h。
 */
#include "bsp_common.h"
#include "LED/led.h"
#include "UART0/uart0.h"
#include "TimerG6_PWM_RGB/timerG6_pwm_rgb.h"
#include "KEY/key.h"
#include "SPI0_OLED/spi0_oled.h"
#include "TimerA1/timera1.h"
#include "QEI/qei.h"
#include "KEY/keypad.h"
#include "ADC/adc.h"

#endif
