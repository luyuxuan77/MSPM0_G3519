#ifndef BSP_COMMON_H
#define BSP_COMMON_H

/*
 * BSP 公共基础头文件。
 * 统一 C 标准库、DriverLib、SysConfig 生成宏与跨模块延时接口。
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#include "ti_msp_dl_config.h"

#define u8 unsigned char
#define u32 unsigned int

void delay_ms(uint32_t ms);
void delay_us(uint32_t us);

#endif
