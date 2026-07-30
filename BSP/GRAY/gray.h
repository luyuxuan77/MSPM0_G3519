#ifndef _GRAY_H_
#define _GRAY_H_
#include "bsp.h"

/* I2C background read — call from main loop when g_i2c_read_flag is set */
void gw_i2c_read_task(void);

/* Offset calc + LCD display using I2C data in gray_now_val[] */
int get_gray_refresh_data(void);

uint8_t get_offset_s(void);

extern uint8_t g_arc_mode;  /* 0=直线循迹 1=圆弧循迹 */

/* ADC read VCC voltage at XT30 connector (still uses ADC0, independent of I2C) */
uint16_t get_adc0_value(void);

#endif
