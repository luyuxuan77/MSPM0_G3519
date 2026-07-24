#ifndef TIMERG6_PWM_RGB_H
#define TIMERG6_PWM_RGB_H

#include "bsp_common.h"

/* WS2812 RGB 灯，使用 SysConfig 配置的 TIMG6（PA29） */
void timerg6_pwm_rgb_init(void);
void set_RGB(uint8_t g, uint8_t r, uint8_t b);

#endif
