#ifndef KEY_H
#define KEY_H

#include "bsp_common.h"

/* 用户按键：PB31，下降沿中断，key_flag 在 0/1 间切换界面 */
void key_init(void);
extern volatile uint8_t key_flag;

#endif
