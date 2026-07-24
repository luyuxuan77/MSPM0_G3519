#ifndef TIMERA1_H
#define TIMERA1_H

#include "bsp_common.h"

extern uint32_t nowtime; /* 100us 时基，供 IMU 解算使用 */

void TimerA1_init(void);

#endif
