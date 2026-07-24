#ifndef QEI_H
#define QEI_H

#include "bsp_common.h"

/*
 * QEI (Quadrature Encoder Interface) 初始化
 *
 * Motor0 编码器: TIMG9, PB29(PHA/CCP0), PB30(PHB/CCP1)
 * Motor1 编码器: TIMG8, PC6(PHA/CCP0), PA22(PHB/CCP1)
 *
 * 注意: SysConfig 的 PWM 模块将 CCP 引脚配置为输出，
 * 本函数会将其修正为输入功能并配置 QEI 模式。
 */

void qei_motor0_init(void);  /* TIMG9 — Motor0 编码器 */
void qei_motor1_init(void);  /* TIMG8 — Motor1 编码器 */

/* 读取编码器计数值 (QEI 模式下 Counter 即为位置) */
uint32_t qei_motor0_get_count(void);
uint32_t qei_motor1_get_count(void);

#endif
