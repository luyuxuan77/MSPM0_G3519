#ifndef YUNTAI_TEST_H
#define YUNTAI_TEST_H

#include "bsp.h"

// 快速测试：只测一个电机，转45度再转回来
void yuntai_test_quick(void);

// 基础测试：测试所有基本功能，包括两个电机
void yuntai_test_basic(void);

// 扫描测试：水平和垂直各扫描一次
void yuntai_test_scan_simple(void);

// 单电机测试：只测试指定电机
void yuntai_test_single_motor(uint8_t motor_addr, float angle);

#endif

