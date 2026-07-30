#ifndef YUNTAI_CONTROL_H
#define YUNTAI_CONTROL_H

#include "bsp.h"

/*===========================================================================
 * 云台电机地址定义
 *===========================================================================*/
#define MOTOR_YAW_ADDR      0x01    // 水平方向电机地址（左右转动）
#define MOTOR_PITCH_ADDR    0x02    // 俯仰方向电机地址（上下转动）

/*===========================================================================
 * 云台控制参数
 *===========================================================================*/
#define YT_DEFAULT_SPEED    300     // 默认速度 (RPM)
#define YT_DEFAULT_ACC      50      // 默认加速度
#define YT_FINE_SPEED       100     // 精细调整速度 (RPM)
#define YT_FINE_ACC         20      // 精细调整加速度

#define YT_PULSES_PER_REV   1764    /* yaw motor shaft PPR (for dead-reckoning step calc) */
#define YT_PULSES_PER_DEG   8.9f   /* platform output: pulses per degree (measured) */

#define YT_PITCH_PPR        3420    /* pitch motor (0x02): measured 9.5 pulses/deg */
#define YT_PITCH_PPD        9.5f   /* pitch motor (platform output) calibration */

/*===========================================================================
 * 云台位置限位（角度，防止机械损坏）
 *===========================================================================*/
#define YT_YAW_MIN_DEG      -90.0f  // 水平最小角度
#define YT_YAW_MAX_DEG       90.0f  // 水平最大角度
#define YT_PITCH_MIN_DEG    -30.0f  // 俯仰最小角度
#define YT_PITCH_MAX_DEG     30.0f  // 俯仰最大角度

/*===========================================================================
 * 云台状态结构体
 *===========================================================================*/
typedef struct {
    float yaw_angle;      // 当前水平角度
    float pitch_angle;    // 当前俯仰角度
    bool  is_moving;      // 是否正在运动
    bool  is_enabled;     // 是否已使能
} yuntai_state_t;

/*===========================================================================
 * 靶心位置结构体
 *===========================================================================*/
typedef struct {
    float target_x;       // 靶心X坐标（像素或角度）
    float target_y;       // 靶心Y坐标（像素或角度）
    bool  target_found;   // 是否找到靶心
} target_info_t;

/*===========================================================================
 * 函数声明
 *===========================================================================*/

// 初始化云台（初始化UART3，复位位置）
void yuntai_control_init(void);

// 使能/失能云台电机
void yuntai_enable(bool enable);

// 复位云台到零位
void yuntai_reset_position(void);

// 绝对位置控制（角度）
void yuntai_move_to_angle(float yaw_deg, float pitch_deg);

// 相对位置控制（角度增量）
void yuntai_move_relative(float yaw_delta, float pitch_delta);

// 快速移动到指定角度
void yuntai_move_to_angle_fast(float yaw_deg, float pitch_deg);

// 精细调整到指定角度
void yuntai_move_to_angle_fine(float yaw_deg, float pitch_deg);

// 停止云台运动
void yuntai_stop(void);

// 获取云台当前状态
void yuntai_get_state(yuntai_state_t *state);

// 自动瞄准靶心（根据视觉反馈）
bool yuntai_auto_aim(target_info_t *target);

// 测试函数：云台扫描测试
void yuntai_test_scan(void);

// 读取电机实际位置（脉冲数）
// 返回：成功=true，超时=false
// 结果存入 *pulses_out（有符号32位，正=CW累计，负=CCW累计）
bool yuntai_read_motor_position(uint8_t addr, int32_t *pulses_out);

#endif
