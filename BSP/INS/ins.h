#ifndef __INS_H
#define __INS_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * 2D Wheel Odometry (编码器 + 陀螺仪 航位推算)
 * ============================================================================
 *
 * 原理:
 *   距离 = (左轮距离 + 右轮距离) / 2   — 来自编码器 (motor.c)
 *   航向 = Yaw (度)                    — 来自 IMU 陀螺仪 (IMU.c, 50Hz)
 *   位置积分:  x += dist * cos(yaw),  y += dist * sin(yaw)
 *
 * 精度:
 *   - 位置误差: 行驶距离的 3~8% (受轮径标定、打滑、陀螺漂移影响)
 *   - 航向漂移: 约 1~2°/min (无磁力计)
 *
 * 使用:
 *   1. Odometry_Init()            — 系统上电调用一次
 *   2. Odometry_Update(dL,dR,yaw) — 每个控制周期 (128ms) 调用
 *   3. Odometry_GetData()         — 读取当前位置/里程
 *   4. Odometry_Reset()           — 按键一键归零
 * ============================================================================ */

/* ---- 里程计数据结构 ---- */
typedef struct {
    float x;            /* 世界坐标 X (cm), 前向为正 */
    float y;            /* 世界坐标 Y (cm), 左向为正 */
    float yaw_deg;      /* 当前航向角 (度), 0=初始朝向 */
    float total_dist;   /* 累计行驶里程 (cm) */
    float dist_L;       /* 左轮累计里程 (cm) */
    float dist_R;       /* 右轮累计里程 (cm) */
} Odometry_t;

/* ---- 旧版 INS 结构 (保留兼容, 已弃用) ---- */
typedef struct
{
    float ax;
    float ay;

    float vx;
    float vy;

    float x;
    float y;

} INS_Data_t;

/* ===== Odometry API (推荐使用) ===== */

/* 初始化/归零里程计 */
void Odometry_Init(void);

/*
 * 里程计主更新函数
 *
 * 参数:
 *   dist_L_cm — 本周期左轮前进距离 (cm), 倒退为负值
 *   dist_R_cm — 本周期右轮前进距离 (cm), 倒退为负值
 *   yaw_deg   — 当前 IMU 航向角 (度)
 *
 * 调用频率: 与 motor_control_update() 同步 (128ms)
 */
void Odometry_Update(float dist_L_cm, float dist_R_cm, float yaw_deg);

/* 一键归零 (位置 + 里程归零, 航向保持不变) */
void Odometry_Reset(void);

/* 获取里程计数据只读指针 */
const Odometry_t* Odometry_GetData(void);

/* ===== 旧版 INS API (保留, 已弃用) ===== */
void INS_Init(void);

/*
 * 旧版 INS 更新 (加速度二次积分 → 已弃用, 精度不可接受)
 *
 * 说明: 此函数保留用于编译兼容, 内部为空实现。
 *       新代码请使用 Odometry_Update()。
 */
void INS_Update(float ax, float ay, float yaw, float dt);

void IMU_get(void);
float IMU_yaw(void);
INS_Data_t *INS_GetData(void);
extern float IMU_YAW_OFFSET;

extern volatile float Roll, Pitch, Yaw;

#endif
