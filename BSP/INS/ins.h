#ifndef __INS_H
#define __INS_H

#include <stdint.h>

/* ============================================================================
 * 2D 轮式里程计 (编码器 + 陀螺仪 航位推算)
 * ----------------------------------------------------------------------------
 *   距离 = (左轮距离 + 右轮距离) / 2   — 来自编码器 (motor.c)
 *   航向 = Yaw (度)                    — 来自 IMU 陀螺仪 (IMU.c)
 *   位置积分:  x += dist*cos(yaw),  y += dist*sin(yaw)
 *
 *   用法:
 *     1. Odometry_Init()            上电调用一次
 *     2. Ohdometry_Update(dL,dR,yaw) 每个里程周期(128ms)调用
 *     3. Odometry_GetData()         读取位置/里程
 *     4. Odometry_Reset()           按键一键归零(航向不清)
 * ============================================================================ */
typedef 
struct {
    float x;            /* 世界坐标 X (cm), 前向为正 */
    float y;            /* 世界坐标 Y (cm), 左向为正 */
    float yaw_deg;      /* 当前航向角 (度), 0=初始朝向 */
    float total_dist;   /* 累计行驶里程 (cm) */
    float dist_L;       /* 左轮累计里程 (cm) */
    float dist_R;       /* 右轮累计里程 (cm) */
} Odometry_t;

/* 初始化/清零里程计 */
void Odometry_Init(void);
/* 里程计主更新: dist_*_cm 为本周期左右轮前进距离(cm, 倒退为负), yaw_deg 为当前航向 */
void Odometry_Update(float dist_L_cm, float dist_R_cm, float yaw_deg);
/* 一键归零 (位置+里程归零, 航向保持不变) */
void Odometry_Reset(void);
/* 获取里程计只读指针 */
const Odometry_t* Odometry_GetData(void);



typedef struct
{
    float ax;
    float ay;
    float az;

    float vx;
    float vy;

    float x;
    float y;

} INS_Data_t;



void INS_Init(void);


/*
 * INS����
 *
 * ax: С��ǰ����ٶ�(m/s2)
 * ay: С��������ٶ�(m/s2)
 * yaw: �����(��)
 * dt: ʱ����(s)
 */
void INS_Update(float ax,
                float ay,
                float yaw,
                float dt);

void INS_UpdateFromIMU(float dt);


//void IMU_get();
float IMU_get();				
INS_Data_t *INS_GetData(void);
extern float IMU_YAW_OFFSET;

extern volatile float Roll,Pitch,Yaw;
extern volatile float Accel_X,Accel_Y,Accel_Z;
#endif
				