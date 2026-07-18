#ifndef __INS_H
#define __INS_H

#include <stdint.h>


typedef struct
{
    float ax;
    float ay;

    float vx;
    float vy;

    float x;
    float y;

} INS_Data_t;



void INS_Init(void);


/*
 * INS更新
 *
 * ax: 小车前向加速度(m/s2)
 * ay: 小车侧向加速度(m/s2)
 * yaw: 航向角(度)
 * dt: 时间间隔(s)
 */
void INS_Update(float ax,
                float ay,
                float yaw,
                float dt);


void IMU_get();
float IMU_yaw();				
INS_Data_t *INS_GetData(void);
extern float IMU_YAW_OFFSET;

extern volatile float Roll,Pitch,Yaw;
#endif