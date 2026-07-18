#ifndef _APP_INIT_H
#define _APP_INIT_H

#include "bsp.h"
void imu_app_init(void);
extern float yaw;
extern float pitch;
extern float roll;

void getYawPitchRoll();
#endif