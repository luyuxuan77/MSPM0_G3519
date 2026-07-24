#ifndef IMU_H
#define IMU_H

#include "bsp_common.h"

#define M_PI (3.1415926535f)

typedef struct {
	float x;
	float y;
	float z;
} xyz_f_t;

extern xyz_f_t north, west;
extern volatile float yaw[5];
extern float motion6[7];

bool IMU_init(void);
void IMU_getYawPitchRoll(float *ypr);

#endif
