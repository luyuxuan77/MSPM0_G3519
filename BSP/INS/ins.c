#include "INS.h"
#include <math.h>
#include "IMU.h"

static INS_Data_t ins_data;
static imu_data_t g_imu_data;


void INS_Init(void)
{
    ins_data.ax = 0;
    ins_data.ay = 0;

    ins_data.vx = 0;
    ins_data.vy = 0;

    ins_data.x = 0;
    ins_data.y = 0;
}

 void IMU_get()
{
	IMU_getData(&g_imu_data);
}

volatile float Roll,Pitch,Yaw ;

float IMU_yaw()
{
	if (g_imu_data.ready) 
	{

		printf("Y:%.1f P:%.1f R:%.1f\r\n",
			   g_imu_data.yaw,
			   g_imu_data.pitch,
			   g_imu_data.roll);
//		
		Roll=g_imu_data.roll;
		Pitch=g_imu_data.pitch;
		Yaw=g_imu_data.yaw;		
	} else {
		printf("IMU not ready\r\n");
	}
 
	return Yaw ;
} 

void INS_Update(float ax,float ay, float yaw, float dt)
{
    float yaw_rad;

    yaw_rad = yaw * 3.1415926f / 180.0f;

    /*
     * 车体坐标
     * 转世界坐标
     */

    float world_ax;
    float world_ay;

    world_ax = ax * cosf(yaw_rad)
             - ay * sinf(yaw_rad);
	
    world_ay = ax * sinf(yaw_rad)
             + ay * cosf(yaw_rad);

    /*
     * 加速度积分得到速度
     */

    ins_data.vx += world_ax * dt;

    ins_data.vy += world_ay * dt;


    /*
     * 速度积分得到位置
     */

    ins_data.x += ins_data.vx * dt;

    ins_data.y += ins_data.vy * dt;


    ins_data.ax = world_ax;
    ins_data.ay = world_ay;

}



INS_Data_t *INS_GetData(void)
{
    return &ins_data;
}