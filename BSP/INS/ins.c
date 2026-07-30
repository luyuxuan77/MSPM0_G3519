#include "INS.h"
#include <math.h>
#include <string.h>
#include "IMU.h"

static INS_Data_t ins_data;
static imu_data_t g_imu_data;

/* ============================================================================
 * 2D 轮式里程计实现
 * ============================================================================ */
static Odometry_t g_odometry;

void Odometry_Init(void)
{
    memset(&g_odometry, 0, sizeof(g_odometry));
}

void Odometry_Update(float dist_L_cm, float dist_R_cm, float yaw_deg)
{
    float dist;
    float yaw_rad;

    /* 小车中心平均前进距离 */
    dist = (dist_L_cm + dist_R_cm) * 0.5f;

    /* 航向转弧度 */
    yaw_rad = yaw_deg * 3.1415926f / 180.0f;

    /* 2D 位置积分 */
    g_odometry.x += dist * cosf(yaw_rad);
    g_odometry.y += dist * sinf(yaw_rad);

    /* 累计里程 (取绝对值, 倒退也算里程) */
    g_odometry.yaw_deg    = yaw_deg;
    g_odometry.total_dist += (dist >= 0.0f ? dist : -dist);
    g_odometry.dist_L     += (dist_L_cm >= 0.0f ? dist_L_cm : -dist_L_cm);
    g_odometry.dist_R     += (dist_R_cm >= 0.0f ? dist_R_cm : -dist_R_cm);
}

void Odometry_Reset(void)
{
    g_odometry.x          = 0.0f;
    g_odometry.y          = 0.0f;
    g_odometry.total_dist = 0.0f;
    g_odometry.dist_L     = 0.0f;
    g_odometry.dist_R     = 0.0f;
    /* yaw_deg 保持不变 — 清零位置但不清零航向 */
}

const Odometry_t* Odometry_GetData(void)
{
    return &g_odometry;
}


void INS_Init(void)
{
    ins_data.ax = 0;
    ins_data.ay = 0;
    ins_data.az = 0;

    ins_data.vx = 0;
    ins_data.vy = 0;

    ins_data.x = 0;
    ins_data.y = 0;
}

// void IMU_get()
//{
//	IMU_getData(&g_imu_data);
//	if (g_imu_data.ready)
//	{
//		Roll  = g_imu_data.roll;
//		Pitch = g_imu_data.pitch;
//		Yaw   = g_imu_data.yaw;
//	}
//}

volatile float Roll,Pitch,Yaw;
volatile float Accel_X,Accel_Y,Accel_Z;

float IMU_get()
{
	IMU_getData(&g_imu_data);
	if (g_imu_data.ready)
	{
		Roll  = g_imu_data.roll;
		Pitch = g_imu_data.pitch;
		Yaw   = g_imu_data.yaw;
		Accel_X = g_imu_data.accel_mg[0];
		Accel_Y = g_imu_data.accel_mg[1];
		Accel_Z = g_imu_data.accel_mg[2];
	}

	return Yaw;
}

void INS_Update(float ax,float ay, float yaw, float dt)
{
    float yaw_rad;

    yaw_rad = yaw * 3.1415926f / 180.0f;

    /*
     * ��������
     * ת��������
     */

    float world_ax;
    float world_ay;

    world_ax = ax * cosf(yaw_rad)
             - ay * sinf(yaw_rad);
	
    world_ay = ax * sinf(yaw_rad)
             + ay * cosf(yaw_rad);

    /*
     * ���ٶȻ��ֵõ��ٶ�
     */

    ins_data.vx += world_ax * dt;

    ins_data.vy += world_ay * dt;


    /*
     * �ٶȻ��ֵõ�λ��
     */

    ins_data.x += ins_data.vx * dt;

    ins_data.y += ins_data.vy * dt;


    ins_data.ax = world_ax;
    ins_data.ay = world_ay;
    ins_data.az = Accel_Z * 0.00980665f;

}

void INS_UpdateFromIMU(float dt)
{
    if (g_imu_data.ready) {
        INS_Update(Accel_X * 0.00980665f,
                   Accel_Y * 0.00980665f,
                   Yaw,
                   dt);
    }
}



INS_Data_t *INS_GetData(void)
{
    return &ins_data;
}