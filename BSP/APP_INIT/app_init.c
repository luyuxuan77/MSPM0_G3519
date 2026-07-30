
#include "bsp.h"

bool imu_ready;
static uint32_t s_imu_tick;
static float s_imu_ypr[3];
static bool s_imu_ready;

void imu_app_init(void)
{
    uint32_t now;


    printf("[IMU] init start\r\n");


    // 初始化LSM6DSV16X
    IMU_init();


    // SPI CS释放
    SPI1_CS_IMU(1);


    // 检查状态
    if(IMU_isReady())
    {
        printf("[IMU] ready\r\n");
    }
    else
    {
        printf("[IMU] error\r\n");
    }


    // 姿态数据清零
    memset(s_imu_ypr,0,sizeof(s_imu_ypr));


    now = system_time_get_tick_ms();

    s_imu_tick = now;


    printf("[IMU] init finish\r\n");

}
float yaw;
float pitch;
float roll;

static void line_follow_update_imu(void)
{
    s_imu_ready = IMU_isReady();

    if (s_imu_ready) {
        IMU_getYawPitchRoll(s_imu_ypr);
		yaw   = s_imu_ypr[0];
        pitch = s_imu_ypr[1];
        roll  = s_imu_ypr[2];
    } else {
        yaw = 0.0f;
        pitch = 0.0f;
        roll = 0.0f;
    }
}

void getYawPitchRoll()
{
	IMU_getYawPitchRoll(s_imu_ypr);
	printf("yaw:%.2f pitch:%.2f roll:%.2f\r\n", yaw, pitch, roll);
}