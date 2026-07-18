/*------------------------------------------------------------------------------
 * Project: Car_sum - Grey-line tracking car
 * Based on: TI-M0SDK-2_08_00_03, sysconfig-1.25.0
 * MCU: MSPM0G3519, Clock: external 40MHz
 *---------------------------------------------------------------------------*/

#include "bsp.h"
#include "IMU.h"

/* ---------- IMU data ---------- */
static imu_data_t g_imu_data;
static uint32_t imu_tick = 0;

/* ---------- Time-slice scheduling ---------- */
static uint32_t print_tick = 0;
static uint32_t pid_tick = 0;
static uint32_t gw_tick = 0;
static uint32_t pid_speed = 0;
static uint32_t motor = 0;
static uint32_t gray = 0;

/* ---------- Global variables ---------- */
float ypr[3];
uint32_t adc0_data[4] = {0};
uint32_t qei_cnt[2] = {0};

int main(void)
{
    /* ===== 1. System init ===== */
    printf("\r\n=== SYSTEM RESET ===\r\n");
    SYSCFG_DL_init();
    uart0_init(115200);
    system_time_init();
	key_init();

    /* ===== 2. IMU init ===== */
	imu_app_init();
	delay_ms(1000);

	LCD_GPIO_Init();
	LCD_Init();
	LCD_Fill(0,0,LCD_W,LCD_H,WHITE);

	IMU_Gyro_Calibrate();

    /* ===== 3. Motor init ===== */
    motor_init();

    /* ===== 4. PID init ===== */
    Pid_Init(&Speed_Pid[0], &L1_PID_Value_Speed);
    Pid_Init(&Speed_Pid[1], &R1_PID_Value_Speed);
    Pid_Init(&Speed_Pid[2], &L2_PID_Value_Speed);
    Pid_Init(&Speed_Pid[3], &R2_PID_Value_Speed);
	motor_speed_pid_init();
	Pid_Init(&Grayscale_Direction_Pid,&PID_Value_Grayscale_Direction[0]);
    Speed_Pid[0].SetPoint = 0;
    Speed_Pid[1].SetPoint = 0;

	/* Gray sensor init */
	adc0_init();
	// LCD_ShowString(20,20,"goffset:",BLACK,WHITE,16,1);  // disabled


    /* ===== 5. Main loop ===== */
    while (1)
    {
//        /* ---- 5.1 IMU data every 10ms ---- */
//        if (system_time_elapsed_ms(&imu_tick, 10))
//		{
//            IMU_get();
//			mode_switch();
//        }

        /* ---- Motor PID control every 128ms ---- */
        if (system_time_elapsed_ms(&pid_tick, 128))
		{
            motor_control_update();
        }

        /* ---- Speed command every 10ms ---- */
        if (system_time_elapsed_ms(&pid_speed, 10))
		{
            Pid_Speed();
        }

		/* ---- Gray sensor data collection every 10ms ---- */
		if (system_time_elapsed_ms(&gray, 10))
		{
			get_gray_refresh_data();
		}

//        /* ---- 5.4 INS task every 10ms ---- */
//        if (system_time_elapsed_ms(&gw_tick, 10))
//		{
////			around();
//			Turn_Left(100);
////			Turn_Right(100);
//		}

//        /* ---- 5.3 Print IMU data every 500ms ---- */
//        if (system_time_elapsed_ms(&print_tick, 500))
//		{
//            IMU_yaw();
////			int tell = road_tell();
////			LCD_ShowIntNum(50,50,tell,3,BLACK,WHITE,24);
//        }


//			Track_Direction_Control(100) ;
//            imu_data_t imu;
//			IMU_getData(&imu);

//			INS_Update(0.0f, 0.0f,imu.yaw, 0.01f);
//			INS_Data_t *ins;

//			ins = INS_GetData();
//			printf("x=%.2f y=%.2f vx=%.2f vy=%.2f\r\n",
//			ins->x,ins->y,ins->vx,ins->vy);
//			get_gray_offset();




//		delay_ms(100);
    }
}
