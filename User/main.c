/*------------------------------------------------------------------------------
 * 公司名称：启是科技
 * 工程名： empty
 * 作者： 孔
 * 创建时间： 2025.12.19.18.25
 * 版本： V1.0
 * 描述： 该工程基于TI-M0SDK-2_08_00_03生成，推荐sysconfig-1.25.0配套使用；目前仅配
 时钟树，使用外部40M晶振作为时钟源，时钟分支皆以最高频率运行。
 *----------------------------------------------------------------------------*/

#include "bsp.h"
#include "IMU.h"

/* ---------- IMU 数据 ---------- */
static imu_data_t g_imu_data;
static uint32_t imu_tick = 0;

/* ---------- 时间片变量 ---------- */
static uint32_t print_tick = 0;
static uint32_t pid_tick = 0;
static uint32_t gw_tick = 0;
static uint32_t pid_speed = 0;
static uint32_t motor = 0;

/* ---------- 其他全局变量（如有需要可保留） ---------- */
float ypr[3];
uint32_t adc0_data[4] = {0};
uint32_t qei_cnt[2] = {0};

int main(void)
{
    /* ===== 1. 系统初始化 ===== */
    printf("\r\n=== SYSTEM RESET ===\r\n");
    SYSCFG_DL_init();
    uart0_init(115200);
    system_time_init();
		key_init();
    /* ===== 4. IMU 初始化 ===== */
	imu_app_init();          // 内部调用 IMU_init() + SPI1_CS_IMU(1)
	delay_ms(1000);          // 等待 IMU 稳定收敛
	
//	LCD_GPIO_Init();
//	LCD_Init();
//	LCD_Fill(0,0,LCD_W,LCD_H,WHITE);	
//	
	IMU_Gyro_Calibrate();	
    /* ===== 2. 电机初始化 ===== */
    motor_init();

    /* ===== 3. PID 初始化 ===== */
    Pid_Init(&Speed_Pid[0], &L1_PID_Value_Speed);
    Pid_Init(&Speed_Pid[1], &R1_PID_Value_Speed);
    Pid_Init(&Speed_Pid[2], &L2_PID_Value_Speed);
    Pid_Init(&Speed_Pid[3], &R2_PID_Value_Speed);
		motor_speed_pid_init();
	adc0_init();
	Pid_Init(&Grayscale_Direction_Pid,&PID_Value_Grayscale_Direction[0]);
    Speed_Pid[0].SetPoint = 0;
    Speed_Pid[1].SetPoint = 0;

//	LCD_ShowString(20,20,"goffset:",BLACK,WHITE,16,1);


    /* ===== 5. 主循环 ===== */
    while (1)
    {                                                                  
//        /* ---- 5.1 每 10ms 更新 IMU 数据（修正后的角度） ---- */
//        if (system_time_elapsed_ms(&imu_tick, 10)) 
//		{
//            IMU_get(); 
//			mode_switch();
//        }
// 
        /* ---- 电机 PID 控制 ---- */
        if (system_time_elapsed_ms(&pid_tick, 128))
		{
            motor_control_update();  // 你的 PID 速度控制函数
        }
		
        /* ---- 下达电机速度指令 ---- */
        if (system_time_elapsed_ms(&pid_speed, 10))
		{
            Pid_Speed();  // 你的 PID 速度控制函数
        }
		
//		/* ---- 开环控制电机速度 ---- */
//		if (system_time_elapsed_ms(&motor, 10))
//				{
//			set_motor_speed(0,0,0,0);
//		}

//        /* ---- 5.4 每 10ms 执行电机 INS 控制 ---- */
//        if (system_time_elapsed_ms(&gw_tick, 10)) 
//		{
////			around();
//			Turn_Left(100);
////			Turn_Right(100);
//		}
		
//        /* ---- 5.3 每 500ms 打印 IMU 数据 ---- */
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
			
			
      
		
//		delay_ms(100);  // 让出 CPU，避免空转
    }
}