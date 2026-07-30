/*------------------------------------------------------------------------------
 * Project: Car_sum - Grey-line tracking car
 * Based on: TI-M0SDK-2_08_00_03, sysconfig-1.25.0
 * MCU: MSPM0G3519, Clock: external 40MHz
 *---------------------------------------------------------------------------*/

#include "bsp.h"
#include "IMU.h"

/* ---------- Time-slice scheduling ---------- */
static uint32_t gray = 0;
static uint32_t menu_tick = 0;
static uint32_t opi_sensor_tick = 0;
static uint32_t imu_dbg_tick = 0;

/* ---------- Global variables ---------- */
float ypr[3];
uint32_t qei_cnt[2] = {0};

int main(void)
{
    /* ===== 1. System init ===== */
    SYSCFG_DL_init();
    uart0_init(115200);
    printf("hello_world!\r\n");

    /* ===== 2. Software I2C + GW Gray Sensor init ===== */
    sw_i2c_init();

    /* 广播并重置传感器地址为默认0x4C */
    i2c_reset();
    delay_ms(100);

    /* gw_ping传感器直到响应 */
    while (gw_ping()) {
        delay_ms(1);
        printf("Ping Faild Try Again!\r\n");
    }
    printf("Ping Succseful!\r\n");

    /* ===== 3. IMU init ===== */
    imu_app_init();
    delay_ms(200);
    LCD_Init();
    LCD_Fill(0,0,LCD_W,LCD_H,WHITE);
    menu_init();

    /* Gray sensor: ADC0 still enabled but I2C is the data source */
    adc0_init();
    IMU_Gyro_Calibrate();

    system_time_init();
    key_init();
    buzzer_init();

    /* ===== OrangePi UART4 init ===== */
//    uart4_opi_init();

    /* ===== 4. Motor init ===== */
    motor_init();

    /* ===== 5. PID init ===== */
    Pid_Init(&Speed_Pid[0], &L1_PID_Value_Speed);
    Pid_Init(&Speed_Pid[1], &R1_PID_Value_Speed);
    Pid_Init(&Speed_Pid[2], &L2_PID_Value_Speed);
    Pid_Init(&Speed_Pid[3], &R2_PID_Value_Speed);
    motor_speed_pid_init();
    Pid_Init(&Grayscale_Direction_Pid,&PID_Value_Grayscale_Direction[0]);
    Pid_Init(&Yaw_Pid,&PID_Value_Yaw);
    Speed_Pid[0].SetPoint = 0;
    Speed_Pid[1].SetPoint = 0;

    Odometry_Init();

    while (1)
    {
        /*
         * BACKGROUND: I2C sensor read, triggered by TIMA1 ISR every 10ms.
         * Must run BEFORE the 10ms gray task so gray_now_val[] is fresh.
         */
        if (g_i2c_read_flag) {
            g_i2c_read_flag = 0;
            gw_i2c_read_task();
        }

        /* ---- Menu update every 50ms ---- */
        if (system_time_elapsed_ms(&menu_tick, 50))
        {
            menu_update();
        }

        /* ---- OrangePi protocol processing ---- */
//        uart4_opi_process();

        if (system_time_elapsed_ms(&opi_sensor_tick, 50))
        {
            g_angle_adc_raw = 0;
            g_angle_cdeg = 0;
//            uart4_opi_send_sensor_data(0, 0, 0);
        }

#if 0  /* IMU串口调试: 置1开启 */
        if (system_time_elapsed_ms(&imu_dbg_tick, 200))
        {
            IMU_get();  /* refresh Accel_X/Y/Z + Yaw/Pitch/Roll */
            printf("[IMU] Accel(mg): X=%6d Y=%6d Z=%6d | Angle(deg): Yaw=%7.1f Pitch=%6.1f Roll=%6.1f\r\n",
                   (int)Accel_X, (int)Accel_Y, (int)Accel_Z,
                   (double)Yaw, (double)Pitch, (double)Roll);
        }
#endif

        /* ---- 10ms task dispatch ---- */
        if (system_time_elapsed_ms(&gray, 10))
        {
            if (g_menu_running && g_menu_task_id == MENU_TASK_LINE_LAP) {
                task2((float)g_track_speed_cm_s);
            } else if (g_menu_running && g_menu_task_id == MENU_TASK_FULL_LAP) {
                task5((float)g_track_speed_cm_s);
                if (route_finish) menu_stop_task();
            } else if (g_menu_running && g_menu_task_id == MENU_TASK_SETPOINT_LAP) {
                task6((float)g_track_speed_cm_s);
                if (route_finish) menu_stop_task();
            } else if (g_menu_running && g_menu_task_id == MENU_TASK_A_TO_B) {
                task4((float)g_track_speed_cm_s);
                if (route_finish) menu_stop_task();
            } else {
                get_gray_refresh_data();
                Speed_Pid[0].SetPoint = 0;
                Speed_Pid[1].SetPoint = 0;
                run_flag = 0;
            }
        }
    }
}
