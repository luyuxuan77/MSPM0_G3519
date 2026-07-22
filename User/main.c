/*------------------------------------------------------------------------------
 * Project: Car_sum - Grey-line tracking car
 * Based on: TI-M0SDK-2_08_00_03, sysconfig-1.25.0
 * MCU: MSPM0G3519, Clock: external 40MHz
 *---------------------------------------------------------------------------*/

#include "bsp.h"
#include "IMU.h"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wpointer-sign"

/* ---------- IMU data ---------- */
static imu_data_t g_imu_data;
static uint32_t imu_tick = 0;

/* ---------- Time-slice scheduling ---------- */
static uint32_t print_tick = 0;
static uint32_t gw_tick = 0;
static uint32_t motor = 0;
static uint32_t gray = 0;
static uint32_t speed_tick = 0;
static uint32_t menu_tick = 0;
static uint32_t odo_tick = 0;

/* ---------- Global variables ---------- */
float ypr[3];
uint32_t adc0_data[4] = {0};
uint32_t qei_cnt[2] = {0};
extern uint8_t g_task1_active;
extern uint8_t g_task1_running;
extern uint8_t g_task3_active;
extern uint8_t g_task3_running;

/* TASK3 state: yaw PID + slow encoder I for straight line */
static float   g_task3_steer      = 0;    /* encoder I trim (speed counts) */
static float   g_task3_start_yaw  = 0;    /* locked initial heading (deg) */
static float   g_task3_yaw_isum   = 0;    /* yaw I term */
static float   g_task3_last_yaw_err = 0;  /* yaw D term: previous error */
static float   g_task3_dist_L     = 0;    /* accumulated left distance (cm) */
static float   g_task3_dist_R     = 0;    /* accumulated right distance (cm) */
static float   g_task3_dist_isum  = 0;    /* encoder distance integral */
static uint8_t g_task3_dist_init  = 0;    /* accumulator needs reset */
static uint8_t g_task3_was_running = 0;   /* detect stop edge for report */

/* ===== Self-test indicator (LED1+LED2 blink, buzzer) ===== */
static void self_test(void)
{
    buzzer_init();

    /* LED1+LED2 blink 5 times (100ms on/off, total 1s) */
    for (int i = 0; i < 5; i++) {
        LED1(1); LED2(1);
        delay_ms(100);
        LED1(0); LED2(0);
        delay_ms(100);
    }

    /* Buzzer 2kHz, 1s */
    buzzer_play(2000, 50);
    delay_ms(500);
    buzzer_stop();
}

int main(void)
{
    /* ===== 1. System init ===== */
    printf("\r\n=== SYSTEM RESET ===\r\n");
    SYSCFG_DL_init();
    uart0_init(115200);

    /* ===== 2. IMU init (before timer ISR to avoid SPI contention) ===== */
    imu_app_init();
    delay_ms(1000);

    LCD_GPIO_Init();
    LCD_Init();
    LCD_Fill(0,0,LCD_W,LCD_H,WHITE);

    IMU_Gyro_Calibrate();

    /* Start system timer AFTER calibration (ISR reads IMU, would contend) */
    system_time_init();
    key_init();

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
    menu_init();

    /* Odometry init */
    Odometry_Init();

    /* ===== 5. Self-test ===== */
    self_test();

    /* ===== 6. Main loop ===== */
    while (1)
    {
        /* IMU read is done in TIMA1 ISR every 20ms — Yaw/Pitch/Roll already updated */

        /* ---- TASK1 serial output: CSV telemetry every 100ms ---- */
        if (g_task1_active && g_task1_running &&
            system_time_elapsed_ms(&speed_tick, 100))
        {
            float L_actual = get_motor_speed_cm_s(0);
            float R_actual = get_motor_speed_cm_s(1);
            printf("%lu,100.0,%.1f,%.1f\r\n", (unsigned long)nowtime, L_actual, R_actual);
        }

        /* ---- Menu update every 50ms ---- */
        if (system_time_elapsed_ms(&menu_tick, 50))
        {
            menu_update();
        }

        /* ---- 10ms tasks: motor setpoints ---- */
        if (system_time_elapsed_ms(&gray, 10))
        {
            if (g_task1_active && g_task1_running) {
                /* TASK 1: PID speed test — both wheels at 100 cm/s */
                float sp = cm_s_to_speed_counts(100.0f);
                Speed_Pid[0].SetPoint = sp;
                Speed_Pid[1].SetPoint = sp;
            } else if (g_task3_active && g_task3_running) {
                /* TASK 3: yaw-locked PI + slow encoder I trim */
                float sp3 = cm_s_to_speed_counts(40.0f);

                /* Yaw PID: P + I + D to dampen oscillation */
                float yaw_err = g_task3_start_yaw - Yaw;
                if (yaw_err >  180.0f) yaw_err -= 360.0f;
                if (yaw_err < -180.0f) yaw_err += 360.0f;

                /* I term */
                g_task3_yaw_isum += yaw_err * 0.15f;
                if (g_task3_yaw_isum >  15.0f) g_task3_yaw_isum =  15.0f;
                if (g_task3_yaw_isum < -15.0f) g_task3_yaw_isum = -15.0f;

                /* D term: rate of error change (10ms loop → ×100 for /s) */
                float yaw_deriv = (yaw_err - g_task3_last_yaw_err) * 100.0f;
                g_task3_last_yaw_err = yaw_err;

                float yaw_steer = yaw_err * 4.0f + g_task3_yaw_isum + yaw_deriv * 0.4f;
                if (yaw_steer >  40.0f) yaw_steer =  40.0f;
                if (yaw_steer < -40.0f) yaw_steer = -40.0f;

                /* Slow encoder I: trim persistent wheel diameter diff, don't fight yaw */
                float total_steer = yaw_steer + g_task3_steer;
                if (total_steer >  50.0f) total_steer =  50.0f;
                if (total_steer < -50.0f) total_steer = -50.0f;

                Speed_Pid[0].SetPoint = sp3 + total_steer;  /* +steer → turn LEFT */
                Speed_Pid[1].SetPoint = sp3 - total_steer;  /* -steer → turn LEFT */
            } else if (run_flag) {
                Track_Direction_Control(g_track_speed_cm_s);
            } else {
                /* Idle: stop motors, reset TASK3 state */
                g_task3_steer = 0;
                g_task3_yaw_isum = 0;
                g_task3_last_yaw_err = 0;
                get_gray_refresh_data();
                Speed_Pid[0].SetPoint = 0;
                Speed_Pid[1].SetPoint = 0;
            }
        }

        /* ---- Odometry update every 128ms ---- */
        if (system_time_elapsed_ms(&odo_tick, 128))
        {
            float dL = get_motor_distance_cm(0);
            float dR = get_motor_distance_cm(1);
            Odometry_Update(dL, dR, Yaw);

            /* TASK3 encoder PI distance correction: keep L/R accumulated distance equal */
            if (g_task3_active && g_task3_running) {
                if (!g_task3_dist_init) {
                    g_task3_dist_L = 0;
                    g_task3_dist_R = 0;
                    g_task3_dist_isum = 0;
                    g_task3_yaw_isum = 0;
                    g_task3_last_yaw_err = 0;
                    g_task3_start_yaw = Yaw;  /* lock current heading */
                    g_task3_dist_init = 1;
                }

                g_task3_dist_L += dL;
                g_task3_dist_R += dR;

                /* Slow encoder I-only: trim wheel diameter diff, don't fight yaw */
                float diff = g_task3_dist_L - g_task3_dist_R;
                g_task3_dist_isum += diff * 0.3f;
                if (g_task3_dist_isum >  20.0f) g_task3_dist_isum =  20.0f;
                if (g_task3_dist_isum < -20.0f) g_task3_dist_isum = -20.0f;

                g_task3_steer = g_task3_dist_isum;  /* I-only, no P */
            } else {
                /* TASK3 not running — reset for next start */
                g_task3_dist_init = 0;
                g_task3_steer = 0;
            }
        }

        /* ---- Serial debug every 100ms ---- */
        if (system_time_elapsed_ms(&print_tick, 100))
        {
            const Odometry_t* odo = Odometry_GetData();

            if (g_task3_active && g_task3_running) {
                float L = get_motor_speed_cm_s(0);
                float R = get_motor_speed_cm_s(1);
                float yaw_err = g_task3_start_yaw - odo->yaw_deg;
                if (yaw_err >  180.0f) yaw_err -= 360.0f;
                if (yaw_err < -180.0f) yaw_err += 360.0f;
                float dy = odo->yaw_deg; if (dy < 0) dy += 360.0f;
                printf("T3|d=%.1f L=%.0f R=%.0f dLR=%.1f st=%.0f yaw=%.0f e=%.0f\r\n",
                       odo->total_dist, L, R,
                       g_task3_dist_L - g_task3_dist_R,
                       g_task3_steer, dy, yaw_err);
            } else {
                float dy = odo->yaw_deg; if (dy < 0) dy += 360.0f;
                printf("ODO| x=%.1f y=%.1f yaw=%.1f dist=%.1f\r\n",
                       odo->x, odo->y, dy, odo->total_dist);
            }

            /* TASK3 completion report: detect running→stopped edge */
            if (g_task3_was_running && !g_task3_running) {
                printf("=== TASK3 DONE ===\r\n");
                printf("  displayed: %.1f cm  (x=%.1f y=%.1f)\r\n",
                       odo->total_dist, odo->x, odo->y);
                printf("  L=%.1f R=%.1f  L-R=%.1f cm\r\n",
                       g_task3_dist_L, g_task3_dist_R,
                       g_task3_dist_L - g_task3_dist_R);
                printf("  |y| = %.1f cm  (%s)\r\n",
                       fabsf(odo->y),
                       fabsf(odo->y) < 5.0f ? "STRAIGHT OK" : "CURVED");
                printf("  --- CALIBRATION ---\r\n");
                printf("  measure REAL distance with ruler!\r\n");
                printf("  ratio = real / %.1f\r\n", odo->total_dist);
                printf("  new D = 6.5 * ratio = %.2f cm\r\n",
                       6.5f * (100.0f / odo->total_dist));
                printf("  (update WHEEL_DIAMETER_CM in motor.h)\r\n");
                printf("=================\r\n");
            }
            g_task3_was_running = g_task3_running;
        }

        /* ---- 5.4 INS task (unused) ---- */
//        if (system_time_elapsed_ms(&gw_tick, 10))
//        {
//            around();
//        }

        /* ---- Print IMU data (unused) ---- */
//        if (system_time_elapsed_ms(&print_tick, 25))
//        {
//            printf("Y:%.1f P:%.1f R:%.1f\r\n", ypr[0], ypr[1], ypr[2]);
//        }
    }
}

#pragma clang diagnostic pop
