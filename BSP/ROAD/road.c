#include "bsp.h"

int route_finish = 0;
int target_round=1;

#define ROAD_ODOM_PERIOD_MS        128U

#define TASK2_START_IGNORE_CM       250.0f
#define TASK2_STOP_BLACK_COUNT       3U
#define TASK2_ALIGN_TURN_MS         50U
#define TASK2_ALIGN_TURN_RATIO       0.7f

/* ===== Smooth inertial path parameters for task4/5/6 ===== */
#define ROAD_STRAIGHT_CM           150.0f
#define ROAD_TASK4_STOP_CM         150.0f
#define ROAD_DECEL_DIST_CM          35.0f
#define ROAD_STOP_DIST_CM            2.0f
#define ROAD_CREEP_CM_S             8.0f
#define ROAD_STRAIGHT_DEFAULT_CM_S  35.0f
#define ROAD_TURN_CM_S              22.0f
#define ROAD_TURN_CREEP_CM_S        10.0f
#define ROAD_TURN_ANGLE_DEG        180.0f
#define ROAD_TURN_DECEL_DEG         35.0f
#define ROAD_LAP_FINISH_ARM_CM     430.0f

#define ROAD_ACCEL_STEP_CM_S         0.70f
#define ROAD_DECEL_STEP_CM_S         0.90f
#define ROAD_YAW_KP                  5.0f
#define ROAD_YAW_STEER_LIMIT        70.0f

/* Right half-circle tuning. Larger ratio = tighter/faster right turn. */
#define ROAD_TURN_RATIO              0.34f
#define ROAD_TURN_RATIO_STEP         0.025f

static uint8_t road_active_count(uint8_t data)
{
    uint8_t cnt = 0;
    for(uint8_t i = 0; i < 8; i++)
    {
        if(data & (1 << i)) cnt++;
    }
    return cnt;
}

static void road_update_odometry(uint32_t *last_tick)
{
    if(system_time_elapsed_ms(last_tick, ROAD_ODOM_PERIOD_MS))
    {
        float dL = get_motor_distance_cm(0);
        float dR = get_motor_distance_cm(1);
        Odometry_Update(dL, dR, Yaw);
    }
}

static float road_absf(float x)
{
    return (x < 0.0f) ? -x : x;
}

static float road_norm_angle(float angle)
{
    while(angle > 180.0f)  angle -= 360.0f;
    while(angle < -180.0f) angle += 360.0f;
    return angle;
}

static float road_ramp_to(float now, float target, float up_step, float down_step)
{
    if(target > now)
    {
        now += up_step;
        if(now > target) now = target;
    }
    else
    {
        now -= down_step;
        if(now < target) now = target;
    }
    return now;
}

static float road_profile_speed(float remain, float cruise, float creep, float decel_dist)
{
    float target = cruise;

    if(remain < decel_dist)
    {
        float ratio = remain / decel_dist;
        if(ratio < 0.0f) ratio = 0.0f;
        if(ratio > 1.0f) ratio = 1.0f;
        target = creep + (cruise - creep) * ratio;
    }

    if(target < creep) target = creep;
    return target;
}

static void road_set_speed_counts(float left_cm_s, float right_cm_s)
{
    Speed_Pid[0].SetPoint = cm_s_to_speed_counts(left_cm_s);
    Speed_Pid[1].SetPoint = cm_s_to_speed_counts(right_cm_s);
}

static uint8_t road_smooth_stop(float *cmd_speed)
{
    *cmd_speed = road_ramp_to(*cmd_speed, 0.0f, ROAD_ACCEL_STEP_CM_S, ROAD_DECEL_STEP_CM_S);
    road_set_speed_counts(*cmd_speed, *cmd_speed);

    if(*cmd_speed <= 0.1f)
    {
        Speed_Pid[0].SetPoint = 0;
        Speed_Pid[1].SetPoint = 0;
        return 1;
    }

    return 0;
}

static uint8_t road_drive_straight(float start_dist, float target_cm, float target_yaw,
                                   float cruise_cm_s, float *cmd_speed)
{
    const Odometry_t *odo = Odometry_GetData();
    float traveled = odo->total_dist - start_dist;
    float remain = target_cm - traveled;

    if(remain <= ROAD_STOP_DIST_CM)
    {
        return 1;
    }

    float target_speed = road_profile_speed(remain, cruise_cm_s, ROAD_CREEP_CM_S, ROAD_DECEL_DIST_CM);
    *cmd_speed = road_ramp_to(*cmd_speed, target_speed, ROAD_ACCEL_STEP_CM_S, ROAD_DECEL_STEP_CM_S);

    float yaw_err = road_norm_angle(target_yaw - Yaw);
    float steer = yaw_err * ROAD_YAW_KP;
    if(steer >  ROAD_YAW_STEER_LIMIT) steer =  ROAD_YAW_STEER_LIMIT;
    if(steer < -ROAD_YAW_STEER_LIMIT) steer = -ROAD_YAW_STEER_LIMIT;

    float sp = cm_s_to_speed_counts(*cmd_speed);
    Speed_Pid[0].SetPoint = sp + steer;
    Speed_Pid[1].SetPoint = sp - steer;

    return 0;
}

static uint8_t road_drive_right_arc_180(float start_yaw, float *cmd_speed, float *turn_ratio)
{
    float turned = road_norm_angle(Yaw - start_yaw);
    if(turned < 0.0f) turned += 360.0f;

    float remain = ROAD_TURN_ANGLE_DEG - turned;
    if(remain <= 2.0f)
    {
        return 1;
    }

    float target_speed = road_profile_speed(remain, ROAD_TURN_CM_S,
                                            ROAD_TURN_CREEP_CM_S, ROAD_TURN_DECEL_DEG);
    *cmd_speed = road_ramp_to(*cmd_speed, target_speed, ROAD_ACCEL_STEP_CM_S, ROAD_DECEL_STEP_CM_S);
    *turn_ratio = road_ramp_to(*turn_ratio, ROAD_TURN_RATIO, ROAD_TURN_RATIO_STEP, ROAD_TURN_RATIO_STEP);

    /* Right arc: left wheel faster, right wheel slower. */
    float left_cm_s  = *cmd_speed * (1.0f + *turn_ratio);
    float right_cm_s = *cmd_speed * (1.0f - *turn_ratio);

    road_set_speed_counts(left_cm_s, right_cm_s);
    return 0;
}

void task2(float speed)//整圈循迹: 直1.5m→弧R0.5m→直1.5m→弧R0.5m
{
    static uint8_t  init_done = 0;
    static uint32_t odom_tick = 0;
    static uint32_t start_ms  = 0;
    static uint8_t  phase = 0;
    static float    phase_start_cm = 0;

#define T2_STRAIGHT_LEN_CM    140.0f
#define T2_ARC_LEN_CM         183.0f
#define T2_TOTAL_CM           643.0f  /* +4cm */
#define T2_STRAIGHT_SPD        56.0f  /* -1 */
#define T2_ARC_SPD             30.0f  /* -0.2 */
#define T2_END_SPD              8.0f
#define T2_RAMP_CM             60.0f  /* 再提前10cm */
#define T2_ALIGN_CM             8.0f
#define T2_ALIGN_SPD            15.0f  /* 对准区降速 */
#define T2_YAW_KP               30.0f

    static float start_yaw = 0;  /* 出发时航向 */

    if (!init_done) {
        Odometry_Reset();
        motor_speed_pid_init();
        odom_tick = system_time_get_tick_ms();
        start_yaw = Yaw;
        phase     = 0;
        phase_start_cm = 0;
        g_arc_mode = 0;
        init_done = 1;
        buzzer_play(800, 50);
        delay_ms(100);
        buzzer_stop();
        start_ms  = system_time_get_tick_ms();  /* 蜂鸣器响完后开始计时 */
    }

    road_update_odometry(&odom_tick);

    const Odometry_t *odo = Odometry_GetData();
    float dist = odo->total_dist;

    /* ---- 阶段判定 ---- */
    float target_spd;
    float phase_dist = dist - phase_start_cm;
    float phase_remain;

    if (phase == 0 && phase_dist >= T2_STRAIGHT_LEN_CM) {
        phase = 1; phase_start_cm = dist; g_arc_mode = 1;  /* 进入圆弧 */
    } else if (phase == 1 && phase_dist >= T2_ARC_LEN_CM) {
        phase = 2; phase_start_cm = dist; g_arc_mode = 0;
    } else if (phase == 2 && phase_dist >= T2_STRAIGHT_LEN_CM) {
        phase = 3; phase_start_cm = dist; g_arc_mode = 1;
    }

    switch (phase) {
    case 0: target_spd = T2_STRAIGHT_SPD; phase_remain = T2_STRAIGHT_LEN_CM - phase_dist; break;
    case 1: target_spd = T2_ARC_SPD;     phase_remain = T2_ARC_LEN_CM - phase_dist;     break;
    case 2: target_spd = T2_STRAIGHT_SPD; phase_remain = T2_STRAIGHT_LEN_CM - phase_dist; break;
    case 3: default:
        target_spd = T2_ARC_SPD;
        phase_remain = T2_ARC_LEN_CM - phase_dist;
        if (phase_remain < 0) phase_remain = 0;
        break;
    }

    /* ---- 速度平滑 ---- */
    float cmd;
    float prev_spd = (phase == 0) ? T2_END_SPD
                   : (phase == 1) ? T2_STRAIGHT_SPD
                   : (phase == 2) ? T2_ARC_SPD
                   : T2_STRAIGHT_SPD;

    if (phase_dist < T2_RAMP_CM) {
        float ratio = phase_dist / T2_RAMP_CM;
        if (ratio > 1.0f) ratio = 1.0f;
        cmd = prev_spd + (target_spd - prev_spd) * ratio;
        if (phase == 0 && cmd < T2_END_SPD) cmd = T2_END_SPD;
    } else {
        cmd = target_spd;
    }
    if (cmd > T2_STRAIGHT_SPD) cmd = T2_STRAIGHT_SPD;
    if (cmd < T2_END_SPD && dist < T2_TOTAL_CM) cmd = T2_END_SPD;

    /* ---- 显示 ---- */
    {
        static char line_str[20] = "";
        char new_str[20];
        uint32_t elapsed = system_time_get_tick_ms() - start_ms;
        uint32_t sec = elapsed / 1000;
        uint32_t hs  = (elapsed % 1000) / 10;
        sprintf(new_str, "P%d %d.%02d %2u.%02u", phase+1,
                (int)dist / 100, (int)dist % 100,
                (unsigned)sec, (unsigned)hs);
        if (strcmp(line_str, new_str) != 0) {
            strcpy(line_str, new_str);
            LCD_ShowString(0, 40, (u8 *)new_str, BLUE, WHITE, 16, 0);
        }
    }

    /* ---- 终点 ---- */
    if (dist >= T2_TOTAL_CM) {
        init_done = 0;
        g_arc_mode = 0;
        uint32_t elapsed = system_time_get_tick_ms() - start_ms;
        uint32_t sec = elapsed / 1000;
        uint32_t hs  = (elapsed % 1000) / 10;
        char end_str[20];
        sprintf(end_str, "DONE %2u.%02us Y%+.0f", (unsigned)sec, (unsigned)hs,
                (double)road_norm_angle(Yaw - start_yaw));
        LCD_ShowString(0, 40, (u8 *)end_str, RED, WHITE, 16, 0);
        buzzer_play(1000, 50); delay_ms(80);
        buzzer_stop();
        menu_stop_task();
        return;
    }

    /* 终点前8cm: 降速 + 航向对准 */
    if (phase == 3 && phase_remain < T2_ALIGN_CM) {
        float align_cmd = (cmd < T2_ALIGN_SPD) ? cmd : T2_ALIGN_SPD;
        gw_offset = get_gray_refresh_data();
        float line_steer = (float)gw_offset * TRACK_KP;
        float yaw_err = road_norm_angle(Yaw - start_yaw);
        float yaw_steer = yaw_err * T2_YAW_KP;
        float steer = line_steer * 0.3f + yaw_steer;
        if (steer >  TRACK_STEER_LIMIT) steer =  TRACK_STEER_LIMIT;
        if (steer < -TRACK_STEER_LIMIT) steer = -TRACK_STEER_LIMIT;

        float fwd = cm_s_to_speed_counts(align_cmd);
        Speed_Pid[0].SetPoint = fwd - steer;
        Speed_Pid[1].SetPoint = fwd + steer;
    } else {
        Track_Direction_Control((int)cmd);
    }

    if (!g_menu_running) {
        Speed_Pid[0].SetPoint = 0;
        Speed_Pid[1].SetPoint = 0;
        init_done = 0;
        g_arc_mode = 0;
    }
#undef T2_STRAIGHT_LEN_CM
#undef T2_ARC_LEN_CM
#undef T2_TOTAL_CM
#undef T2_STRAIGHT_SPD
#undef T2_ARC_SPD
#undef T2_END_SPD
#undef T2_RAMP_CM
#undef T2_ALIGN_CM
#undef T2_ALIGN_SPD
#undef T2_YAW_KP
}

void task4(float speed)//循迹前进1.5m: PID自然加速 + 末段减速停车 (移植自task2)
{
    static uint8_t  init_done = 0;
    static uint32_t odom_tick = 0;
    static uint32_t start_ms  = 0;

#define TASK4_TOTAL_CM     150.0f   /* 总行程 1.5m */
#define TASK4_DECEL_CM      30.0f   /* 末段减速区长度 */
#define TASK4_CRUISE_CM_S   27.4f   /* 巡航速度 */
#define TASK4_MIN_CM_S       5.0f   /* 起步/末段最低速度 */

    if (!init_done) {
        Odometry_Reset();
        motor_speed_pid_init();
        odom_tick = system_time_get_tick_ms();
        start_ms  = system_time_get_tick_ms();
        init_done = 1;
        buzzer_play(800, 50);
        delay_ms(100);
        buzzer_stop();
    }

    road_update_odometry(&odom_tick);

    const Odometry_t *odo = Odometry_GetData();
    float dist = odo->total_dist;
    float remain = TASK4_TOTAL_CM - dist;

    float cmd;
    if (remain <= 0) {
        cmd = 0;
    } else if (remain < TASK4_DECEL_CM) {
        cmd = TASK4_MIN_CM_S + (TASK4_CRUISE_CM_S - TASK4_MIN_CM_S) * (remain / TASK4_DECEL_CM);
    } else {
        cmd = TASK4_CRUISE_CM_S;
    }
    if (cmd < TASK4_MIN_CM_S && remain > 0) cmd = TASK4_MIN_CM_S;

    /* 显示计时+距离 */
    {
        static char line_str[20] = "";
        char new_str[20];
        uint32_t elapsed = system_time_get_tick_ms() - start_ms;
        uint32_t sec = elapsed / 1000;
        uint32_t hs  = (elapsed % 1000) / 10;
        sprintf(new_str, "%d.%02dm %2u.%02us", (int)dist / 100, (int)dist % 100,
                (unsigned)sec, (unsigned)hs);
        if (strcmp(line_str, new_str) != 0) {
            strcpy(line_str, new_str);
            LCD_ShowString(0, 40, (u8 *)new_str, BLUE, WHITE, 16, 0);
        }
    }

    if (dist >= TASK4_TOTAL_CM) {
        init_done = 0;
        {
            uint32_t elapsed = system_time_get_tick_ms() - start_ms;
            uint32_t sec = elapsed / 1000;
            uint32_t hs  = (elapsed % 1000) / 10;
            char end_str[20];
            sprintf(end_str, "DONE %2u.%02us", (unsigned)sec, (unsigned)hs);
            LCD_ShowString(0, 40, (u8 *)end_str, RED, WHITE, 16, 0);
        }
        buzzer_play(1000, 50); delay_ms(80);
        buzzer_stop();
        menu_stop_task();
        return;
    }

    Track_Direction_Control((int)cmd);

    if (!g_menu_running) {
        Speed_Pid[0].SetPoint = 0;
        Speed_Pid[1].SetPoint = 0;
        init_done = 0;
    }
#undef TASK4_TOTAL_CM
#undef TASK4_DECEL_CM
#undef TASK4_CRUISE_CM_S
#undef TASK4_MIN_CM_S
}

static void road_lap_task(float speed)
{
    static int step = 0;
    static uint32_t odom_tick = 0;
    static float segment_start_dist = 0.0f;
    static float segment_start_yaw = 0.0f;
    static float target_yaw = 0.0f;
    static float cmd_speed = 0.0f;
    static float turn_ratio = 0.0f;

    if(step == 7 && route_finish == 0)
    {
        step = 0;
    }

    switch(step)
    {
    case 0:
        Move_Angle_Reset();
        Odometry_Reset();
        motor_speed_pid_init();
        odom_tick = nowtime;
        segment_start_dist = 0.0f;
        segment_start_yaw = Yaw;
        target_yaw = Yaw;
        cmd_speed = 0.0f;
        turn_ratio = 0.0f;
        route_finish = 0;
        step = 1;
        break;

    case 1: /* straight 1.5m */
        road_update_odometry(&odom_tick);
        if(road_drive_straight(segment_start_dist, ROAD_STRAIGHT_CM, target_yaw, speed, &cmd_speed))
        {
            segment_start_yaw = Yaw;
            turn_ratio = 0.0f;
            step = 2;
        }
        break;

    case 2: /* right half-circle */
        road_update_odometry(&odom_tick);
        if(road_drive_right_arc_180(segment_start_yaw, &cmd_speed, &turn_ratio))
        {
            const Odometry_t *odo = Odometry_GetData();
            segment_start_dist = odo->total_dist;
            target_yaw = Yaw;
            step = 3;
        }
        break;

    case 3: /* straight 1.5m */
        road_update_odometry(&odom_tick);
        if(road_drive_straight(segment_start_dist, ROAD_STRAIGHT_CM, target_yaw, speed, &cmd_speed))
        {
            segment_start_yaw = Yaw;
            turn_ratio = 0.0f;
            step = 4;
        }
        break;

    case 4: /* second right half-circle */
        road_update_odometry(&odom_tick);
        if(road_drive_right_arc_180(segment_start_yaw, &cmd_speed, &turn_ratio))
        {
            step = 5;
        }
        break;

    case 5: /* final approach: inertial path is done, use stop line only near finish */
    {
        road_update_odometry(&odom_tick);
        int tell = road_tell();
        const Odometry_t *odo = Odometry_GetData();

        if(odo->total_dist >= ROAD_LAP_FINISH_ARM_CM && tell == 4)
        {
            step = 6;
        }
        else
        {
            /* Slow final approach, keep current heading stable. */
            road_drive_straight(odo->total_dist, 9999.0f, Yaw, ROAD_TURN_CM_S, &cmd_speed);
        }
        break;
    }

    case 6:
        if(road_smooth_stop(&cmd_speed))
        {
            route_finish = 1;
            step = 7;
        }
        break;

    case 7:
        Speed_Pid[0].SetPoint = 0;
        Speed_Pid[1].SetPoint = 0;
        break;
    }
}

void task5(float speed)//一圈惯导: 直线1.5m + 右半圆 + 直线1.5m + 右半圆
{
    road_lap_task(speed);
}

void task6(float speed)//同task5路径, 球任意目标位置由上位机/菜单任务处理
{
    road_lap_task(speed);
}

