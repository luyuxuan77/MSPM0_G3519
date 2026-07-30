#include "bsp.h"
#include "yuntai.h"
static float    yuntai_limit_angle(float angle, float min_deg, float max_deg);
static uint32_t yuntai_deg_to_pulses(float deg);
static void     yuntai_move_motor(uint8_t addr, float angle_deg, uint16_t speed, uint8_t acc);

static yuntai_state_t s_yuntai_state = {
    .yaw_angle   = 0.0f,
    .pitch_angle = 0.0f,
    .is_moving   = false,
    .is_enabled  = false
};

static float yuntai_limit_angle(float angle, float min_deg, float max_deg)
{
    if (angle < min_deg) return min_deg;
    if (angle > max_deg) return max_deg;
    return angle;
}

static uint32_t yuntai_deg_to_pulses(float deg)
{
    float p = deg * YT_PULSES_PER_DEG;
    if (p < 0.0f) p = -p;
    return (uint32_t)p;
}

static void yuntai_move_motor(uint8_t addr, float angle_deg,
                               uint16_t speed, uint8_t acc)
{
    uint8_t  dir;
    uint32_t pulses;
    if (angle_deg >= 0.0f) { dir = 0; }
    else { dir = 1; angle_deg = -angle_deg; }
    pulses = yuntai_deg_to_pulses(angle_deg);
    Emm_V5_Pos_Control(addr, dir, speed, acc, pulses, 1, false);
}

void yuntai_control_init(void)
{
    printf("[YUNTAI] init\r\n");
    uart3_init(115200U);
    delay_ms(10);
    s_yuntai_state.yaw_angle   = 0.0f;
    s_yuntai_state.pitch_angle = 0.0f;
    s_yuntai_state.is_moving   = false;
    s_yuntai_state.is_enabled  = false;
}

void yuntai_enable(bool enable)
{
    Emm_V5_En_Control(MOTOR_YAW_ADDR,   enable, false);
    delay_ms(10);
    Emm_V5_En_Control(MOTOR_PITCH_ADDR, enable, false);
    delay_ms(10);
    s_yuntai_state.is_enabled = enable;
    printf("[YUNTAI] %s\r\n", enable ? "enabled" : "disabled");
}

void yuntai_reset_position(void)
{
    Emm_V5_Reset_CurPos_To_Zero(MOTOR_YAW_ADDR);
    delay_ms(20);
    Emm_V5_Reset_CurPos_To_Zero(MOTOR_PITCH_ADDR);
    delay_ms(20);
    s_yuntai_state.yaw_angle   = 0.0f;
    s_yuntai_state.pitch_angle = 0.0f;
    printf("[YUNTAI] position reset\r\n");
}

void yuntai_move_to_angle(float yaw_deg, float pitch_deg)
{
    float yaw_delta, pitch_delta;
    yaw_deg   = yuntai_limit_angle(yaw_deg,   YT_YAW_MIN_DEG,   YT_YAW_MAX_DEG);
    pitch_deg = yuntai_limit_angle(pitch_deg, YT_PITCH_MIN_DEG, YT_PITCH_MAX_DEG);
    yaw_delta   = yaw_deg   - s_yuntai_state.yaw_angle;
    pitch_delta = pitch_deg - s_yuntai_state.pitch_angle;
    yuntai_move_motor(MOTOR_YAW_ADDR,   yaw_delta,   YT_DEFAULT_SPEED, YT_DEFAULT_ACC);
    yuntai_move_motor(MOTOR_PITCH_ADDR, pitch_delta, YT_DEFAULT_SPEED, YT_DEFAULT_ACC);
    s_yuntai_state.yaw_angle   = yaw_deg;
    s_yuntai_state.pitch_angle = pitch_deg;
    s_yuntai_state.is_moving   = true;
}

void yuntai_move_relative(float yaw_delta, float pitch_delta)
{
    yuntai_move_to_angle(s_yuntai_state.yaw_angle   + yaw_delta,
                         s_yuntai_state.pitch_angle + pitch_delta);
}

void yuntai_move_to_angle_fast(float yaw_deg, float pitch_deg)
{
    float yaw_delta, pitch_delta;
    yaw_deg   = yuntai_limit_angle(yaw_deg,   YT_YAW_MIN_DEG,   YT_YAW_MAX_DEG);
    pitch_deg = yuntai_limit_angle(pitch_deg, YT_PITCH_MIN_DEG, YT_PITCH_MAX_DEG);
    yaw_delta   = yaw_deg   - s_yuntai_state.yaw_angle;
    pitch_delta = pitch_deg - s_yuntai_state.pitch_angle;
    yuntai_move_motor(MOTOR_YAW_ADDR,   yaw_delta,   YT_DEFAULT_SPEED + 200, YT_DEFAULT_ACC + 30);
    yuntai_move_motor(MOTOR_PITCH_ADDR, pitch_delta, YT_DEFAULT_SPEED + 200, YT_DEFAULT_ACC + 30);
    s_yuntai_state.yaw_angle   = yaw_deg;
    s_yuntai_state.pitch_angle = pitch_deg;
    s_yuntai_state.is_moving   = true;
}

void yuntai_move_to_angle_fine(float yaw_deg, float pitch_deg)
{
    float yaw_delta, pitch_delta;
    yaw_deg   = yuntai_limit_angle(yaw_deg,   YT_YAW_MIN_DEG,   YT_YAW_MAX_DEG);
    pitch_deg = yuntai_limit_angle(pitch_deg, YT_PITCH_MIN_DEG, YT_PITCH_MAX_DEG);
    yaw_delta   = yaw_deg   - s_yuntai_state.yaw_angle;
    pitch_delta = pitch_deg - s_yuntai_state.pitch_angle;
    yuntai_move_motor(MOTOR_YAW_ADDR,   yaw_delta,   YT_FINE_SPEED, YT_FINE_ACC);
    yuntai_move_motor(MOTOR_PITCH_ADDR, pitch_delta, YT_FINE_SPEED, YT_FINE_ACC);
    s_yuntai_state.yaw_angle   = yaw_deg;
    s_yuntai_state.pitch_angle = pitch_deg;
    s_yuntai_state.is_moving   = true;
}

void yuntai_stop(void)
{
    Emm_V5_Stop_Now(MOTOR_YAW_ADDR,   false);
    Emm_V5_Stop_Now(MOTOR_PITCH_ADDR, false);
    s_yuntai_state.is_moving = false;
}

void yuntai_get_state(yuntai_state_t *state)
{
    if (state != NULL) *state = s_yuntai_state;
}

bool yuntai_auto_aim(target_info_t *target)
{
    const float threshold = 0.1f;   /* ~10% FOV dead-band */
    float ex, ey, sx, sy;
    uint8_t dx, dy;
    if (target == NULL || !target->target_found) return false;
    ex = target->target_x;
    ey = target->target_y;
    if (ABS(ex) < threshold && ABS(ey) < threshold) {
        Emm_V5_Stop_Now(MOTOR_YAW_ADDR,   false);
        Emm_V5_Stop_Now(MOTOR_PITCH_ADDR, false);
        s_yuntai_state.is_moving = false;
        return true;
    }
    if (ABS(ex) >= threshold) {
        dx = (ex > 0.0f) ? 0U : 1U;
        sx = ABS(ex) * 80.0f;
        if (sx <  20.0f) sx =  20.0f;
        if (sx > 250.0f) sx = 250.0f;
        Emm_V5_Vel_Control(MOTOR_YAW_ADDR, dx, (uint16_t)sx, 0, false);
    } else {
        Emm_V5_Stop_Now(MOTOR_YAW_ADDR, false);
    }
    if (ABS(ey) >= threshold) {
        dy = (ey > 0.0f) ? 0U : 1U;
        sy = ABS(ey) * 80.0f;
        if (sy <  20.0f) sy =  20.0f;
        if (sy > 250.0f) sy = 250.0f;
        Emm_V5_Vel_Control(MOTOR_PITCH_ADDR, dy, (uint16_t)sy, 0, false);
    } else {
        Emm_V5_Stop_Now(MOTOR_PITCH_ADDR, false);
    }
    s_yuntai_state.is_moving = true;
    return false;
}

void yuntai_test_scan(void)
{
    yuntai_enable(true);
    delay_ms(100);
    yuntai_reset_position();
    delay_ms(500);
    yuntai_move_to_angle( 30.0f, 0.0f); delay_ms(2000);
    yuntai_move_to_angle(-30.0f, 0.0f); delay_ms(2000);
    yuntai_move_to_angle(  0.0f, 0.0f); delay_ms(1000);
    yuntai_move_to_angle(0.0f,  20.0f); delay_ms(2000);
    yuntai_move_to_angle(0.0f, -20.0f); delay_ms(2000);
    yuntai_move_to_angle(0.0f,   0.0f); delay_ms(1000);
    yuntai_enable(false);
    printf("[YUNTAI] scan done\r\n");
}

bool yuntai_read_motor_position(uint8_t addr, int32_t *pulses_out)
{
    extern uint8_t uart3_rec_data[30];
    extern uint8_t uart3_count;

    uint32_t t0 = system_time_get_tick_ms();
    uint8_t  found = 0;
    int32_t  result = 0;

    uart3_count = 0;
    memset(uart3_rec_data, 0, sizeof(uart3_rec_data));

    Emm_V5_Read_Sys_Params(addr, S_CPOS);

    while ((system_time_get_tick_ms() - t0) < 200U) {
        if (uart3_count >= 8) {
            for (uint8_t i = 0; (i + 7) < uart3_count; i++) {
                if (uart3_rec_data[i]   == addr &&
                    uart3_rec_data[i+1] == 0x36 &&
                    uart3_rec_data[i+7] == 0x6B) {
                    uint8_t  dir = uart3_rec_data[i+2];
                    uint32_t raw = ((uint32_t)uart3_rec_data[i+3] << 24)
                                 | ((uint32_t)uart3_rec_data[i+4] << 16)
                                 | ((uint32_t)uart3_rec_data[i+5] <<  8)
                                 |  (uint32_t)uart3_rec_data[i+6];
                    float angle_deg = (float)raw * 0.005f;
                    if (dir != 0) angle_deg = -angle_deg;
                    result = (int32_t)(angle_deg * YT_PULSES_PER_DEG);
                    found  = 1;
                    break;
                }
            }
            if (found) break;
            /* frame not found yet — keep waiting for more bytes */
        }
        for (volatile uint32_t d = 0; d < 1000; d++);
    }

    if (found) { *pulses_out = result; return true; }
    return false;
}
