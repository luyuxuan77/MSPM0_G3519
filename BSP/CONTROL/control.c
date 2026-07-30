#include "control.h"
#include "SPI0_LCD/lcd.h"

/* ===========================================================================
 * 普通循迹 — PD(带微分滤波) + 丢线保持 + 恒定速度
 * ---------------------------------------------------------------------------
 * 调用: main.c 每 10ms 调 Track_Direction_Control() 一次。
 * 反馈: gw_offset = get_gray_refresh_data() 连续质心偏差(≈±60, 右偏为正)。
 *
 * 要素:
 *   1. PD: P跟随 + D(滤波)阻尼震荡
 *   2. 丢线保持: 8路全丢线时保持上次转向
 *   3. 全程恒定速度, 不降速
 *
 * 调参:
 *   直道来回摆   → 调小 KP 或 调大 KD
 *   高频抖(噪声) → 调小 D_LPF
 *   循迹跟不住   → 调大 KP/KD
 *   丢线冲出     → 确认 LOST_HOLD, 或调小巡航速度
 * =========================================================================== */
#define TRACK_KP              2.5f
#define TRACK_KD              0.5f
#define TRACK_D_LPF           0.15f
#define TRACK_STEER_LIMIT     145.0f
#define TRACK_STEER_SLEW_STEP 14.0f  /* 转向变化率限制 */
#define TRACK_FWD_SLEW_STEP   3.6f   /* 前进速度变化率限制, 起步平缓 */
#define TRACK_LOST_HOLD      1       /* 1=丢线保持上次转向 0=丢线走直线 */

/* 兼容旧宏名(下方若有引用) */
#define TRACK_ERROR_GAIN     TRACK_KP

// Grayscale PID struct and P/I/D values (3 sets for different speeds)
PID_TypeDef Grayscale_Pid;
p_i_d_Value_TypeDef PID_Value_Grayscale[3] = {{  0.0, 0, 0 },   // preset P/I/D values
											  { 0.0, 0.0, 0.0 },
											  { 55.0, 0.0, 10.0 }};

// Grayscale Direction PID (reserved, not used in direct-error mode)
PID_TypeDef Grayscale_Direction_Pid;
p_i_d_Value_TypeDef PID_Value_Grayscale_Direction[3] = {{ 2.3, 0.5, 0.0},
														{ 0.0, 0.0, 0.0 },
														{ 0.0, 0.0, 0.0 }};

static Track_State_t track_state = TRACK_LINE;

static uint8_t left_corner_cnt = 0;
static uint8_t right_corner_cnt = 0;
static uint8_t turn_cnt = 0;
int gw_offset = 0;

//	imu角度环
PID_TypeDef Yaw_Pid;
p_i_d_Value_TypeDef PID_Value_Yaw={ 2.0, 0, 0.5};

void Track_Direction_Control(int speed_cm_s)
{
    static float track_last_err  = 0;   /* 上次偏差(D项) */
    static float track_dfilt     = 0;   /* 滤波后微分 */
    static float track_last_steer= 0;   /* 上次转向(丢线保持) */
    static float track_last_fwd  = 0;   /* 上次前进速度, 用于限速平滑 */

    /* 恒定速度 cm/s → 编码器计数 */
    float speed = cm_s_to_speed_counts((float)speed_cm_s);

    gw_offset = get_gray_refresh_data();     /* 连续质心偏差, 右偏为正 */
    uint8_t seen = get_offset_s();           /* 8路命中位图, 0=全丢线 */

    float error = (float)gw_offset;
    float steer;

    if (TRACK_LOST_HOLD && seen == 0) {
        /* 丢线: 保持上次转向, 微分清零避免跳变 */
        steer = track_last_steer;
        track_dfilt = 0;
        track_last_err = error;
    } else {
        /* PD: P跟随 + D(滤波)阻尼 */
        float d_raw = error - track_last_err;
        track_dfilt += (d_raw - track_dfilt) * TRACK_D_LPF;
        track_last_err = error;

        steer = error * TRACK_KP + track_dfilt * TRACK_KD;
        if (steer >  TRACK_STEER_LIMIT) steer =  TRACK_STEER_LIMIT;
        if (steer < -TRACK_STEER_LIMIT) steer = -TRACK_STEER_LIMIT;
    }

    /* 转向变化率限制 */
    float steer_delta = steer - track_last_steer;
    if (steer_delta > TRACK_STEER_SLEW_STEP) {
        steer = track_last_steer + TRACK_STEER_SLEW_STEP;
    } else if (steer_delta < -TRACK_STEER_SLEW_STEP) {
        steer = track_last_steer - TRACK_STEER_SLEW_STEP;
    }
    track_last_steer = steer;

    /* 恒定速度, 不降速 */
    float fwd = speed;

    /* 前进速度变化率限制 */
    if (track_last_fwd == 0) {
        track_last_fwd = fwd;
    }
    float fwd_delta = fwd - track_last_fwd;
    if (fwd_delta > TRACK_FWD_SLEW_STEP) {
        fwd = track_last_fwd + TRACK_FWD_SLEW_STEP;
    } else if (fwd_delta < -TRACK_FWD_SLEW_STEP) {
        fwd = track_last_fwd - TRACK_FWD_SLEW_STEP;
    }
    track_last_fwd = fwd;

    /* 差速转向: steer>0(线在右)→右转(左轮快,右轮慢) */
    float left  = fwd - steer;
    float right = fwd + steer;

    Speed_Pid[0].SetPoint = left;
    Speed_Pid[1].SetPoint = right;
}
//
//	LCD_ShowString(20, 90, "LEFT:",  BLACK, LIGHTBLUE, 16, 1);
//	LCD_ShowString(120,90, "RIGHT:", BLACK, LIGHTBLUE, 16, 1);
//	LCD_ShowString(20, 100,"ActualValue:", BLACK, WHITE, 16, 1);
//
//	LCD_ShowIntNum(50, 90, left, 4, BLACK, WHITE, 16);
//    LCD_ShowIntNum(160,90, right, 4, BLACK, WHITE, 16);

//	printf("error:%d  left:%.1f  right:%.1f\r\n", error, left, right);

float turn_start_yaw;

void Turn_Start()
{
    turn_start_yaw = Yaw;
}

int Turn_Left(float speed_cm_s)
{
    float x;
    x = turn_start_yaw - Yaw;

    if (x > 180)  x -= 360;
    if (x < -180) x += 360;

    if (x > 72)
    {
        Speed_Pid[0].SetPoint = 0;
        Speed_Pid[1].SetPoint = 0;
        return 1;
    }
    else if (x > 30)
    {
        Speed_Pid[0].SetPoint = 0;
        Speed_Pid[1].SetPoint = cm_s_to_speed_counts(speed_cm_s * 0.35f);
    }
    else
    {
        Speed_Pid[0].SetPoint = 0;
        Speed_Pid[1].SetPoint = cm_s_to_speed_counts(speed_cm_s);
    }
    return 0;
}

int Turn_Right(float speed_cm_s)
{
    float x;
    x = Yaw - turn_start_yaw;

    if (x > 180)  x -= 360;
    if (x < -180) x += 360;

    if (x > 74)
    {
        Speed_Pid[0].SetPoint = 0;
        Speed_Pid[1].SetPoint = 0;
        return 1;
    }
    else if (x > 30)
    {
        Speed_Pid[0].SetPoint = cm_s_to_speed_counts(speed_cm_s * 0.35f);
        Speed_Pid[1].SetPoint = 0;
    }
    else
    {
        Speed_Pid[0].SetPoint = cm_s_to_speed_counts(speed_cm_s);
        Speed_Pid[1].SetPoint = 0;
    }
    return 0;
}


float move_start_yaw;
float move_target_yaw;
uint8_t angle_move_flag;

void Angle_Move_Start(float angle)
{
//    move_start_yaw=Yaw;
    move_target_yaw=angle;

    if(move_target_yaw>180)
        move_target_yaw-=360;

    Yaw_Pid.SumError = 0;
    Yaw_Pid.LastError = 0;
    Yaw_Pid.PrevError = 0;
    angle_move_flag=1;
}

void Angle_Move_Update(float speed)
{
    float error;

    if(!angle_move_flag)
        return;

    error=Yaw-move_target_yaw;
    if(error>180)  error-=360;
    if(error<-180)error+=360;

    Pid_control(&Yaw_Pid,error,0);
    Pid_OutLimit(&Yaw_Pid,50);

    Speed_Pid[0].SetPoint=speed+Yaw_Pid.ActualValue*0.6;
    Speed_Pid[1].SetPoint=speed-Yaw_Pid.ActualValue*0.6;
//	LCD_ShowIntNum(50, 120, Yaw_Pid.ActualValue, 4, RED, WHITE, 24);
	printf("ActualValue = %.2f\r\n",Yaw_Pid.ActualValue*0.4);
}

static uint8_t move_init=0;
static uint8_t angle_init=0;
void Move_Angle(float speed,float angle)
{
    if(!move_init)
    {
        Angle_Move_Start(angle);
        move_init=1;
    }

    Angle_Move_Update(speed);
}

void Move_Angle_Reset(void)
{
    move_init = 0;
    angle_move_flag = 0;

    Yaw_Pid.SumError = 0;
    Yaw_Pid.LastError = 0;
    Yaw_Pid.PrevError = 0;
}

static float normalize_yaw_error(float angle)
{
    while(angle > 180)  angle -= 360;
    while(angle < -180) angle += 360;
    return angle;
}

void Spin_Turn_Reset(void)
{
    Yaw_Pid.SumError = 0;
    Yaw_Pid.LastError = 0;
    Yaw_Pid.PrevError = 0;
}

#define SPIN_TURN_KP        2.0f
#define SPIN_TURN_MIN_SPEED 70.0f
#define SPIN_TURN_TOLERANCE 2.0f

static int spin_turn_by_error(float speed, float error)
{
    float abs_error = (error < 0) ? -error : error;
    float turn_speed;

    if(speed < 0)
    {
        speed = -speed;
    }

    if(abs_error <= SPIN_TURN_TOLERANCE)
    {
        Speed_Pid[0].SetPoint = 0;
        Speed_Pid[1].SetPoint = 0;
        Spin_Turn_Reset();
        return 1;
    }

    // Proportional spin speed. Smaller error gives smaller speed to avoid shaking.
    turn_speed = abs_error * SPIN_TURN_KP;

    if(turn_speed > speed)
    {
        turn_speed = speed;
    }

    // Keep enough speed to overcome static friction before reaching tolerance.
    if(turn_speed < SPIN_TURN_MIN_SPEED)
    {
        turn_speed = SPIN_TURN_MIN_SPEED;
    }

    if(error > 0)
    {
        // Right spin: left wheel forward, right wheel backward.
        Speed_Pid[0].SetPoint = turn_speed;
        Speed_Pid[1].SetPoint = -turn_speed;
    }
    else
    {
        // Left spin: left wheel backward, right wheel forward.
        Speed_Pid[0].SetPoint = -turn_speed;
        Speed_Pid[1].SetPoint = turn_speed;
    }

    return 0;
}

int Spin_Turn(float speed, float angle)
{
    static uint8_t spin_init = 0;
    static float spin_start_yaw = 0;

    if(!spin_init)
    {
        spin_start_yaw = Yaw;
        spin_init = 1;
    }

    // Positive angle turns right, negative angle turns left.
    float turned = normalize_yaw_error(Yaw - spin_start_yaw);
    float error = normalize_yaw_error(angle - turned);

    if(spin_turn_by_error(speed, error))
    {
        spin_init = 0;
        return 1;
    }

    return 0;
}

int Spin_Turn_To_Angle(float speed, float target_angle)
{
    target_angle = normalize_yaw_error(target_angle);
    float error = normalize_yaw_error(target_angle - Yaw);

    return spin_turn_by_error(speed, error);
}
int road_tell()
{
    get_gray_refresh_data();
    u8 data = get_offset_s();

    if (data == 0)  // blank / no line detected
    {
        return 1;
    }

    // ===== Count left/right active sensors =====
    int left_count  = ((data >> 0) & 1) + ((data >> 1) & 1) + ((data >> 2) & 1) + ((data >> 3) & 1);
    int right_count = ((data >> 4) & 1) + ((data >> 5) & 1) + ((data >> 6) & 1) + ((data >> 7) & 1);
	int com_cnt = left_count + right_count;
  
    if (com_cnt >=3)
    {
        return 4;
    }

	// ===== T-junction: >=3 sensors on one side =====
    if (left_count >= 3 && right_count < 3)
    {
        return 2;  // Left T
    }
    if (right_count >= 3 && left_count < 3)
    {
        return 3;  // Right T
    }

    // ===== Cross: both sides >=4 =====

    return 0;
}

static uint8_t black_cnt=0;
static uint8_t white_cnt=0;
uint8_t check_black(uint8_t tell)
{
    static uint8_t cnt=0;

    if(tell!=1)
    {
        cnt++;
        if(cnt>=5)
        {
            cnt=0;
            return 1;
        }
    }
    else
    {
        cnt=0;
    }

    return 0;
}

uint8_t check_white(uint8_t tell)
{
    static uint8_t cnt=0;

    if(tell==1)
    {
        cnt++;
        if(cnt>=5)
        {
            cnt=0;
            return 1;
        }
    }
    else
    {
        cnt=0;
    }

    return 0;
}
