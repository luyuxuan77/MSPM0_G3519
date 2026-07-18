#include "control.h"
#include "SPI0_LCD/lcd.h"

// ===== Direct-error steering config =====
#define TRACK_ERROR_GAIN     3.4f   // steering gain factor
#define TRACK_STEER_LIMIT    20.0f  // ± steer clamp limit (applied to motor speed), tunable

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

void Track_Direction_Control(int speed)
{
    gw_offset = get_gray_refresh_data();
//	printf("gw_offset:%d\r\n", gw_offset);

    // ---- Direct centroid error (no PID) ----
    int error = gw_offset;

    // steer = error * gain, then clamp the result
    float steer = error * TRACK_ERROR_GAIN;
    if (steer >  TRACK_STEER_LIMIT) steer =  TRACK_STEER_LIMIT;
    if (steer < -TRACK_STEER_LIMIT) steer = -TRACK_STEER_LIMIT;

    // LCD: show steer value (clear then draw, same x=108 for both signs)
    LCD_Fill(108, 65, 155, 80, WHITE);  // 3 digits × 16px = 48px width
    if (steer < 0)
    {
        LCD_ShowString(108, 65, "-", BLACK, WHITE, 16, 1);
        LCD_ShowIntNum(116, 65, -(int)steer, 3, BLACK, WHITE, 16);
    }
    else
    {
        LCD_ShowIntNum(108, 65, (int)steer, 3, BLACK, WHITE, 16);
    }

    // Differential steering: steer > 0 (line to the right) → turn right
    float left  = speed + steer;
    float right = speed - steer;

//    Speed_Pid[0].SetPoint = left;
//    Speed_Pid[1].SetPoint = right;
//	set_motor_speed(-left, -right, 0, 0);
//
//	LCD_ShowString(20, 90, "LEFT:",  BLACK, LIGHTBLUE, 16, 1);
//	LCD_ShowString(120,90, "RIGHT:", BLACK, LIGHTBLUE, 16, 1);
//	LCD_ShowString(20, 100,"ActualValue:", BLACK, WHITE, 16, 1);
//
//	LCD_ShowIntNum(50, 90, left, 4, BLACK, WHITE, 16);
//    LCD_ShowIntNum(160,90, right, 4, BLACK, WHITE, 16);

//	printf("error:%d  left:%.1f  right:%.1f\r\n", error, left, right);
}

float turn_start_yaw;

void Turn_Start()
{
    turn_start_yaw = Yaw;
}

int Turn_Left(float speed)
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
        Speed_Pid[1].SetPoint = 30;
    }
    else
    {
        Speed_Pid[0].SetPoint = 0;
        Speed_Pid[1].SetPoint = 80;
    }
    return 0;
}

int Turn_Right(float speed)
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
        Speed_Pid[0].SetPoint = 30;
        Speed_Pid[1].SetPoint = 0;
    }
    else
    {
        Speed_Pid[0].SetPoint = 80;
        Speed_Pid[1].SetPoint = 0;
    }
    return 0;
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

    // ===== T-junction: >=3 sensors on one side =====
    if (left_count >= 3 && right_count < 3)
    {
        return 2;  // Left T
    }
    if (right_count >= 3 && left_count < 3)
    {
        return 3;  // Right T
    }

    // ===== Cross: both sides >=3 =====
    if (left_count >= 3 && right_count >= 3)
    {
        return 4;
    }

    return 0;
}
