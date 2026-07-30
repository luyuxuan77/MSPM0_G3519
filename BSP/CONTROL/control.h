 #ifndef __CONTROL_H
#define __CONTROL_H
#include "bsp.h"
extern PID_TypeDef Grayscale_Pid;
extern p_i_d_Value_TypeDef PID_Value_Grayscale[3];

extern PID_TypeDef Grayscale_Direction_Pid;
extern p_i_d_Value_TypeDef PID_Value_Grayscale_Direction[3];

extern PID_TypeDef Yaw_Pid;
extern p_i_d_Value_TypeDef PID_Value_Yaw;


void Track_Direction_Control(int speed);
int Track_Direction_Control_LEFT(float speed);
int Track_Direction_Control_RIGHT(float speed);	
int Track_Direction_Control_reverse (float speed);
int road_tell();

void Turn_Start(void);
int Turn_Left(float speed);
int Turn_Right(float speed);

void Angle_Move_Start(float angle);
void Angle_Move_Update(float speed);
void Move_Angle(float speed,float angle);
void Angle_Move_Clear(void);
void Move_Angle_Reset();
void Spin_Turn_Reset(void);
int Spin_Turn(float speed, float angle);
int Spin_Turn_To_Angle(float speed, float target_angle);

extern float target_yaw;
typedef enum
{
    TRACK_LINE = 0,
    TRACK_TURN_LEFT,
    TRACK_TURN_RIGHT

}Track_State_t;

uint8_t check_black(uint8_t tell);
uint8_t check_white(uint8_t tell);

#endif

