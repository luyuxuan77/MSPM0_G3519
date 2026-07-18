 #ifndef __CONTROL_H
#define __CONTROL_H
#include "bsp.h"
extern PID_TypeDef Grayscale_Pid;
extern p_i_d_Value_TypeDef PID_Value_Grayscale[3];

extern PID_TypeDef Grayscale_Direction_Pid;
extern p_i_d_Value_TypeDef PID_Value_Grayscale_Direction[3];


void Track_Direction_Control(int speed);
int Track_Direction_Control_LEFT(float speed);
int Track_Direction_Control_RIGHT(float speed);	
int Track_Direction_Control_reverse (float speed);
int road_tell();

void Turn_Start(void);
int Turn_Left(float speed);
int Turn_Right(float speed);

typedef enum
{
    TRACK_LINE = 0,
    TRACK_TURN_LEFT,
    TRACK_TURN_RIGHT

}Track_State_t;

#endif

