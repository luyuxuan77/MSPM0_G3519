#ifndef PID_H
#define PID_H

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "math.h"

//PID结构体变量，储存PID各参数
typedef struct
{
	float SetPoint;		/*目标值*/
	float ActualValue;	/*期望值*/
	float SumError;		/*累计偏差*/
	float Proportion;	/*比例系数*/
	float Integral;		/*积分系数*/
	float Derivative;	/*微分系数*/
	float Error;		/*此次偏差*/
	float LastError;	/*上次偏差*/
	float PrevError;	/*上上次偏差*/
}PID_TypeDef;			/*pid算法中用到的参数集合*/
extern PID_TypeDef Position_position_Pid[4];
extern PID_TypeDef Position_speed_Pid[4];
extern PID_TypeDef Speed_Pid[4];

#define L1_Position_position_Pid	Position_position_Pid[0]	//将Position_Pid[0]重命名为L1_Position_Pid，方便代码编写
#define R1_Position_position_Pid	Position_position_Pid[1]	//将Position_Pid[1]重命名为R1_Position_Pid，方便代码编写
#define L2_Position_position_Pid	Position_position_Pid[2]	//将Position_Pid[2]重命名为L2_Position_Pid，方便代码编写
#define R2_Position_position_Pid	Position_position_Pid[3]	//将Position_Pid[3]重命名为R2_Position_Pid，方便代码编写
#define L1_Position_speed_Pid	Position_speed_Pid[0]	//将Position_speed_Pid[0]重命名为L1_Position_Pid，方便代码编写
#define R1_Position_speed_Pid	Position_speed_Pid[1]	//将Position_speed_Pid[1]重命名为R1_Position_Pid，方便代码编写
#define L2_Position_speed_Pid	Position_speed_Pid[2]	//将Position_speed_Pid[2]重命名为L2_Position_Pid，方便代码编写
#define R2_Position_speed_Pid	Position_speed_Pid[3]	//将Position_speed_Pid[3]重命名为R2_Position_Pid，方便代码编写

#define L1_Speed_Pid	Speed_Pid[0]	//将Speed_Pid[0]重命名为L1_Speed_Pid，方便代码编写
#define R1_Speed_Pid	Speed_Pid[1]	//将Speed_Pid[1]重命名为R1_Speed_Pid，方便代码编写
#define L2_Speed_Pid	Speed_Pid[2]	//将Speed_Pid[2]重命名为L2_Speed_Pid，方便代码编写
#define R2_Speed_Pid	Speed_Pid[3]	//将Speed_Pid[3]重命名为R2_Speed_Pid，方便代码编写
 
//p，i，d三值结构体，储存某pid参数集合中的p，i，d三值
typedef struct
{
	float KP;			/*比例常数*/
	float KI;			/*积分常数*/
	float KD;			/*微分常数*/
}p_i_d_Value_TypeDef;	/*比例，积分，微分3值大小结构体*/
extern p_i_d_Value_TypeDef PID_Value_Position_position[4];
extern p_i_d_Value_TypeDef PID_Value_Position_speed[4];
extern p_i_d_Value_TypeDef PID_Value_Speed[4];

#define POSITION_Position_PID_TYPE	0	/*位置控制位置环PID算法类型，0位置式，1增量式*/
#define POSITION_Speed_PID_TYPE 	0	/*位置控制位置环PID算法类型，0位置式，1增量式*/
#define SPEED_PID_TYPE				0	/*速度控制PID算法类型，0位置式，1增量式*/

#define L1_PID_Value_Position_position	PID_Value_Position_position[0]	//将0号电机定义为左1轮，PID_Value_Position[0]为0号电机位置控制位置环p，i，d三值（用于储存0号电机p，i，d三值）
#define R1_PID_Value_Position_position	PID_Value_Position_position[1]	//将1号电机定义为右1轮，PID_Value_Position[1]为1号电机位置控制位置环p，i，d三值（用于储存1号电机p，i，d三值）
#define L2_PID_Value_Position_position	PID_Value_Position_position[2]	//将2号电机定义为左2轮，PID_Value_Position[2]为2号电机位置控制位置环p，i，d三值（用于储存2号电机p，i，d三值）
#define R2_PID_Value_Position_position	PID_Value_Position_position[3]	//将3号电机定义为右2轮，PID_Value_Position[3]为3号电机位置控制位置环p，i，d三值（用于储存3号电机p，i，d三值）
#define L1_PID_Value_Position_speed	PID_Value_Position_speed[0]	//将0号电机定义为左1轮，PID_Value_Position[0]为0号电机位置控制速度环p，i，d三值（用于储存0号电机p，i，d三值）
#define R1_PID_Value_Position_speed	PID_Value_Position_speed[1]	//将1号电机定义为右1轮，PID_Value_Position[1]为1号电机位置控制速度环p，i，d三值（用于储存1号电机p，i，d三值）
#define L2_PID_Value_Position_speed	PID_Value_Position_speed[2]	//将2号电机定义为左2轮，PID_Value_Position[2]为2号电机位置控制速度环p，i，d三值（用于储存2号电机p，i，d三值）
#define R2_PID_Value_Position_speed	PID_Value_Position_speed[3]	//将3号电机定义为右2轮，PID_Value_Position[3]为3号电机位置控制速度环p，i，d三值（用于储存3号电机p，i，d三值）

#define L1_PID_Value_Speed	PID_Value_Speed[0]	//将0号电机定义为左1轮，PID_Value_Position[0]为0号电机速度控制p，i，d三值（用于储存0号电机p，i，d三值）
#define R1_PID_Value_Speed	PID_Value_Speed[1]	//将1号电机定义为右1轮，PID_Value_Position[1]为1号电机速度控制p，i，d三值（用于储存1号电机p，i，d三值）
#define L2_PID_Value_Speed	PID_Value_Speed[2]	//将2号电机定义为左2轮，PID_Value_Position[2]为2号电机速度控制p，i，d三值（用于储存2号电机p，i，d三值）
#define R2_PID_Value_Speed	PID_Value_Speed[3]	//将3号电机定义为右2轮，PID_Value_Position[3]为3号电机速度控制p，i，d三值（用于储存3号电机p，i，d三值）

void Pid_Init( PID_TypeDef* PID, p_i_d_Value_TypeDef* pid_Value );
int32_t Pid_control( PID_TypeDef* PID, float Feedback_value, unsigned char Mode );
int32_t Pid_OutLimit( PID_TypeDef* PID, int32_t limit);
#endif




