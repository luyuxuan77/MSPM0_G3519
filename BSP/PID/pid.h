#ifndef PID_H
#define PID_H

#include "bsp_common.h"

/*
 * PID structure — holds all state for one PID controller instance.
 *
 * Members:
 *   SetPoint    — target value
 *   ActualValue — output value (computed by Pid_control)
 *   SumError    — accumulated error (positional PID integral)
 *   Proportion  — Kp coefficient
 *   Integral    — Ki coefficient
 *   Derivative  — Kd coefficient
 *   Error       — current error
 *   LastError   — previous error (for D term)
 *   PrevError   — error before LastError (for incremental PID)
 */
typedef struct
{
    float SetPoint;
    float ActualValue;
    float SumError;
    float Proportion;
    float Integral;
    float Derivative;
    float Error;
    float LastError;
    float PrevError;
} PID_TypeDef;

extern PID_TypeDef Position_position_Pid[4];
extern PID_TypeDef Position_speed_Pid[4];
extern PID_TypeDef Speed_Pid[4];

/* Convenience aliases for Speed_Pid[] */
#define L1_Speed_Pid    Speed_Pid[0]   /* M0 (left)  speed PID */
#define R1_Speed_Pid    Speed_Pid[1]   /* M1 (right) speed PID */
#define L2_Speed_Pid    Speed_Pid[2]   /* reserved */
#define R2_Speed_Pid    Speed_Pid[3]   /* reserved */

/* Convenience aliases for Position_position_Pid[] */
#define L1_Position_position_Pid  Position_position_Pid[0]
#define R1_Position_position_Pid  Position_position_Pid[1]
#define L2_Position_position_Pid  Position_position_Pid[2]
#define R2_Position_position_Pid  Position_position_Pid[3]

/* Convenience aliases for Position_speed_Pid[] */
#define L1_Position_speed_Pid     Position_speed_Pid[0]
#define R1_Position_speed_Pid     Position_speed_Pid[1]
#define L2_Position_speed_Pid     Position_speed_Pid[2]
#define R2_Position_speed_Pid     Position_speed_Pid[3]

/*
 * p_i_d_Value_TypeDef — triplet of Kp, Ki, Kd for one controller.
 */
typedef struct
{
    float KP;
    float KI;
    float KD;
} p_i_d_Value_TypeDef;

extern p_i_d_Value_TypeDef PID_Value_Position_position[4];
extern p_i_d_Value_TypeDef PID_Value_Position_speed[4];
extern p_i_d_Value_TypeDef PID_Value_Speed[4];

/* PID algorithm type selection: 0=positional, 1=incremental */
#define POSITION_Position_PID_TYPE  0
#define POSITION_Speed_PID_TYPE     0
#define SPEED_PID_TYPE              0

/* Convenience aliases for PID_Value_Speed[] */
#define L1_PID_Value_Speed          PID_Value_Speed[0]
#define R1_PID_Value_Speed          PID_Value_Speed[1]
#define L2_PID_Value_Speed          PID_Value_Speed[2]
#define R2_PID_Value_Speed          PID_Value_Speed[3]

/* Convenience aliases for PID_Value_Position_position[] */
#define L1_PID_Value_Position_position  PID_Value_Position_position[0]
#define R1_PID_Value_Position_position  PID_Value_Position_position[1]
#define L2_PID_Value_Position_position  PID_Value_Position_position[2]
#define R2_PID_Value_Position_position  PID_Value_Position_position[3]

/* Convenience aliases for PID_Value_Position_speed[] */
#define L1_PID_Value_Position_speed     PID_Value_Position_speed[0]
#define R1_PID_Value_Position_speed     PID_Value_Position_speed[1]
#define L2_PID_Value_Position_speed     PID_Value_Position_speed[2]
#define R2_PID_Value_Position_speed     PID_Value_Position_speed[3]

void Pid_Init(PID_TypeDef* PID, p_i_d_Value_TypeDef* pid_Value);
int32_t Pid_control(PID_TypeDef* PID, float Feedback_value, unsigned char Mode);
int32_t Pid_OutLimit(PID_TypeDef* PID, int32_t limit);

#endif
