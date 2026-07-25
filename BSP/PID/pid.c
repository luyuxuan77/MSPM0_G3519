#include "PID/pid.h"

/*
 * ===========================================================================
 * PID Controller Module — ported from Car_sum project
 * ===========================================================================
 *
 * Supports both positional (Mode=0) and incremental (Mode=1) PID.
 *
 * Speed PID parameters (positional, SPEED_PID_TYPE == 0):
 *   [0] Left front  (M0): Kp=1.20, Ki=0.50, Kd=0.40
 *   [1] Right front (M1): Kp=1.15, Ki=0.50, Kd=0.40
 *   [2] Left rear  (reserved)
 *   [3] Right rear (reserved)
 *
 * Integral anti-windup: SumError clamped to +/-4000.
 * Output limiting: use Pid_OutLimit() after Pid_control().
 *
 * Tuning tips:
 *   1. Set Ki=0, Kd=0, increase Kp until speed reaches target with slight oscillation
 *   2. Add Ki gradually until steady-state error is eliminated
 *   3. Add Kd to dampen residual oscillation
 *   4. Verify at multiple speed levels
 * ===========================================================================
 */

/* PID instances for 3 control loops (4 motors each) */
PID_TypeDef Position_position_Pid[4];  /* Position control — outer position loop */
PID_TypeDef Position_speed_Pid[4];     /* Position control — inner speed loop */
PID_TypeDef Speed_Pid[4];              /* Independent speed control */

/* ---- Position-position PID parameters ---- */
#if POSITION_Position_PID_TYPE == 0
p_i_d_Value_TypeDef PID_Value_Position_position[4] = {
    { 0.48, 0.0, 0.1 },
    { 0.48, 0.0, 0.1 },
    { 0.48, 0.0, 0.1 },
    { 0.48, 0.0, 0.1 }
};
#else
p_i_d_Value_TypeDef PID_Value_Position_position[4] = {
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 }
};
#endif

/* ---- Position-speed PID parameters ---- */
#if POSITION_Speed_PID_TYPE == 0
p_i_d_Value_TypeDef PID_Value_Position_speed[4] = {
    { 0, 0, 0.0 },
    { 0, 0, 0.0 },
    { 0, 0, 0.0 },
    { 0, 0, 0.0 }
};
#else
p_i_d_Value_TypeDef PID_Value_Position_speed[4] = {
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 }
};
#endif

/*
 * Speed-loop PID parameters (positional, SPEED_PID_TYPE == 0).
 *
 * Scope: MSPM0G3519 2-wheel, speed range 0~700 (encoder counts/period)
 * PWM limit: 3600 (PWM period 4000)
 * Control period: 128 ms (defined by motor_control_update call rate)
 *
 * Tuning history (target speed 100):
 *   Initial: Kp=1.3, Ki=0.255, Kd=0.1 -> steady-state 46~104, persistent oscillation
 *   Final:   Kp=0.85, Ki=0.25, Kd=0.60 -> steady-state 99~101, stable in 7 periods
 *
 * [0] M0 (left):  Kp=1.20, Ki=0.50, Kd=0.40
 * [1] M1 (right): Kp=1.15, Ki=0.50, Kd=0.40
 * [2] reserved
 * [3] reserved
 */
#if SPEED_PID_TYPE == 0
p_i_d_Value_TypeDef PID_Value_Speed[4] = {
    { 1.20, 0.50, 0.40 },     /* [0] M0 (left) */
    { 1.15, 0.50, 0.40 },     /* [1] M1 (right) */
    { 1.18888, 0.2555, 0.1 }, /* [2] reserved */
    { 1.18888, 0.2555, 0.1 }  /* [3] reserved */
};
#else
p_i_d_Value_TypeDef PID_Value_Speed[4] = {
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 3.0 }
};
#endif

/*
 * Pid_Init() — Initialize a PID instance with parameter values.
 *
 * Clears all state (SetPoint, SumError, Error, LastError, PrevError)
 * and loads Kp/Ki/Kd from the parameter struct.
 *
 * Call once per PID instance at startup, or when switching control modes.
 */
void Pid_Init(PID_TypeDef* PID, p_i_d_Value_TypeDef* pid_Value)
{
    PID->SetPoint    = 0.0;
    PID->ActualValue = 0.0;
    PID->SumError    = 0.0;
    PID->Error       = 0.0;
    PID->LastError   = 0.0;
    PID->PrevError   = 0.0;
    PID->Proportion  = pid_Value->KP;
    PID->Integral    = pid_Value->KI;
    PID->Derivative  = pid_Value->KD;
}

/*
 * Pid_control() — Core PID algorithm.
 *
 * Parameters:
 *   PID            — PID instance pointer
 *   Feedback_value — measured/actual value (float)
 *   Mode           — 0 = positional, 1 = incremental
 *
 * Mode 0 (positional):
 *   Error = SetPoint - Feedback_value
 *   SumError += Error (clamped to +/-4000)
 *   Output = Kp*Error + Ki*SumError + Kd*(Error - LastError)
 *
 * Mode 1 (incremental):
 *   Output += Kp*(Error - LastError) + Ki*Error + Kd*(Error - 2*LastError + PrevError)
 *
 * Integral limit (+/-4000):
 *   At Ki=0.50, I-term max contribution = 0.50*4000 = 2000 PWM
 *   High-speed (>500) needs larger SumError for sufficient PWM thrust
 *   If Ki is changed, verify: I_max = Ki * 4000 >= PWM_MAX
 *
 * Returns: raw output value (no limiting applied — use Pid_OutLimit afterwards)
 */
int32_t Pid_control(PID_TypeDef* PID, float Feedback_value, unsigned char Mode)
{
    PID->Error = PID->SetPoint - Feedback_value;

    if (Mode == 0)
    {
        /* Positional PID — speed loop uses this mode */
        PID->SumError += PID->Error;
        if (PID->SumError > 4000)
            PID->SumError = 4000;
        else if (PID->SumError < -4000)
            PID->SumError = -4000;

        PID->ActualValue = PID->Proportion * PID->Error
                         + PID->Integral * PID->SumError
                         + PID->Derivative * (PID->Error - PID->LastError);

        PID->LastError = PID->Error;
    }
    else
    {
        /* Incremental PID — steering loops use this mode */
        PID->ActualValue += PID->Proportion * (PID->Error - PID->LastError)
                          + PID->Integral * PID->Error
                          + PID->Derivative * (PID->Error - 2 * PID->LastError + PID->PrevError);

        PID->PrevError = PID->LastError;
        PID->LastError = PID->Error;
    }

    return PID->ActualValue;
}

/*
 * Pid_OutLimit() — Clamp PID output to [-limit, +limit].
 *
 * Directly modifies PID->ActualValue and returns the clamped value.
 *
 * Usage:
 *   pwm = Pid_OutLimit(&Speed_Pid[0], MOTOR_PWM_MAX);
 */
int32_t Pid_OutLimit(PID_TypeDef* PID, int32_t limit)
{
    if (PID->ActualValue > limit)
        PID->ActualValue = limit;
    else if (PID->ActualValue < -limit)
        PID->ActualValue = -limit;

    return PID->ActualValue;
}
