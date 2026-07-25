#ifndef MOTOR_H
#define MOTOR_H

#include "bsp.h"

/* ===========================================================================
 * Motor Control Module — ported from Car_sum, adapted for CORE_TEST V2.0
 * ===========================================================================
 *
 * Hardware (CORE_TEST):
 *   TIMG14 (MOTOR_PWM): 40 MHz, period=4000, edge-aligned PWM
 *     CC0 -> PB28  Motor0 PWM
 *     CC1 -> PB13  Motor1 PWM
 *   TIMG9 (QEI_M0):   PB29(PHA), PB30(PHB) — Motor0 encoder (quadrature 4x)
 *   TIMG8 (QEI_M1):   PC6(PHA),  PA22(PHB)  — Motor1 encoder (quadrature 4x)
 *   Direction GPIO:   PA31 -> Motor0 PH, PB0 -> Motor1 PH
 *
 * Encoder spec (MG513P30_12V or compatible):
 *   PPR=13, gear ratio 1:30, QEI 4x = 1560 counts/wheel_rev
 *   Wheel diameter=65 mm, circumference=20.42 cm
 *
 * Control period: 128 ms (motor_control_update call rate)
 * PWM output period: 10 ms (Pid_Speed call rate)
 * ===========================================================================
 */

/* ---- PWM Constants ---- */
#define MOTOR_PWM_PERIOD    4000    /* TIMG14 period (SysConfig) */
#define MOTOR_PWM_MAX       3600    /* PID output clamp (90% of period) */
#define MOTOR_PWM_STOP      3999    /* CCR value for 0% duty (brake) */

/* ---- PWM Timer Instance (from SysConfig ti_msp_dl_config.h) ---- */
#define MOTOR_PWM_INST      TIMG14_Motor_INST   /* TIMG14 */

/* ---- QEI Timer Instances ---- */
#define QEI_M0_INST         TIMG9   /* Motor0 encoder */
#define QEI_M1_INST         TIMG8   /* Motor1 encoder */

/* ---- Motor Direction Macros ---- */
#define M0_PH(en)  (en) ? DL_GPIO_setPins(Motor_DIR_M0_PH_PORT, Motor_DIR_M0_PH_PIN) \
                        : DL_GPIO_clearPins(Motor_DIR_M0_PH_PORT, Motor_DIR_M0_PH_PIN)
#define M1_PH(en)  (en) ? DL_GPIO_setPins(Motor_DIR_M1_PH_PORT, Motor_DIR_M1_PH_PIN) \
                        : DL_GPIO_clearPins(Motor_DIR_M1_PH_PORT, Motor_DIR_M1_PH_PIN)

/*
 * NOTE: If M1 is physically mounted in the opposite orientation,
 * invert the M1_PH logic by swapping setPins/clearPins above,
 * or negate the pwm1 value before calling set_motor_speed().
 */

/* ---- Physical Parameters (MG513P30_12V) ---- */
#define WHEEL_DIAMETER_CM               6.5f
#define ENCODER_PPR                     13.0f
#define GEAR_RATIO                      30.0f
#define QEI_RESOLUTION                  4       /* 4x quadrature mode */
#define SPEED_PERIOD_SEC                0.128f  /* 128 ms control period */
#define WHEEL_CIRCUMFERENCE_CM          (3.1415926f * WHEEL_DIAMETER_CM)
#define ENCODER_PULSES_PER_WHEEL_REV    (ENCODER_PPR * GEAR_RATIO * QEI_RESOLUTION)

/* ===========================================================================
 * API Functions
 * =========================================================================== */

/* Hardware init — call once at boot (before motor_speed_pid_init) */
void motor_init(void);

/* Set raw motor speed (signed PWM, -4000..+4000).
   Called by Pid_Speed() each 10 ms. */
void set_motor_speed(int m0, int m1);

/* Read encoder counts: cnt[0]=M0, cnt[1]=M1 */
void get_motor_qei_cnt(uint32_t *cnt);

/* PID speed control — call every 128 ms.
   Reads encoders, runs low-pass filter + ramp + PID, updates pwm0/pwm1. */
void motor_control_update(void);

/* Initialize PID state + encoder baseline. Call before enabling motors. */
void motor_speed_pid_init(void);

/* Current PWM output values (read-only for display/debug) */
extern int pwm0;
extern int pwm1;

/* Write pwm0/pwm1 to hardware — call every 10 ms, right after motor_control_update. */
void Pid_Speed(void);

/* ---- Physical Speed / Distance Conversion ---- */

/* Get real speed (cm/s). motor_id: 0=M0(left), 1=M1(right) */
float get_motor_speed_cm_s(int motor_id);

/* Get distance traveled this period (cm). motor_id: 0=M0, 1=M1 */
float get_motor_distance_cm(int motor_id);

/* Convert cm/s to encoder counts per 128 ms period (for PID SetPoint).
   Formula: counts = cm_s * 1560 * 0.128 / 20.42 ≈ cm_s * 9.78 */
float cm_s_to_speed_counts(float cm_s);

#endif /* MOTOR_H */
