#ifndef MOTOR_H
#define MOTOR_H
#include "bsp.h"
void set_motor_speed(int m1,int m2,int m3,int m4);

void motor_init(void);

void get_motor_qei_cnt(uint32_t *cnt);
void motor_control_update(void);				
void motor_speed_pid_init(void);

void print_date();
void Pid_Speed();


/* ===== Real Speed Calculation (encoder pulses -> cm/s, cm) ===== */
/*
 * Hardware (MG513P30_12V):
 *   gear ratio 1:30, encoder 13 PPR, wheel dia 65mm
 *   QEI 4x: DL_TIMER_QEI_MODE_2_INPUT -> 4 counts per encoder pulse
 *   speed measurement period: 128ms
 */
#define WHEEL_DIAMETER_CM           6.5f
#define ENCODER_PPR                 13.0f
#define GEAR_RATIO                  30.0f
#define QEI_RESOLUTION              4       // 4x quadrature counting
#define SPEED_PERIOD_SEC            0.128f
#define WHEEL_CIRCUMFERENCE_CM       (3.1415926f * WHEEL_DIAMETER_CM)
#define ENCODER_PULSES_PER_WHEEL_REV (ENCODER_PPR * GEAR_RATIO * QEI_RESOLUTION)

/* Get real speed (cm/s). motor_id: 0=left, 1=right */
float get_motor_speed_cm_s(int motor_id);

/* Get distance this period (cm). motor_id: 0=left, 1=right */
float get_motor_distance_cm(int motor_id);

/* Convert cm/s to encoder counts/128ms for PID SetPoint.
   Formula: counts = cm_s * 1560 * 0.128 / 20.42 ~ cm_s * 9.78 */
float cm_s_to_speed_counts(float cm_s);

#endif
