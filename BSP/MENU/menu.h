#ifndef MENU_H
#define MENU_H
#include "bsp.h"

#define MENU_TASK_NONE          0
#define MENU_TASK_VIDEO         1
#define MENU_TASK_LINE_LAP      2
#define MENU_TASK_BALL_SWING    3
#define MENU_TASK_A_TO_B        4
#define MENU_TASK_FULL_LAP      5
#define MENU_TASK_SETPOINT_LAP  6
#define MENU_TASK_ANGLE_MON     7
#define MENU_TASK_UART_STATUS   8

#define MENU_RECORD_TASK_ID     1
#define MENU_SETPOINT_STEP_MM    5    /* T6: 5mm step is enough for teacher-set position */
#define MENU_SETPOINT_MIN_MM   -125
#define MENU_SETPOINT_MAX_MM    125

/* Per-sensor thresholds: index 0..7 = sensor pos 1..8 (L->R) */
extern uint16_t g_gray_threshold[8];

/* Track speed (cm/s) */
extern int g_track_speed_cm_s;

extern uint8_t  g_menu_task_id;
extern uint8_t  g_menu_running;
extern uint8_t  g_menu_recording;
extern uint32_t g_menu_task_start_ms;
extern uint32_t g_menu_task_elapsed_ms;
extern int16_t  g_menu_setpoint_mm;
extern uint16_t g_angle_adc_raw;
extern int16_t  g_angle_cdeg;

void menu_init(void);
void menu_update(void);
void menu_stop_task(void);

/* ===== Global timer (0.01s precision, no-flicker) ===== */
void     task_timer_reset(void);
void     task_timer_start(void);
void     task_timer_stop(void);
uint32_t task_timer_get_cs(void);
void     task_timer_draw(uint16_t x, uint16_t y, uint16_t color, uint16_t bg);

#endif
