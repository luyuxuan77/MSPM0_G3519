#ifndef MENU_H
#define MENU_H
#include "bsp.h"

/* Per-sensor thresholds: index 0..7 = sensor pos 1..8 (L->R) */
extern uint16_t g_gray_threshold[8];

/* Track speed (cm/s) */
extern int g_track_speed_cm_s;

/* ===== Menu states ===== */
#define STATE_MAIN              0
#define STATE_THRESHOLD_SELECT  1
#define STATE_THRESHOLD_ADJUST  2
#define STATE_TASK1             3
#define STATE_TASK2             4
#define STATE_TASK3             5
#define STATE_TASK4             6
#define STATE_THRESHOLD_DISPLAY 7

extern uint8_t g_show_gray_display;

extern uint8_t g_task1_active;
extern uint8_t g_task1_running;
extern uint8_t run_flag;

void menu_init(void);
void menu_update(void);

#endif
