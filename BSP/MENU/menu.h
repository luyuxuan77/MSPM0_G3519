#ifndef MENU_H
#define MENU_H
#include "bsp.h"

/* Per-sensor thresholds: index 0..7 = sensor pos 1..8 (L->R) */
extern uint16_t g_gray_threshold[8];

/* Track speed (cm/s) */
extern int g_track_speed_cm_s;

void menu_init(void);
void menu_update(void);

#endif
