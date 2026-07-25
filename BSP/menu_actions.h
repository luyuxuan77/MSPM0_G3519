#ifndef MENU_ACTIONS_H
#define MENU_ACTIONS_H

#include "bsp.h"

/* ===========================================================================
 * Menu Action Functions — one per leaf menu item.
 *
 * Each function blocks until the user presses '*' (or the action completes),
 * then returns to the menu system.
 * ===========================================================================
 */

/* ---- RGB Cycle ---- */
void act_rgb_cycle(void);

/* ---- Buzzer Test ---- */
void act_buzzer(void);

/* ---- Motor Ctrl (4 sub-items) ---- */
void act_m0fwd(void);
void act_m0rev(void);
void act_m1fwd(void);
void act_m1rev(void);

/* ---- ADC Monitor ---- */
void act_adc_monitor(void);

/* ---- Keypad Test ---- */
void act_keypad_test(void);

/* ---- System Info ---- */
void act_sys_info(void);

/* ---- OLED Demo ---- */
void act_oled_demo(void);

/* ---- UART Monitor ---- */
void act_uart1_monitor(void);
void act_uart4_monitor(void);

#endif /* MENU_ACTIONS_H */
