#ifndef KEY_H
#define KEY_H

#include "bsp.h"

#define KEY_USER DL_GPIO_readPins(KEY_PORT, KEY_User_PIN)
void key_init(void);

extern uint8_t key1_flag;
extern uint8_t key2_flag;

void mode_switch();
extern uint8_t run_flag;
extern int route_finish;

extern uint8_t key1_flag;
extern uint8_t key2_flag;

/*
 * Key event flags (set by ISR, cleared by application after reading)
 *   key1_short = 1 -> Key1 short press  (< 1.5s)
 *   key1_long  = 1 -> Key1 long press   (>= 1.5s)
 *   key2_short = 1 -> Key2 short press
 *   key2_long  = 1 -> Key2 long press
 */
extern volatile uint8_t key1_short;
extern volatile uint8_t key1_long;
extern volatile uint8_t key2_short;
extern volatile uint8_t key2_long;
extern volatile uint8_t k1_held;
extern volatile uint8_t k2_held;

void key_tick_1ms(void);  // called from TIMA1 ISR every 1ms

#endif
