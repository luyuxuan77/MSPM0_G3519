#ifndef KEY_H
#define KEY_H

#include "bsp.h"



#define KEY_USER DL_GPIO_readPins(KEY_PORT, KEY_User_PIN)
void key_init(void);
extern uint8_t key_flag;

void mode_switch();
//extern uint8_t mode;
extern uint8_t run_flag;
extern int route_finish;

extern uint8_t key1_flag;
extern uint8_t key2_flag;

#endif

