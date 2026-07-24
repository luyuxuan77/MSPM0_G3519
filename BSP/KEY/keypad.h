#ifndef KEYPAD_H
#define KEYPAD_H

#include "bsp_common.h"

#define KEYPAD_KEY_NONE  0xFF

void keypad_init(void);
void keypad_isr_handler(void);  /* 在 GROUP1_IRQHandler 中调用 */
uint8_t keypad_scan(void);      /* 主循环非阻塞调用 */

#endif
