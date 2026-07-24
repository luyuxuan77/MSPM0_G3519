#ifndef LED_H
#define LED_H

#include "bsp_common.h"

/* 控制 LED1（PA14） */
#define LED1(en)                                      \
	do {                                              \
		if (en)                                       \
			DL_GPIO_setPins(LED_PORT, LED_L1_PIN);     \
		else                                          \
			DL_GPIO_clearPins(LED_PORT, LED_L1_PIN);   \
	} while (0)

#define LED1_toggle DL_GPIO_togglePins(LED_PORT, LED_L1_PIN)

/* 控制 LED2（PA17） */
#define LED2(en)                                      \
	do {                                              \
		if (en)                                       \
			DL_GPIO_setPins(LED_PORT, LED_L2_PIN);     \
		else                                          \
			DL_GPIO_clearPins(LED_PORT, LED_L2_PIN);   \
	} while (0)

#define LED2_toggle DL_GPIO_togglePins(LED_PORT, LED_L2_PIN)

void LED_init(void);

#endif
