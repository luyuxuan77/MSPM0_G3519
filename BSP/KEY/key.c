#include "KEY/key.h"
#include "KEY/keypad.h"
#include "LED/led.h"

void key_init(void)
{
	NVIC_EnableIRQ(key_INT_IRQN);
}

volatile uint8_t key_flag = 0;

void GROUP1_IRQHandler(void)
{
	uint32_t irqn_key = DL_GPIO_getEnabledInterruptStatus(key_PORT, key_user_PIN);

	if ((irqn_key & key_user_PIN) == key_user_PIN) {
		DL_GPIO_clearInterruptStatus(key_PORT, key_user_PIN);
		key_flag = !key_flag;
		LED2_toggle;
	}

	/* 矩阵键盘行中断 (GPIOC PC0~PC3) */
	keypad_isr_handler();
}
