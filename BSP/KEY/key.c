#include "KEY/key.h"
#include "led.h"
void key_init(void)
{
	NVIC_EnableIRQ(KEY_GPIOA_INT_IRQN); // 开启中断
	NVIC_EnableIRQ(KEY_GPIOC_INT_IRQN); // 开启中断
	LED1(1) ;
	LED2(1) ;
}

uint8_t key1_flag=0;
uint8_t key2_flag=0;
uint8_t mode=0;
uint8_t run_flag = 0;
volatile uint32_t key1_time = 0;
volatile uint32_t key2_time = 0;

//模式1：key1是按下的次数，k2短按一下是确定，长按5秒是切换模式.目前有模式1，模式2

void GROUP1_IRQHandler(void)
{
	 uint32_t now = nowtime;
	
	/*收集可以产生触发中断的引脚*/
	uint32_t IRQn_key = DL_GPIO_getEnabledInterruptStatus(KEY_Key1_PORT, KEY_Key1_PIN);
	/*根据不同引脚执行对应中断代码*/
	if (IRQn_key & KEY_Key1_PIN) 
	{	
		DL_GPIO_clearInterruptStatus(KEY_Key1_PORT,KEY_Key1_PIN);
		if(now-key1_time>50)
        {
            LED1_toggle;
            key1_flag++;
            key1_time=now;
        }
	}
	uint32_t status = DL_GPIO_getEnabledInterruptStatus(KEY_Key2_PORT, KEY_Key2_PIN);
	
	if(status & KEY_Key2_PIN)
    {
		DL_GPIO_clearInterruptStatus(KEY_Key2_PORT,KEY_Key2_PIN);
		 if(now-key2_time>50)
        {
            LED2_toggle;
            key2_flag++;
			run_flag = 1; 
            key2_time=now;
        }
    }
}

void mode_switch()
{
	switch(mode)
	{
		case 0:		
			if (key2_flag)
			{
				target_round=key1_flag;
				if(target_round>5) target_round=5;
				key2_flag=0;
			}
			break;
//		case 1:
			
	}
}

