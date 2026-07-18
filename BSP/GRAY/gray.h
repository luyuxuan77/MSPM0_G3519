#ifndef _GRAY_H_
#define _GRAY_H_
#include "bsp.h"


/*********************************************gw_gray************** *****************************************/
#define gw_gray_ad0(x) ((x) ? DL_GPIO_setPins(gw_ad_PORT, gw_ad_D0_PIN) : DL_GPIO_clearPins(gw_ad_PORT, gw_ad_D0_PIN))
#define gw_gray_ad1(x) ((x) ? DL_GPIO_setPins(gw_ad_PORT, gw_ad_D1_PIN) : DL_GPIO_clearPins(gw_ad_PORT, gw_ad_D1_PIN))
#define gw_gray_ad2(x) ((x) ? DL_GPIO_setPins(gw_ad_PORT, gw_ad_D2_PIN) : DL_GPIO_clearPins(gw_ad_PORT, gw_ad_D2_PIN)) 

void get_gray_refresh_data(void);
int get_gray_offset(void);

uint8_t get_offset_s(void);

uint16_t get_adc0_value();//采集VCC输入电压，即XT30接口输入电压
/**************************************************************************************************************/



#endif
