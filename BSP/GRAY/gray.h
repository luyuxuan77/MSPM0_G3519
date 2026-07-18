#ifndef _GRAY_H_
#define _GRAY_H_
#include "bsp.h"


/*********************************************gw_gray************** *****************************************/
#define gw_gray_ad0(x) ((x) ? DL_GPIO_setPins(gw_ad_PORT, gw_ad_D0_PIN) : DL_GPIO_clearPins(gw_ad_PORT, gw_ad_D0_PIN))
#define gw_gray_ad1(x) ((x) ? DL_GPIO_setPins(gw_ad_PORT, gw_ad_D1_PIN) : DL_GPIO_clearPins(gw_ad_PORT, gw_ad_D1_PIN))
#define gw_gray_ad2(x) ((x) ? DL_GPIO_setPins(gw_ad_PORT, gw_ad_D2_PIN) : DL_GPIO_clearPins(gw_ad_PORT, gw_ad_D2_PIN)) 

int get_gray_refresh_data(void);  // 采集8路灰度 + 计算偏移量 + LCD显示 (merged)
// int get_gray_offset(void);      // 已合并到 get_gray_refresh_data

uint8_t get_offset_s(void);

uint16_t get_adc0_value();//�ɼ�VCC�����ѹ����XT30�ӿ������ѹ
/**************************************************************************************************************/



#endif
