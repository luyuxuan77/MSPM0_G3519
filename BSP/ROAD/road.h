#ifndef __ROAD_H
#define __ROAD_H 

//////////////////////////////////////////////////////////////////////////////////	   
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//串口1初始化 
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/5/2
//版本：V1.1
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved
//********************************************************************************
//修改说明
//V1.1 20150411
//修改OS_CRITICAL_METHOD宏判断为：SYSTEM_SUPPORT_OS


void around();
void route1(float speed);
void test(float speed);
extern int target_round;

#endif	   

