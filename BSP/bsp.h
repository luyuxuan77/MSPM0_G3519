#ifndef BSP_H
#define BSP_H

/******************系统头文件***************/
#include "ti_msp_dl_config.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "math.h"
/*******************************************/
#define u8 unsigned char
#define u32 unsigned int
#define u16  uint16_t
/******************用户自定义头文件*********/
#include "UART0/uart0.h"
#include "SPI0_LCD/lcd.h"
#include "SPI0_LCD/lcd_init.h"
#include "TimerA1/TimerA1.h"
#include "LED/led.h"
#include "SPI1/spi1.h"
#include "lsm6dsv16x_reg.h"
#include "pid.h"
#include "gray.h"
#include "IMU.h"
#include "APP_INIT.h"
#include "INS.h"

#include "Buzzer/buzzer.h"
#include "PWM1_Shared/pwm1_shared.h"


//#include "TimerG6_PWM_RGB/timerG6_pwm_rgb.h"
#include "MOTOR/motor.h"
#include "KEY/key.h"
#include "ADC0/adc0.h"
#include "control.h"
#include "road.h"
#include "MENU/menu.h"
//#include "UART1_FTD/uart1_ftd.h"
#include "SCSLib/SCS.h"
#include "SCSLib/SMS_STS.h"
#include "SCSLib/SCSCL.h"
/*******************************************/
void lsm6dsv16x_sensor_fusion(void);
#endif
