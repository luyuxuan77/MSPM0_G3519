#ifndef SPI0_OLED_H
#define SPI0_OLED_H

#include "bsp_common.h"

#define OLED_CMD  0
#define OLED_DATA 1

#define OLED_CS_Clr()  DL_GPIO_clearPins(OLED_CS_PORT, OLED_CS_PIN)
#define OLED_CS_Set()  DL_GPIO_setPins(OLED_CS_PORT, OLED_CS_PIN)
#define OLED_RST_Clr() DL_GPIO_clearPins(OLED_RES_PORT, OLED_RES_PIN)
#define OLED_RST_Set() DL_GPIO_setPins(OLED_RES_PORT, OLED_RES_PIN)
#define OLED_DC_Clr()  DL_GPIO_clearPins(OLED_DC_PORT, OLED_DC_PIN)
#define OLED_DC_Set()  DL_GPIO_setPins(OLED_DC_PORT, OLED_DC_PIN)

#define Max_Column 128
#define Max_Row    64
#define X_WIDTH    128
#define Y_WIDTH    64
#define SIZE       16

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(u8 x, u8 y, u8 chr);
void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len, u8 size2);
void OLED_ShowString(u8 x, u8 y, u8 *p);
void OLED_ShowString_Small(u8 x, u8 y, u8 *p);
void OLED_Set_Pos(unsigned char x, unsigned char y);

#endif
