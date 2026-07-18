#ifndef UART1_FTD_H
#define UART1_FTD_H
#include "bsp.h"

void ftUart_Send(uint8_t *nDat , int nLen);


int ftUart_Read(uint8_t *nDat, int nLen);


void ftBus_Delay(void);


void uart1_init(uint32_t baud);


#endif
