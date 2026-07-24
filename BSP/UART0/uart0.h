#ifndef UART0_H
#define UART0_H

#include "bsp_common.h"

void uart0_init(uint32_t baud);
void doubleToStr(double value, char *str, int precision);

#endif
