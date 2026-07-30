#ifndef UART3_H
#define UART3_H
#include "bsp.h"

extern uint8_t uart3_rec_data[30];
extern uint8_t uart3_count;

void uart3_init(uint32_t baud);
void usart_SendCmd(uint8_t *cmd, uint8_t len);
void uart3_send_byte(uint8_t data);

#endif
