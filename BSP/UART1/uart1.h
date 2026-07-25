#ifndef UART1_H
#define UART1_H

#include "bsp_common.h"

#define UART1_RX_BUF_SIZE  512

/* ── 环形接收缓冲区 (ISR 写入, 应用读取) ── */
extern volatile uint8_t  uart1_rx_buf[UART1_RX_BUF_SIZE];
extern volatile uint16_t uart1_rx_head;
extern volatile uint16_t uart1_rx_tail;

/* ── API ── */
void     uart1_init(uint32_t baud);
void     uart1_send_byte(uint8_t data);
void     uart1_send_bytes(const uint8_t *data, uint16_t len);
void     uart1_send_string(const char *str);
uint16_t uart1_available(void);
uint8_t  uart1_read_byte(void);
uint16_t uart1_read_bytes(uint8_t *buf, uint16_t max_len);
void     uart1_flush_rx(void);

#endif
