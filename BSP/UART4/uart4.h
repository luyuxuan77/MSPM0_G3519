#ifndef UART4_H
#define UART4_H

#include "bsp_common.h"

#define UART4_RX_BUF_SIZE  512

/* ── 环形接收缓冲区 (ISR 写入, 应用读取) ── */
extern volatile uint8_t  uart4_rx_buf[UART4_RX_BUF_SIZE];
extern volatile uint16_t uart4_rx_head;
extern volatile uint16_t uart4_rx_tail;

/* ── API ── */
void     uart4_init(uint32_t baud);
void     uart4_send_byte(uint8_t data);
void     uart4_send_bytes(const uint8_t *data, uint16_t len);
void     uart4_send_string(const char *str);
uint16_t uart4_available(void);
uint8_t  uart4_read_byte(void);
uint16_t uart4_read_bytes(uint8_t *buf, uint16_t max_len);
void     uart4_flush_rx(void);

#endif
