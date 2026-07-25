/*------------------------------------------------------------------------------
 * UART1 驱动 — 蓝牙串口
 *
 * 引脚: PA8 (TX), PA9 (RX)   波特率: 115200
 * 功能: 接收中断 + 环形缓冲 + 多字节发送/不定长接收
 *----------------------------------------------------------------------------*/
#include "UART1/uart1.h"

/* ── 环形接收缓冲区 ── */
volatile uint8_t  uart1_rx_buf[UART1_RX_BUF_SIZE];
volatile uint16_t uart1_rx_head = 0;
volatile uint16_t uart1_rx_tail = 0;

/* ================================================================
 *  uart1_init — 使能 NVIC (硬件已在 SYSCFG_DL_init 中初始化)
 * ================================================================ */
void uart1_init(uint32_t baud)
{
    (void)baud;
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
}

/* ================================================================
 *  发送 API
 * ================================================================ */
void uart1_send_byte(uint8_t data)
{
    DL_UART_Main_transmitData(UART_1_INST, data);
    while (DL_UART_Main_isBusy(UART_1_INST));
}

void uart1_send_bytes(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++) {
        DL_UART_Main_transmitData(UART_1_INST, data[i]);
        while (DL_UART_Main_isBusy(UART_1_INST));
    }
}

void uart1_send_string(const char *str)
{
    uart1_send_bytes((const uint8_t *)str, (uint16_t)strlen(str));
}

/* ================================================================
 *  接收 API
 * ================================================================ */
uint16_t uart1_available(void)
{
    return (uint16_t)((uart1_rx_head + UART1_RX_BUF_SIZE - uart1_rx_tail)
                      % UART1_RX_BUF_SIZE);
}

uint8_t uart1_read_byte(void)
{
    /* 阻塞等待 */
    while (uart1_rx_head == uart1_rx_tail);
    uint8_t data = uart1_rx_buf[uart1_rx_tail];
    uart1_rx_tail = (uart1_rx_tail + 1U) % UART1_RX_BUF_SIZE;
    return data;
}

uint16_t uart1_read_bytes(uint8_t *buf, uint16_t max_len)
{
    uint16_t count = 0;
    while (count < max_len && uart1_rx_head != uart1_rx_tail) {
        buf[count++] = uart1_rx_buf[uart1_rx_tail];
        uart1_rx_tail = (uart1_rx_tail + 1U) % UART1_RX_BUF_SIZE;
    }
    return count;
}

void uart1_flush_rx(void)
{
    __disable_irq();
    uart1_rx_tail = uart1_rx_head;
    __enable_irq();
}

/* ================================================================
 *  UART1 接收中断服务函数
 * ================================================================ */
void UART1_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(UART_1_INST) == DL_UART_MAIN_IIDX_RX) {
        uint8_t  data      = DL_UART_receiveData(UART_1_INST);
        uint16_t next_head = (uart1_rx_head + 1U) % UART1_RX_BUF_SIZE;

        /* 缓冲区未满则写入, 满则丢弃 */
        if (next_head != uart1_rx_tail) {
            uart1_rx_buf[uart1_rx_head] = data;
            uart1_rx_head               = next_head;
        }
    }
}
