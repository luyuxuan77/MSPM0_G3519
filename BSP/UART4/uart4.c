/*------------------------------------------------------------------------------
 * UART4 驱动 — 步进驱动串口
 *
 * 引脚: PB10 (TX), PB18 (RX)   波特率: 115200
 * 功能: 接收中断 + 环形缓冲 + 多字节发送/不定长接收
 *
 * 注意: SysConfig 未为 UART4 开启 RX 中断, 本驱动在 uart4_init() 中使能。
 *----------------------------------------------------------------------------*/
#include "UART4/uart4.h"

/* ── 环形接收缓冲区 ── */
volatile uint8_t  uart4_rx_buf[UART4_RX_BUF_SIZE];
volatile uint16_t uart4_rx_head = 0;
volatile uint16_t uart4_rx_tail = 0;

/* ================================================================
 *  uart4_init — 使能 RX 中断 + NVIC
 * ================================================================ */
void uart4_init(uint32_t baud)
{
    (void)baud;

    /* SysConfig 未使能 UART4 的 RX 中断, 在此补上 */
    DL_UART_Main_enableInterrupt(UART_4_INST, DL_UART_MAIN_INTERRUPT_RX);

    NVIC_ClearPendingIRQ(UART_4_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_4_INST_INT_IRQN);
}

/* ================================================================
 *  发送 API
 * ================================================================ */
void uart4_send_byte(uint8_t data)
{
    DL_UART_Main_transmitData(UART_4_INST, data);
    while (DL_UART_Main_isBusy(UART_4_INST));
}

void uart4_send_bytes(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++) {
        DL_UART_Main_transmitData(UART_4_INST, data[i]);
        while (DL_UART_Main_isBusy(UART_4_INST));
    }
}

void uart4_send_string(const char *str)
{
    uart4_send_bytes((const uint8_t *)str, (uint16_t)strlen(str));
}

/* ================================================================
 *  接收 API
 * ================================================================ */
uint16_t uart4_available(void)
{
    return (uint16_t)((uart4_rx_head + UART4_RX_BUF_SIZE - uart4_rx_tail)
                      % UART4_RX_BUF_SIZE);
}

uint8_t uart4_read_byte(void)
{
    while (uart4_rx_head == uart4_rx_tail);
    uint8_t data = uart4_rx_buf[uart4_rx_tail];
    uart4_rx_tail = (uart4_rx_tail + 1U) % UART4_RX_BUF_SIZE;
    return data;
}

uint16_t uart4_read_bytes(uint8_t *buf, uint16_t max_len)
{
    uint16_t count = 0;
    while (count < max_len && uart4_rx_head != uart4_rx_tail) {
        buf[count++] = uart4_rx_buf[uart4_rx_tail];
        uart4_rx_tail = (uart4_rx_tail + 1U) % UART4_RX_BUF_SIZE;
    }
    return count;
}

void uart4_flush_rx(void)
{
    __disable_irq();
    uart4_rx_tail = uart4_rx_head;
    __enable_irq();
}

/* ================================================================
 *  UART4 接收中断服务函数
 * ================================================================ */
void UART4_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(UART_4_INST) == DL_UART_MAIN_IIDX_RX) {
        uint8_t  data      = DL_UART_receiveData(UART_4_INST);
        uint16_t next_head = (uart4_rx_head + 1U) % UART4_RX_BUF_SIZE;

        if (next_head != uart4_rx_tail) {
            uart4_rx_buf[uart4_rx_head] = data;
            uart4_rx_head               = next_head;
        }
    }
}
