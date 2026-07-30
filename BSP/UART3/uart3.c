#include "UART3/uart3.h"

/**************************************************************************
UART3
115200 8-N-1
PB12(TX) PB13(RX)
**************************************************************************/

uint8_t uart3_rec_data[30] = {0};
uint8_t uart3_count = 0;

/**************************************************************************
UART3
**************************************************************************/
void uart3_init(uint32_t baud)
{
    /* Enable RX interrupt — SysConfig does not configure this for UART3 */
    DL_UART_Main_enableInterrupt(yuntai_UART3_INST, DL_UART_MAIN_INTERRUPT_RX);

    DL_UART_Main_enable(yuntai_UART3_INST);

    NVIC_ClearPendingIRQ(yuntai_UART3_INST_INT_IRQN);
    NVIC_EnableIRQ(yuntai_UART3_INST_INT_IRQN);
}

/**************************************************************************
**************************************************************************/
void uart3_send_byte(uint8_t data)
{
    DL_UART_transmitData(yuntai_UART3_INST, data);
    while (DL_UART_Main_isBusy(yuntai_UART3_INST));
}

/**************************************************************************
usart_SendCmd
**************************************************************************/
void usart_SendCmd(uint8_t *cmd, uint8_t len)
{
    uint8_t i = 0;
    for (i = 0; i < len; i++)
    {
        DL_UART_transmitData(yuntai_UART3_INST, cmd[i]);
        while (DL_UART_Main_isBusy(yuntai_UART3_INST));
    }
}

/**************************************************************************
UART3
**************************************************************************/
void UART3_IRQHandler(void)
{
    uint8_t res;
    if (DL_UART_Main_getPendingInterrupt(yuntai_UART3_INST) == DL_UART_MAIN_IIDX_RX)
    {
        res = DL_UART_receiveData(yuntai_UART3_INST);
        if (uart3_count < 30) {
            uart3_rec_data[uart3_count++] = res;
        }
        /* drop byte silently when full — caller must clear uart3_count before each query */
    }
}
