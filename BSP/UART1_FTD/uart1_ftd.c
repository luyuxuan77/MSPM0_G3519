#include "UART1_FTD/uart1_ftd.h"






void send_ftd_u8(uint8_t data)
{
	/* 发送一个数据 */
	DL_UART_transmitData(UART1, (uint8_t)data);
	/* 等待数据传输完毕 */
	while (DL_UART_Main_isBusy(UART1));

}
uint8_t uart1_rec_data[30]={0};
uint8_t uart1_count=0;



//FT舵机串口指令发送函数
void ftUart_Send(uint8_t *nDat , int nLen)
{
	int len=0;
	for(;len<nLen;len++)
	{
		DL_UART_transmitData(UART1, *(nDat+len));
		while (DL_UART_Main_isBusy(UART1));
	}
}


//FT舵机串口指令应答接收函数
int ftUart_Read(uint8_t *nDat, int nLen)
{
	int len=0;
	uart1_count=0;
	printf("len=%d", nLen) ;
//	while(uart1_count!=nLen-1);
//	
//	for(;len<nLen;len++)
//	{
//		*(nDat+len)=uart1_rec_data[len];
//	}
	return 0;
}



void UART1_IRQHandler(void)
{
    uint8_t res;
    if (DL_UART_Main_getPendingInterrupt(UART1) == DL_UART_MAIN_IIDX_RX) // 接收到数据
    {
        res = DL_UART_receiveData(UART1); // 将接收到的数据赋给变量res
		if(uart1_count>29) uart1_count=0;
		uart1_rec_data[uart1_count++]=res;
    }
}

void uart1_init(uint32_t baud)
{
    DL_UART_Main_enable(UART_1_FTD_INST);

    NVIC_ClearPendingIRQ(UART_1_FTD_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_FTD_INST_INT_IRQN);
}

//FT舵机总线切换延时，时间大于10us
void ftBus_Delay(void)
{
	delay_ms(1);
}
