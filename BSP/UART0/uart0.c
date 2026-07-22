#include "UART0/uart0.h"

/**************************************************************************
函数功能：重定义fputc/fputs函数
函数说明：用于重定义fputc/fputs函数，由于fputc/fputs函数仅有一个，所以无法
实现多个串口均用printf打印，所以本工程仅USART0使用printf，若有需要自行更改。
入口参数：略
返回  值：略
其    他：如果发现printf可以输出一般字符，而无法输出参数，
需要重定向fputs的同时，再重定向puts
**************************************************************************/
#pragma(__use_no_semihosting)
struct FILE
{
	int handle;
};
FILE __stdout;
void _sys_exit(int x)
{
	x = x;
}
int fputc(int ch, FILE *f)
{
	
	/* 发送一个数据 */
	DL_UART_Main_transmitData(UART0, (uint8_t)ch);
	/* 等待数据传输完毕 */
	while (DL_UART_Main_isBusy(UART0))
		;

	return ch;
}

int fputs(const char *_ptr, register FILE *_fp)
{
	uint16_t i, len;
	len = strlen(_ptr);
	for (i = 0; i < len; i++)
	{
		/* 发送一个数据 */
		DL_UART_Main_transmitData(UART0, (uint8_t)_ptr[i]);
		/* 等待数据传输完毕 */
		while (DL_UART_Main_isBusy(UART0))
			;
	}

	return len;
}

void delay_ms(uint32_t ms)
{
    delay_cycles(80000 * ms);
}
void delay_us(uint32_t us)
{
    delay_cycles(80 * us);
}

///**************************************************************************
//函数功能：UART0初始化函数
//函数说明：
//        配置波特率的参数计算公式
//        BRD = UART Clock / （OVS * Baudrate） = integerDivisor.X
//        fractionalDivisor = （X*64）+0.5
//入口参数：选择波特率，目前只有9600
//返回  值：无
//**************************************************************************/
//static const DL_UART_Main_ClockConfig gUART_0ClockConfig = {
//    .clockSel = DL_UART_MAIN_CLOCK_BUSCLK,
//    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1};

//static const DL_UART_Main_Config gUART_0Config = {
//    .mode = DL_UART_MAIN_MODE_NORMAL,
//    .direction = DL_UART_MAIN_DIRECTION_TX_RX,
//    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
//    .parity = DL_UART_MAIN_PARITY_NONE,
//    .wordLength = DL_UART_MAIN_WORD_LENGTH_8_BITS,
//    .stopBits = DL_UART_MAIN_STOP_BITS_ONE};
void uart0_init(uint32_t baud)
{
    /*
     * Reconfigure baud rate (SysConfig sets 115200 by default).
     * Clock = BUSCLK 40MHz, 16x oversampling.
     * Divisor = 40,000,000 / (16 * baud)
     */
    if (baud != 115200) {
        DL_UART_Main_setOversampling(UART_0_INST, DL_UART_OVERSAMPLING_RATE_16X);
        uint32_t divisor = 40000000U / (16U * baud);
        uint32_t remainder = 40000000U % (16U * baud);
        uint32_t frac = (remainder * 64U + (8U * baud)) / (16U * baud);
        DL_UART_Main_setBaudRateDivisor(UART_0_INST, divisor, frac);
    }

    DL_UART_Main_enable(UART_0_INST);

    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

/**************************************************************************
函数功能：UART0中断服务函数
**************************************************************************/
uint8_t g_usart0_rx_buf[20];
uint8_t gEchoData = 0;
void UART0_IRQHandler(void)
{
    uint8_t res;
    printf("ok\r\n");
    if (DL_UART_Main_getPendingInterrupt(UART0) == DL_UART_MAIN_IIDX_RX) // 接收到数据
    {
        res = DL_UART_receiveData(UART0); // 将接收到的数据赋给变量res
    }
}

/**************************************************************************
函数功能：浮点转字符串
函数说明：无
入口参数：value ->浮点数  *str ->字符串  precision ->保留有效数字位数
返回  值：无
**************************************************************************/
void doubleToStr(double value, char *str, int precision)
{
    long wholePart = (long)value;
    double fractionalPart = fmod(value, 1.0);
    /*使用sprintf函数将整数部分和一个小数点写入字符串*/
    sprintf(str, "%ld.", wholePart);
    str += strlen(str);
    for (int i = 0; i < precision; ++i)
    {
        fractionalPart *= 10;
        long digit = (long)fractionalPart;
        sprintf(str, "%ld", digit);
        str += strlen(str);
        fractionalPart -= digit;
    }
    *str = '\0';
}
