#include "bsp.h"

/*
	AD0   PB26
	AD1   PB23
	AD2   PB20
	OUT   PB25 ADC0 CH4
*/



volatile bool gCheckADC=false;
uint16_t get_adc0_value()
{
//	printf("0");
    uint16_t gAdcResult =0;
	DL_ADC12_setStartAddress(ADC_gray_INST, DL_ADC12_SEQ_START_ADDR_04);
    DL_ADC12_startConversion(ADC_gray_INST);
	
    while (false == gCheckADC) {
           // __WFE();
        }
	
	gAdcResult = DL_ADC12_getMemResult(ADC_gray_INST, DL_ADC12_MEM_IDX_0);
    
    gCheckADC = false;
	DL_ADC12_enableConversions(ADC_gray_INST);	
    return gAdcResult;
}
void ADC_gray_INST_IRQHandler(void)
{
//	printf("1");
    //查询并清除ADC中断
    switch (DL_ADC12_getPendingInterrupt(ADC_gray_INST))
    {

        case DL_ADC12_IIDX_MEM0_RESULT_LOADED:

            gCheckADC = true;//将标志位置1
            break;
        default:
            break;
    }
}


uint16_t gray_now_val[8]={0};
void get_gray_refresh_data(void)
{

    uint8_t num=1; 
    for (; num<9; num++) {
        switch (num) {
        case 1: gw_gray_ad2(0),gw_gray_ad1(0),gw_gray_ad0(0);break;
        case 2: gw_gray_ad2(0),gw_gray_ad1(0),gw_gray_ad0(1);break;
        case 3: gw_gray_ad2(0),gw_gray_ad1(1),gw_gray_ad0(0);break;
        case 4: gw_gray_ad2(0),gw_gray_ad1(1),gw_gray_ad0(1);break;
        case 5: gw_gray_ad2(1),gw_gray_ad1(0),gw_gray_ad0(0);break;
        case 6: gw_gray_ad2(1),gw_gray_ad1(0),gw_gray_ad0(1);break;
        case 7: gw_gray_ad2(1),gw_gray_ad1(1),gw_gray_ad0(0);break;
        case 8: gw_gray_ad2(1),gw_gray_ad1(1),gw_gray_ad0(1);break;
        }
        delay_us(200);
//		printf("1");	
        gray_now_val[num-1]=get_adc0_value();

    }
//    // LCD显示8路灰度值
//    LCD_ShowIntNum(0, 40, gray_now_val[7], 4, BLACK, WHITE, 16);
//    LCD_ShowIntNum(60,40, gray_now_val[3], 4, BLACK, WHITE, 16);
//    LCD_ShowIntNum(120,40,gray_now_val[5], 4, BLACK, WHITE, 16);
//    LCD_ShowIntNum(180,40,gray_now_val[1], 4, BLACK, WHITE, 16);

//    LCD_ShowIntNum(0, 60, gray_now_val[6], 4, BLACK, WHITE, 16);
//    LCD_ShowIntNum(60,60,gray_now_val[2], 4, BLACK, WHITE, 16);
//    LCD_ShowIntNum(120,60,gray_now_val[4], 4, BLACK, WHITE, 16);
//    LCD_ShowIntNum(180,60,gray_now_val[0], 4, BLACK, WHITE, 16);
//	printf("%d\t %d\t %d\t %d\t %d\t %d\t %d\t %d\t \r\n",
//	gray_now_val[0],gray_now_val[4],gray_now_val[2],gray_now_val[6],gray_now_val[1],gray_now_val[5],gray_now_val[3],gray_now_val[7]);
}

#define grat_black   1300
uint8_t offset_s=0;
int get_gray_offset(void)
{
    int goffset=0;
    offset_s=0;
    get_gray_refresh_data();
    if (gray_now_val[0]<1000) 
        goffset+=7,offset_s|=1<<7;
    if (gray_now_val[4]<1000) 
        goffset+=5,offset_s|=1<<6;
    if (gray_now_val[2]<1000) 
        goffset+=3,offset_s|=1<<5;
    if (gray_now_val[6]<1000) 
        goffset+=1,offset_s|=1<<4;

    if (gray_now_val[1]<1000) 
        goffset-=1,offset_s|=1<<3;
    if (gray_now_val[5]<1000) 
        goffset-=3,offset_s|=1<<2;
    if (gray_now_val[3]<1000) 
        goffset-=5,offset_s|=1<<1;
     if (gray_now_val[7]<1000) 
         goffset-=7,offset_s|=1<<0;
//    printf("goffset=%d\r\n",offset_s);
	 // LCD显示offset_s
//    LCD_ShowIntNum(100,20,offset_s,5,BLACK,WHITE,16);
    return goffset;
}

uint8_t get_offset_s(void)

{
    return offset_s;
}
