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
    // ADC interrupt handler
    switch (DL_ADC12_getPendingInterrupt(ADC_gray_INST))
    {

        case DL_ADC12_IIDX_MEM0_RESULT_LOADED:

            gCheckADC = true;// flag = 1
            break;
        default:
            break;
    }
}


uint16_t gray_now_val[8]={0};
uint8_t offset_s=0;

// 传感器物理排列(左→右): [7] [3] [5] [1] | [6] [2] [4] [0]
// 权重: 外侧大, 内侧小 (非线性), 中间微调, 两侧激进修正
static const int8_t gray_weight[8] = {
    -14,   // idx 0: 传感器[7] 最左侧
     -9,   // idx 1: 传感器[3] 左中外
     -3,   // idx 2: 传感器[5] 左中内
     -1,   // idx 3: 传感器[1] 紧邻中线左侧, 微调
      1,   // idx 4: 传感器[6] 紧邻中线右侧, 微调
      3,   // idx 5: 传感器[2] 右中内
      9,   // idx 6: 传感器[4] 右中外
     14    // idx 7: 传感器[0] 最右侧
};

int get_gray_refresh_data(void)
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

	// === offset calc: continuous darkness centroid ===
	// darkness = how far below threshold, centroid gives smooth position
	// center=0, edges=large, continuous interpolation between sensors
	#define GOFFSET_GAIN 3   // tune this for steering aggressiveness
	offset_s = 0;
	int total_darkness = 0;
	int weighted_sum = 0;

	// physical order (left → right): [7] [3] [5] [1] | [6] [2] [4] [0]
	if (gray_now_val[7] < g_gray_threshold[0]) { int d = g_gray_threshold[0] - gray_now_val[7]; total_darkness += d; weighted_sum += gray_weight[0] * d; offset_s |= 1 << 0; }
	if (gray_now_val[3] < g_gray_threshold[1]) { int d = g_gray_threshold[1] - gray_now_val[3]; total_darkness += d; weighted_sum += gray_weight[1] * d; offset_s |= 1 << 1; }
	if (gray_now_val[5] < g_gray_threshold[2]) { int d = g_gray_threshold[2] - gray_now_val[5]; total_darkness += d; weighted_sum += gray_weight[2] * d; offset_s |= 1 << 2; }
	if (gray_now_val[1] < g_gray_threshold[3]) { int d = g_gray_threshold[3] - gray_now_val[1]; total_darkness += d; weighted_sum += gray_weight[3] * d; offset_s |= 1 << 3; }
	if (gray_now_val[6] < g_gray_threshold[4]) { int d = g_gray_threshold[4] - gray_now_val[6]; total_darkness += d; weighted_sum += gray_weight[4] * d; offset_s |= 1 << 4; }
	if (gray_now_val[2] < g_gray_threshold[5]) { int d = g_gray_threshold[5] - gray_now_val[2]; total_darkness += d; weighted_sum += gray_weight[5] * d; offset_s |= 1 << 5; }
	if (gray_now_val[4] < g_gray_threshold[6]) { int d = g_gray_threshold[6] - gray_now_val[4]; total_darkness += d; weighted_sum += gray_weight[6] * d; offset_s |= 1 << 6; }
	if (gray_now_val[0] < g_gray_threshold[7]) { int d = g_gray_threshold[7] - gray_now_val[0]; total_darkness += d; weighted_sum += gray_weight[7] * d; offset_s |= 1 << 7; }

	int goffset = (total_darkness > 0) ? (weighted_sum * GOFFSET_GAIN / total_darkness) : 0;
	if (g_show_gray_display) {
	// LCD: squares(y=8) + values(y=20), physical order [0][4][2][6]|[1][5][3][7]
	// threshold=1000: below -> RED(detected), above -> BLACK
	LCD_Fill(0,   8, 23, 17, (gray_now_val[7] < g_gray_threshold[0]) ? RED : BLACK);
	LCD_Fill(36,  8, 59, 17, (gray_now_val[3] < g_gray_threshold[1]) ? RED : BLACK);
	LCD_Fill(72,  8, 95, 17, (gray_now_val[5] < g_gray_threshold[2]) ? RED : BLACK);
	LCD_Fill(108, 8, 131,17, (gray_now_val[1] < g_gray_threshold[3]) ? RED : BLACK);
	LCD_Fill(144, 8, 167,17, (gray_now_val[6] < g_gray_threshold[4]) ? RED : BLACK);
	LCD_Fill(180, 8, 203,17, (gray_now_val[2] < g_gray_threshold[5]) ? RED : BLACK);
	LCD_Fill(216, 8, 239,17, (gray_now_val[4] < g_gray_threshold[6]) ? RED : BLACK);
	LCD_Fill(252, 8, 275,17, (gray_now_val[0] < g_gray_threshold[7]) ? RED : BLACK);

	LCD_ShowIntNum(0,  20, gray_now_val[7], 4, BLACK, WHITE, 12);
	LCD_ShowIntNum(36, 20, gray_now_val[3], 4, BLACK, WHITE, 12);
	LCD_ShowIntNum(72, 20, gray_now_val[5], 4, BLACK, WHITE, 12);
	LCD_ShowIntNum(108,20, gray_now_val[1], 4, BLACK, WHITE, 12);
	LCD_ShowIntNum(144,20, gray_now_val[6], 4, BLACK, WHITE, 12);
	LCD_ShowIntNum(180,20, gray_now_val[2], 4, BLACK, WHITE, 12);
	LCD_ShowIntNum(216,20, gray_now_val[4], 4, BLACK, WHITE, 12);
	LCD_ShowIntNum(252,20, gray_now_val[0], 4, BLACK, WHITE, 12);
	} else {
	LCD_Fill(0, 8, 275, 32, WHITE);
	}


	return goffset;
//	printf("%d\t %d\t %d\t %d\t %d\t %d\t %d\t %d\t \r\n",
//	gray_now_val[0],gray_now_val[4],gray_now_val[2],gray_now_val[6],gray_now_val[1],gray_now_val[5],gray_now_val[3],gray_now_val[7]);
}

#define grat_black   1300
/* === merged into get_gray_refresh_data ===
////int get_gray_offset(void)
////{
////    int goffset=0;
////    offset_s=0;
////    get_gray_refresh_data();
////    if (gray_now_val[0]<1000) 
////        goffset+=7,offset_s|=1<<7;
////    if (gray_now_val[4]<1000) 
////        goffset+=5,offset_s|=1<<6;
////    if (gray_now_val[2]<1000) 
////        goffset+=3,offset_s|=1<<5;
////    if (gray_now_val[6]<1000) 
////        goffset+=1,offset_s|=1<<4;
////
////    if (gray_now_val[1]<1000) 
////        goffset-=1,offset_s|=1<<3;
////    if (gray_now_val[5]<1000) 
////        goffset-=3,offset_s|=1<<2;
////    if (gray_now_val[3]<1000) 
////        goffset-=5,offset_s|=1<<1;
////     if (gray_now_val[7]<1000) 
////         goffset-=7,offset_s|=1<<0;
//////    printf("goffset=%d\r\n",offset_s);
////	 // LCD show offset_s
//////    LCD_ShowIntNum(100,20,offset_s,5,BLACK,WHITE,16);
////    return goffset;
////}
//*/
//
//*/

uint8_t get_offset_s(void)

{
    return offset_s;
}
