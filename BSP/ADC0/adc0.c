#include "ADC0/adc0.h"


//volatile bool gCheckADC;


void adc0_init(void)
{

	NVIC_EnableIRQ(ADC_gray_INST_INT_IRQN);
//	gCheckADC  = false;
}

uint16_t adc0_read_angle_raw(void)
{
    static uint16_t last_raw = ANGLE_ADC_ZERO_RAW;
    uint32_t timeout = 20000;

    DL_ADC12_enableConversions(ADC_gray_INST);
    DL_ADC12_startConversion(ADC_gray_INST);
    while ((DL_ADC12_getStatus(ADC_gray_INST) & DL_ADC12_STATUS_CONVERSION_ACTIVE) && timeout--) {}

    if (timeout != 0) {
        last_raw = (uint16_t)DL_ADC12_getMemResult(ADC_gray_INST, ADC_gray_ADCMEM_0);
    }
    return last_raw;
}

int16_t angle_adc_to_cdeg(uint16_t raw)
{
    int32_t cdeg = ((int32_t)raw - ANGLE_ADC_ZERO_RAW) * ANGLE_ADC_CDEG_PER_COUNT;
    if (cdeg > 32767) cdeg = 32767;
    if (cdeg < -32768) cdeg = -32768;
    return (int16_t)cdeg;
}

//void get_adc0_num_val(uint32_t *adc0)
//{

//	DL_ADC12_startConversion(ADC_gray_INST);
//    /* Wait until all data channels have been loaded. */
//    while (gCheckADC == false) {
//        }
//	
//    *adc0 =
//            DL_ADC12_getMemResult(ADC_gray_INST, DL_ADC12_MEM_IDX_0);
//    *(adc0+1) =
//            DL_ADC12_getMemResult(ADC_gray_INST, DL_ADC12_MEM_IDX_1); 
//    *(adc0+2) =
//            DL_ADC12_getMemResult(ADC_gray_INST, DL_ADC12_MEM_IDX_2);
//    *(adc0+3) =
//            DL_ADC12_getMemResult(ADC_gray_INST, DL_ADC12_MEM_IDX_3);
////    *(adc0+4) =
////            DL_ADC12_getMemResult(ADC_gray_INST, DL_ADC12_MEM_IDX_4);
////    *(adc0+5) =
////            DL_ADC12_getMemResult(ADC_gray_INST, DL_ADC12_MEM_IDX_5);
//    gCheckADC = false;
//    DL_ADC12_enableConversions(ADC_gray_INST);
//}




///* Check for the last result to be loaded then change boolean */
//void ADC_gray_INST_IRQHandler(void)
//{
//    switch (DL_ADC12_getPendingInterrupt(ADC_gray_INST)) {
//        case DL_ADC12_IIDX_MEM3_RESULT_LOADED:
//            gCheckADC = true;
//            break;
//        default:
//            break;
//    }
//}



