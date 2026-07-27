#include "ADC/adc.h"

/* ── 全局 ADC 数据: 一次序列全扫后存放 ── */
uint16_t g_adc_raw[8];

/*
 * 一次采样全部 8 路 (ADC0 序列扫 7 路 + ADC1 单通道)
 * 调用后 g_adc_raw[0..7] 即更新为最新值
 */
void adc_sample_all(void)
{
    /* ADC0: 序列模式一次 SC 转换 MEM0→MEM6 */
    DL_ADC12_enableConversions(ADC0);
    DL_ADC12_startConversion(ADC0);
    while (DL_ADC12_getStatus(ADC0) & DL_ADC12_STATUS_CONVERSION_ACTIVE) {}

    g_adc_raw[0] = (uint16_t)DL_ADC12_getMemResult(ADC0, DL_ADC12_MEM_IDX_0);
    g_adc_raw[1] = (uint16_t)DL_ADC12_getMemResult(ADC0, DL_ADC12_MEM_IDX_1);
    g_adc_raw[2] = (uint16_t)DL_ADC12_getMemResult(ADC0, DL_ADC12_MEM_IDX_2);
    g_adc_raw[3] = (uint16_t)DL_ADC12_getMemResult(ADC0, DL_ADC12_MEM_IDX_3);
    g_adc_raw[4] = (uint16_t)DL_ADC12_getMemResult(ADC0, DL_ADC12_MEM_IDX_4);
    g_adc_raw[5] = (uint16_t)DL_ADC12_getMemResult(ADC0, DL_ADC12_MEM_IDX_5);
    g_adc_raw[6] = (uint16_t)DL_ADC12_getMemResult(ADC0, DL_ADC12_MEM_IDX_6);

    /* ADC1: 单通道 */
    DL_ADC12_enableConversions(ADC1);
    DL_ADC12_startConversion(ADC1);
    while (DL_ADC12_getStatus(ADC1) & DL_ADC12_STATUS_CONVERSION_ACTIVE) {}
    g_adc_raw[7] = (uint16_t)DL_ADC12_getMemResult(ADC1, DL_ADC12_MEM_IDX_0);
}

/*
 * ADC0 / ADC1 初始化
 *
 * 配置 (参照 Car_sum PB24 已验证):
 *   - 时钟:   SYSOSC / 8 = 4 MHz, freqRange 24-32M
 *   - 参考:   VDDA (~3.3V)
 *   - 采样:   SCOMP0 = 400
 *   - 模式:   DISABLED + 单次 + 软件触发
 */

/* ============================== ADC0 (7通道) ============================== */

void adc0_init(void)
{
    DL_ADC12_reset(ADC0);
    DL_ADC12_enablePower(ADC0);

    /* 时钟: SYSOSC / 8 = 4 MHz (Car_sum) */
    static const DL_ADC12_ClockConfig clkCfg = {
        .clockSel    = DL_ADC12_CLOCK_SYSOSC,
        .freqRange   = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
        .divideRatio = DL_ADC12_CLOCK_DIVIDE_8,
    };
    DL_ADC12_setClockConfig(ADC0, &clkCfg);

    /* 采样时间 (Car_sum) */
    DL_ADC12_setSampleTime0(ADC0, 400);

    /* 序列模式: 一次 SC 触发, 硬件自动扫 MEM0→...→MEM6 */
    DL_ADC12_initSeqSample(ADC0,
        DL_ADC12_REPEAT_MODE_DISABLED,
        DL_ADC12_SAMPLING_SOURCE_AUTO,
        DL_ADC12_TRIG_SRC_SOFTWARE,
        DL_ADC12_SEQ_START_ADDR_00,
        DL_ADC12_SEQ_END_ADDR_06,
        DL_ADC12_SAMP_CONV_RES_12_BIT,
        DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);

    /* CH0 - PA27 */
    DL_ADC12_configConversionMem(ADC0, DL_ADC12_MEM_IDX_0,
        DL_ADC12_INPUT_CHAN_0, DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

    /* CH1 - PA26 */
    DL_ADC12_configConversionMem(ADC0, DL_ADC12_MEM_IDX_1,
        DL_ADC12_INPUT_CHAN_1, DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

    /* CH2 - PA25 */
    DL_ADC12_configConversionMem(ADC0, DL_ADC12_MEM_IDX_2,
        DL_ADC12_INPUT_CHAN_2, DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

    /* CH3 - PA24 */
    DL_ADC12_configConversionMem(ADC0, DL_ADC12_MEM_IDX_3,
        DL_ADC12_INPUT_CHAN_3, DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

    /* CH4 - PB25 */
    DL_ADC12_configConversionMem(ADC0, DL_ADC12_MEM_IDX_4,
        DL_ADC12_INPUT_CHAN_4, DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

    /* CH5 - PB24 */
    DL_ADC12_configConversionMem(ADC0, DL_ADC12_MEM_IDX_5,
        DL_ADC12_INPUT_CHAN_5, DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

    /* CH6 - PA16 */
    DL_ADC12_configConversionMem(ADC0, DL_ADC12_MEM_IDX_6,
        DL_ADC12_INPUT_CHAN_6, DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

    DL_ADC12_enableConversions(ADC0);
    delay_cycles(16000);
}

/*
 * 序列模式下一次 SC 转换全部 7 路, 逐个返回各 MEM 结果
 * initSeqSample 已设 START=0, END=6, 无需每次改址
 */
uint16_t adc0_read_channel(uint8_t channel)
{
    if (channel > 6) return 0;

    DL_ADC12_enableConversions(ADC0);
    DL_ADC12_startConversion(ADC0);
    while (DL_ADC12_getStatus(ADC0) & DL_ADC12_STATUS_CONVERSION_ACTIVE) {}
    return (uint16_t)DL_ADC12_getMemResult(ADC0, (DL_ADC12_MEM_IDX)channel);
}

/* ============================== ADC1 (1通道) ============================== */

void adc1_init(void)
{
    DL_ADC12_reset(ADC1);
    DL_ADC12_enablePower(ADC1);

    static const DL_ADC12_ClockConfig clkCfg = {
        .clockSel    = DL_ADC12_CLOCK_SYSOSC,
        .freqRange   = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
        .divideRatio = DL_ADC12_CLOCK_DIVIDE_8,
    };
    DL_ADC12_setClockConfig(ADC1, &clkCfg);
    DL_ADC12_setSampleTime0(ADC1, 400);

    DL_ADC12_initSingleSample(ADC1,
        DL_ADC12_REPEAT_MODE_DISABLED,
        DL_ADC12_SAMPLING_SOURCE_AUTO,
        DL_ADC12_TRIG_SRC_SOFTWARE,
        DL_ADC12_SAMP_CONV_RES_12_BIT,
        DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);

    DL_ADC12_configConversionMem(ADC1, DL_ADC12_MEM_IDX_0,
        DL_ADC12_INPUT_CHAN_0, DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

    DL_ADC12_enableConversions(ADC1);
    delay_cycles(16000);
}

uint16_t adc1_read_channel(uint8_t channel)
{
    if (channel > 0) return 0;

    DL_ADC12_disableConversions(ADC1);
    DL_ADC12_setStartAddress(ADC1, DL_ADC12_SEQ_START_ADDR_00);
    DL_ADC12_setEndAddress(ADC1, DL_ADC12_SEQ_END_ADDR_00);
    DL_ADC12_enableConversions(ADC1);
    DL_ADC12_startConversion(ADC1);
    while (DL_ADC12_getStatus(ADC1) & DL_ADC12_STATUS_CONVERSION_ACTIVE) {}
    return (uint16_t)DL_ADC12_getMemResult(ADC1, DL_ADC12_MEM_IDX_0);
}
