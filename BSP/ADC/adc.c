#include "ADC/adc.h"

/*
 * ADC0 / ADC1 初始化
 *
 * 配置参数:
 *   - 分辨率:     12-bit
 *   - 参考电压:   VREF 内部参考 (2.5V)
 *   - 采样时间:   约 1 µs (可根据输入阻抗调整)
 *   - 转换模式:   单次软件触发
 *   - 时钟源:     ULPCLK (4 MHz, SYSOSC 32 MHz / 8)
 *
 * ADC 引脚不需要额外的 GPIO 数字配置：
 * DL_GPIO_reset() 后引脚处于高阻模拟态，正好是 ADC 输入所需状态。
 */

/* ============================== ADC0 (7通道) ============================== */

void adc0_init(void)
{
    /* 1. 复位并上电 ADC0 */
    DL_ADC12_reset(ADC0);
    DL_ADC12_enablePower(ADC0);

    /*
     * 2. 时钟配置: ULPCLK / 1 = 4 MHz
     * (ULPCLK = SYSOSC / 8 = 4 MHz)
     */
    static const DL_ADC12_ClockConfig clkCfg = {
        .clockSel    = DL_ADC12_CLOCK_ULPCLK,
        .freqRange   = DL_ADC12_CLOCK_FREQ_RANGE_1_TO_4,
        .divideRatio = DL_ADC12_CLOCK_DIVIDE_1,
    };
    DL_ADC12_setClockConfig(ADC0, &clkCfg);

    /*
     * 3. 设置采样时间 (SCOMP0)
     * 采样时间 = (SCOMP0 + 1) / ADC_CLK, SCOMP0=7 → ~1µs @ 8MHz
     */
    DL_ADC12_setSampleTime0(ADC0, 7);

    /*
     * 4. 初始化单次采样模式: 12-bit, 软件触发, 无符号输出
     */
    DL_ADC12_initSingleSample(ADC0,
        DL_ADC12_REPEAT_MODE_DISABLED,
        DL_ADC12_SAMPLING_SOURCE_AUTO,
        DL_ADC12_TRIG_SRC_SOFTWARE,
        DL_ADC12_SAMP_CONV_RES_12_BIT,
        DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);

    /*
     * 5. 配置各通道的 ADC 转换内存
     */
    /* CH0 - PA27 */
    DL_ADC12_configConversionMem(ADC0, DL_ADC12_MEM_IDX_0,
        DL_ADC12_INPUT_CHAN_0,
        DL_ADC12_REFERENCE_VOLTAGE_INTREF,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
        DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED,
        DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

    /* CH1 - PA26 */
    DL_ADC12_configConversionMem(ADC0, DL_ADC12_MEM_IDX_1,
        DL_ADC12_INPUT_CHAN_1,
        DL_ADC12_REFERENCE_VOLTAGE_INTREF,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
        DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED,
        DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

    /* CH2 - PA25 */
    DL_ADC12_configConversionMem(ADC0, DL_ADC12_MEM_IDX_2,
        DL_ADC12_INPUT_CHAN_2,
        DL_ADC12_REFERENCE_VOLTAGE_INTREF,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
        DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED,
        DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

    /* CH3 - PA24 */
    DL_ADC12_configConversionMem(ADC0, DL_ADC12_MEM_IDX_3,
        DL_ADC12_INPUT_CHAN_3,
        DL_ADC12_REFERENCE_VOLTAGE_INTREF,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
        DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED,
        DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

    /* CH4 - PB25 */
    DL_ADC12_configConversionMem(ADC0, DL_ADC12_MEM_IDX_4,
        DL_ADC12_INPUT_CHAN_4,
        DL_ADC12_REFERENCE_VOLTAGE_INTREF,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
        DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED,
        DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

    /* CH5 - PB24 */
    DL_ADC12_configConversionMem(ADC0, DL_ADC12_MEM_IDX_5,
        DL_ADC12_INPUT_CHAN_5,
        DL_ADC12_REFERENCE_VOLTAGE_INTREF,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
        DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED,
        DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

    /* CH6 - PA16 */
    DL_ADC12_configConversionMem(ADC0, DL_ADC12_MEM_IDX_6,
        DL_ADC12_INPUT_CHAN_6,
        DL_ADC12_REFERENCE_VOLTAGE_INTREF,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
        DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED,
        DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

    /* 6. 使能 ADC0 转换 */
    DL_ADC12_enableConversions(ADC0);

    /* 7. 等待 ADC 稳定 (约 200µs @ 80MHz) */
    delay_cycles(16000);
}

uint16_t adc0_read_channel(uint8_t channel)
{
    if (channel > 6) return 0;

    /* 设置起始地址为指定通道 */
    DL_ADC12_setStartAddress(ADC0, channel);

    /* 软件触发指定通道 */
    DL_ADC12_startConversion(ADC0);
    while (DL_ADC12_getStatus(ADC0) & DL_ADC12_STATUS_CONVERSION_ACTIVE)
        ;

    return DL_ADC12_getMemResult(ADC0, (DL_ADC12_MEM_IDX)channel);
}

/* ============================== ADC1 (1通道) ============================== */

void adc1_init(void)
{
    /* 1. 复位并上电 ADC1 */
    DL_ADC12_reset(ADC1);
    DL_ADC12_enablePower(ADC1);

    /*
     * 2. 时钟配置
     */
    static const DL_ADC12_ClockConfig clkCfg = {
        .clockSel    = DL_ADC12_CLOCK_ULPCLK,
        .freqRange   = DL_ADC12_CLOCK_FREQ_RANGE_1_TO_4,
        .divideRatio = DL_ADC12_CLOCK_DIVIDE_1,
    };
    DL_ADC12_setClockConfig(ADC1, &clkCfg);

    /*
     * 3. 采样时间
     */
    DL_ADC12_setSampleTime0(ADC1, 7);

    /*
     * 4. 初始化单次采样模式
     */
    DL_ADC12_initSingleSample(ADC1,
        DL_ADC12_REPEAT_MODE_DISABLED,
        DL_ADC12_SAMPLING_SOURCE_AUTO,
        DL_ADC12_TRIG_SRC_SOFTWARE,
        DL_ADC12_SAMP_CONV_RES_12_BIT,
        DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);

    /*
     * 5. CH0 — PA15
     */
    DL_ADC12_configConversionMem(ADC1, DL_ADC12_MEM_IDX_0,
        DL_ADC12_INPUT_CHAN_0,
        DL_ADC12_REFERENCE_VOLTAGE_INTREF,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
        DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED,
        DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

    /* 6. 使能 ADC1 */
    DL_ADC12_enableConversions(ADC1);
    delay_cycles(16000);
}

uint16_t adc1_read_channel(uint8_t channel)
{
    if (channel > 0) return 0;

    DL_ADC12_startConversion(ADC1);
    while (DL_ADC12_getStatus(ADC1) & DL_ADC12_STATUS_CONVERSION_ACTIVE)
        ;

    return DL_ADC12_getMemResult(ADC1, DL_ADC12_MEM_IDX_0);
}
