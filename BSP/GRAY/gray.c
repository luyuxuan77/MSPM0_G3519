/*
 * gray.c
 * GW Grayscale Sensor via Software I2C (移植自 pro_bigger_car111)
 * I2C: PA0=SCL, PA1=SDA, Sensor address: 0x4C
 */

#include "bsp.h"

/* ---- GW Gray Sensor register map ---- */
#define GW_GRAY_ADDR_DEF          0x4CU
#define GW_GRAY_DIGITAL_MODE      0xDDU
#define GW_GRAY_ANALOG_MODE       0xB0U
#define GW_GRAY_ANALOG_NORMALIZE  0xCFU
#define GW_GRAY_CH_EN_ALL         0xFFU

/* ---- Retain ADC0 read for battery voltage (XT30) ---- */
volatile bool gCheckADC = false;

uint16_t get_adc0_value()
{
    uint16_t gAdcResult = 0;
    DL_ADC12_setStartAddress(ADC_gray_INST, DL_ADC12_SEQ_START_ADDR_04);
    DL_ADC12_startConversion(ADC_gray_INST);

    while (false == gCheckADC) {
    }

    gAdcResult = DL_ADC12_getMemResult(ADC_gray_INST, DL_ADC12_MEM_IDX_0);

    gCheckADC = false;
    DL_ADC12_enableConversions(ADC_gray_INST);
    return gAdcResult;
}

void ADC_gray_INST_IRQHandler(void)
{
    switch (DL_ADC12_getPendingInterrupt(ADC_gray_INST))
    {
        case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
            gCheckADC = true;
            break;
        default:
            break;
    }
}

/* ---- 灰度传感器全局数据 ---- */
uint16_t gray_now_val[8] = {0};    /* 原始模拟值 (0-255) */
uint8_t  offset_s = 0;              /* 数字位图 */
uint8_t  gw_digital = 0;            /* 数字量 (来自传感器) */
uint8_t  gw_analog[8] = {0};        /* 模拟量 (Anolog) */
uint8_t  gw_normalize[8] = {0};     /* 归一化值 (Normalize) */
uint8_t  gw_rx_buf[256] = {0};      /* 串口打印缓冲 */

/*
 * gw_i2c_read_task()
 *
 * I2C 背景读取任务，由主循环中 g_i2c_read_flag 触发
 *   1. 读取数字量 (Digtal)
 *   2. 读取模拟量 (Anolog)
 *   3. 开启归一化 → 读取归一化值 (Normalize) → 关闭归一化
 *   4. 通过UART0打印三行数据
 */
void gw_i2c_read_task(void)
{
    /* ---- 1. 数字量 ---- */
    IIC_ReadBytes(GW_GRAY_ADDR_DEF << 1, GW_GRAY_DIGITAL_MODE,
                  &gw_digital, 1);

    /* ---- 2. 模拟量 ---- */
    IIC_ReadBytes(GW_GRAY_ADDR_DEF << 1, GW_GRAY_ANALOG_MODE,
                  &gw_analog[0], 8);

    /* ---- 3. 归一化 ---- */
    {
        uint8_t norm_cmd[2];
        norm_cmd[0] = GW_GRAY_ANALOG_NORMALIZE;
        norm_cmd[1] = GW_GRAY_CH_EN_ALL;
        IIC_WriteBytes(GW_GRAY_ADDR_DEF << 1, GW_GRAY_ANALOG_NORMALIZE,
                       &norm_cmd[1], 2);
        delay_ms(10);

        IIC_ReadBytes(GW_GRAY_ADDR_DEF << 1, GW_GRAY_ANALOG_MODE,
                      &gw_normalize[0], 8);

        norm_cmd[1] = 0x00;
        IIC_WriteBytes(GW_GRAY_ADDR_DEF << 1, GW_GRAY_ANALOG_NORMALIZE,
                       &norm_cmd[1], 2);
    }

#if 0  /* 灰度串口调试: 置1开启 */
    /* ---- 4. 串口打印 ---- */
    sprintf((char *)gw_rx_buf,
            "Digtal %d-%d-%d-%d-%d-%d-%d-%d\r\n",
            (gw_digital >> 0) & 0x01, (gw_digital >> 1) & 0x01,
            (gw_digital >> 2) & 0x01, (gw_digital >> 3) & 0x01,
            (gw_digital >> 4) & 0x01, (gw_digital >> 5) & 0x01,
            (gw_digital >> 6) & 0x01, (gw_digital >> 7) & 0x01);
    printf("%s", gw_rx_buf);

    sprintf((char *)gw_rx_buf,
            "Anolog %d-%d-%d-%d-%d-%d-%d-%d\r\n",
            gw_analog[0], gw_analog[1], gw_analog[2], gw_analog[3],
            gw_analog[4], gw_analog[5], gw_analog[6], gw_analog[7]);
    printf("%s", gw_rx_buf);

    sprintf((char *)gw_rx_buf,
            "Normalize %d-%d-%d-%d-%d-%d-%d-%d\r\n",
            gw_normalize[0], gw_normalize[1], gw_normalize[2], gw_normalize[3],
            gw_normalize[4], gw_normalize[5], gw_normalize[6], gw_normalize[7]);
    printf("%s", gw_rx_buf);
#endif
}

/*
 * 加权暗心叠加算法 (Weighted Darkness Centroid)
 *
 * 原理：
 *   1. Digtal 位做黑白判断：bit=0 表示压线（黑），bit=1 表示离线（白）
 *      → 不再依赖 g_gray_threshold，无需阈值校准即可判断线位置
 *   2. Normalize 值计算暗度：darkness = 255 - normalize_value
 *      → 值越大越黑，校准后的数据所有探头一致，能精确比较暗度深浅
 *   3. 暗度加权质心：goffset = Σ(weight × darkness) / Σ(darkness) × GAIN
 *      → 同时压到两个探头时，暗度比例决定小车走在线中间
 *
 * 传感器物理排列 (左 → 右):
 *   探头1  探头2  探头3  探头4 | 探头5  探头6  探头7  探头8
 *   bit0   bit1   bit2   bit3 | bit4   bit5   bit6   bit7  (gw_digital)
 *   [0]    [1]    [2]    [3]  | [4]    [5]    [6]    [7]   (gw_normalize)
 */

/* 位置权重: 负=左, 正=右, 外侧大内侧小, 中心微调 */
static const int8_t probe_weight[8] = {
     20,   /* 探头1: 最左侧 */
     17,   /* 探头2: 左中外 */
      3,   /* 探头3: 左中内 */
      1,   /* 探头4: 中心偏左, 微调 */
     -1,   /* 探头5: 中心偏右, 微调 */
     -3,   /* 探头6: 右中内 */
    -17,   /* 探头7: 右中外 */
    -20    /* 探头8: 最右侧 */
};

/* 圆弧循迹权重: 中间加强, 外侧适度 */
static const int8_t probe_weight_arc[8] = {
     0,   /* 探头1: 最左侧 */
     21,   /* 探头2: 左中外 */
     20,   /* 探头3: 左中内 */
      8,   /* 探头4: 中心偏左 */
     -8,   /* 探头5: 中心偏右 */
    -20,   /* 探头6: 右中内 */
    -21,   /* 探头7: 右中外 */
    -0    /* 探头8: 最右侧 */
};

uint8_t g_arc_mode = 0;  /* 0=直线循迹 1=圆弧循迹, task2设置 */

/*
 * get_gray_refresh_data()
 *
 * 使用 gw_digital (Digtal) 判断压线 + gw_normalize (Normalize) 计算暗度质心。
 * I2C 数据由 gw_i2c_read_task() 在后台更新。
 * g_arc_mode=1 时使用圆弧权重数组。
 *
 * Returns: goffset — 加权暗心偏差 (负=偏左需右转, 正=偏右需左转)
 */
int get_gray_refresh_data(void)
{
#define GOFFSET_GAIN 3   /* 转向灵敏度, 越大越激进 */

    const int8_t *pw = g_arc_mode ? probe_weight_arc : probe_weight;

    offset_s = 0;
    int total_darkness = 0;
    int weighted_sum = 0;

    for (int i = 0; i < 8; i++) {
        uint8_t on_line = !((gw_digital >> i) & 0x01);  /* bit=0 → 压线 */

        if (on_line) {
            /* 暗度 = 255 - normalize, 全黑=255, 全白=0 */
            int darkness = 255 - (int)gw_normalize[i];
            if (darkness < 0) darkness = 0;

            total_darkness += darkness;
            weighted_sum  += pw[i] * darkness;
            offset_s      |= (1 << i);
        }
    }

    int goffset = (total_darkness > 0)
        ? (weighted_sum * GOFFSET_GAIN / total_darkness) : 0;

    return goffset;
}

uint8_t get_offset_s(void)
{
    return offset_s;
}
