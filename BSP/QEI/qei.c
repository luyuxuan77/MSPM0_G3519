#include "QEI/qei.h"

/*
 * QEI (Quadrature Encoder Interface) 初始化
 *
 * Motor0 编码器: TIMG9, PB29(PHA/CCP0), PB30(PHB/CCP1)
 * Motor1 编码器: TIMG8, PC6(PHA/CCP0),  PA22(PHB/CCP1)
 *
 * 编码器引脚不经过 SysConfig（PWM 模块会错误地设为输出），
 * 此处使用直接 IOMUX 值配置为输入捕获模式。
 */

/* IOMUX 定义 (来自 MSPM0G3519 数据手册) */
#define QEI_M0_PHA_IOMUX     (IOMUX_PINCM66)            /* PB29 — TIMG9 CCP0 */
#define QEI_M0_PHA_FUNC      IOMUX_PINCM66_PF_TIMG9_CCP0
#define QEI_M0_PHB_IOMUX     (IOMUX_PINCM67)            /* PB30 — TIMG9 CCP1 */
#define QEI_M0_PHB_FUNC      IOMUX_PINCM67_PF_TIMG9_CCP1

#define QEI_M1_PHA_IOMUX     (IOMUX_PINCM84)            /* PC6  — TIMG8 CCP0 */
#define QEI_M1_PHA_FUNC      IOMUX_PINCM84_PF_TIMG8_CCP0
#define QEI_M1_PHB_IOMUX     (IOMUX_PINCM47)            /* PA22 — TIMG8 CCP1 */
#define QEI_M1_PHB_FUNC      IOMUX_PINCM47_PF_TIMG8_CCP1

/* =========================== Motor0 编码器 (TIMG9) =========================== */

void qei_motor0_init(void)
{
    /*
     * 步骤1: 引脚初始化 — 设为 TIMG CCP 输入功能
     */
    DL_GPIO_initPeripheralInputFunction(QEI_M0_PHA_IOMUX, QEI_M0_PHA_FUNC);
    DL_GPIO_initPeripheralInputFunction(QEI_M0_PHB_IOMUX, QEI_M0_PHB_FUNC);

    /*
     * 步骤2: 复位并上电 TIMG9
     */
    DL_TimerG_reset(TIMG9);
    DL_TimerG_enablePower(TIMG9);

    /*
     * 步骤3: 时钟配置 (总线时钟 80 MHz, 不分频)
     */
    static const DL_Timer_ClockConfig clockCfg = {
        .clockSel    = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale    = 0U,
    };
    DL_TimerG_setClockConfig(TIMG9, (DL_Timer_ClockConfig *)&clockCfg);

    /*
     * 步骤4: 设置 CCP 方向为输入
     */
    DL_TimerG_setCCPDirection(TIMG9, DL_TIMER_CC0_INPUT | DL_TIMER_CC1_INPUT);

    /*
     * 步骤5: QEI 模式配置 — 分别对 CCP0/CCP1 配置，
     *        DL_Timer_configQEI 会设置 CCCTL、IFCTL 及计数器控制寄存器。
     */
    DL_Timer_configQEI(TIMG9, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_0_INDEX);

    DL_Timer_configQEI(TIMG9, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_1_INDEX);

    /*
     * 步骤6: 设置最大计数值 (QEI 使用 Load 寄存器)
     */
    DL_Timer_setLoadValue(TIMG9, 0xFFFF);

    /*
     * 步骤7: 启动计数器
     */
    DL_Timer_startCounter(TIMG9);
}

uint32_t qei_motor0_get_count(void)
{
    return DL_Timer_getTimerCount(TIMG9);
}

/* =========================== Motor1 编码器 (TIMG8) =========================== */

void qei_motor1_init(void)
{
    /*
     * 步骤1: 引脚初始化 — 设为 TIMG CCP 输入功能
     */
    DL_GPIO_initPeripheralInputFunction(QEI_M1_PHA_IOMUX, QEI_M1_PHA_FUNC);
    DL_GPIO_initPeripheralInputFunction(QEI_M1_PHB_IOMUX, QEI_M1_PHB_FUNC);

    /*
     * 步骤2: 复位并上电 TIMG8
     */
    DL_TimerG_reset(TIMG8);
    DL_TimerG_enablePower(TIMG8);

    /*
     * 步骤3: 时钟配置
     */
    static const DL_Timer_ClockConfig clockCfg = {
        .clockSel    = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale    = 0U,
    };
    DL_TimerG_setClockConfig(TIMG8, (DL_Timer_ClockConfig *)&clockCfg);

    /*
     * 步骤4: CCP 方向 = 输入
     */
    DL_TimerG_setCCPDirection(TIMG8, DL_TIMER_CC0_INPUT | DL_TIMER_CC1_INPUT);

    /*
     * 步骤5: QEI 模式配置 — CCP0/CCP1
     */
    DL_Timer_configQEI(TIMG8, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_0_INDEX);

    DL_Timer_configQEI(TIMG8, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_1_INDEX);

    /*
     * 步骤6: 最大计数值
     */
    DL_Timer_setLoadValue(TIMG8, 0xFFFF);

    /*
     * 步骤7: 启动计数器
     */
    DL_Timer_startCounter(TIMG8);
}

uint32_t qei_motor1_get_count(void)
{
    return DL_Timer_getTimerCount(TIMG8);
}
