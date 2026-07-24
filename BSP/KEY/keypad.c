#include "KEY/keypad.h"

extern uint32_t nowtime;

/* ===== 按键映射 ===== */
static const uint8_t key_map[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

static const uint32_t row_pins[4] = {
    MatrixKeypad_R1_PIN, MatrixKeypad_R2_PIN,
    MatrixKeypad_R3_PIN, MatrixKeypad_R4_PIN
};

typedef struct { GPIO_Regs *port; uint32_t pin; } col_pin_t;

static const col_pin_t col_pins[4] = {
    { MatrixKeypad_C1_PORT, MatrixKeypad_C1_PIN },
    { MatrixKeypad_C2_PORT, MatrixKeypad_C2_PIN },
    { MatrixKeypad_C3_PORT, MatrixKeypad_C3_PIN },
    { MatrixKeypad_C4_PORT, MatrixKeypad_C4_PIN }
};

#define ALL_ROW_PINS  (MatrixKeypad_R1_PIN | MatrixKeypad_R2_PIN | \
                       MatrixKeypad_R3_PIN | MatrixKeypad_R4_PIN)

/* ===== 消抖参数 (nowtime 100us tick) ===== */
#define DEBOUNCE_PRESS_TICKS    100   /* 10ms */
#define DEBOUNCE_RELEASE_TICKS  150   /* 15ms */

/* ===== 状态机 ===== */
typedef enum {
    STATE_IDLE,              /* 空闲, 行中断已使能, 等待按键 */
    STATE_DEBOUNCE,          /* 中断触发, 消抖确认中 */
    STATE_PRESSED,           /* 按键已确认并上报, 等待释放 */
    STATE_RELEASE            /* 释放消抖中 */
} kp_state_t;

static volatile uint8_t kp_irq_flag   = 0;  /* ISR 置 1, 主循环消费 */
static          kp_state_t kp_state   = STATE_IDLE;
static          uint8_t    kp_pending = KEYPAD_KEY_NONE;
static          uint32_t   kp_tick    = 0;

/* ===== 底层: 扫描矩阵, 返回按下的键 ===== */
static uint8_t scan_raw(void)
{
    uint8_t col, row;

    for (col = 0; col < 4; col++) {
        /* 只拉低当前列，其余拉高 */
        uint8_t c;
        for (c = 0; c < 4; c++) {
            if (c == col)
                DL_GPIO_clearPins(col_pins[c].port, col_pins[c].pin);
            else
                DL_GPIO_setPins(col_pins[c].port, col_pins[c].pin);
        }

        delay_us(50);

        for (row = 0; row < 4; row++) {
            if (DL_GPIO_readPins(GPIOC, row_pins[row]) == 0) {
                /* 恢复所有列为低电平 (中断空闲态) */
                for (c = 0; c < 4; c++)
                    DL_GPIO_clearPins(col_pins[c].port, col_pins[c].pin);
                return key_map[row][col];
            }
        }
    }

    /* 无按键, 恢复所有列为低电平 */
    for (col = 0; col < 4; col++)
        DL_GPIO_clearPins(col_pins[col].port, col_pins[col].pin);

    return KEYPAD_KEY_NONE;
}

/* ===== 行中断使能/禁止 ===== */
static void rows_interrupt_enable(void)
{
    DL_GPIO_clearInterruptStatus(GPIOC, ALL_ROW_PINS);
    DL_GPIO_enableInterrupt(GPIOC, ALL_ROW_PINS);
}

static void rows_interrupt_disable(void)
{
    DL_GPIO_disableInterrupt(GPIOC, ALL_ROW_PINS);
    DL_GPIO_clearInterruptStatus(GPIOC, ALL_ROW_PINS);
}

/* ===== 初始化: 列全低 + 行中断下降沿 ===== */
void keypad_init(void)
{
    uint8_t i;

    /* 所有列输出低电平 (空闲态 = 任意按键按下即产生下降沿) */
    for (i = 0; i < 4; i++) {
        DL_GPIO_clearPins(col_pins[i].port, col_pins[i].pin);
    }

    /* 4 个行引脚: 下降沿中断 (PC0~PC3 在低 16 位) */
    DL_GPIO_setLowerPinsPolarity(GPIOC,
        DL_GPIO_PIN_0_EDGE_FALL | DL_GPIO_PIN_1_EDGE_FALL |
        DL_GPIO_PIN_2_EDGE_FALL | DL_GPIO_PIN_3_EDGE_FALL);

    rows_interrupt_enable();
    NVIC_EnableIRQ(GPIOC_INT_IRQn);
}

/* ===== ISR: 由 GROUP1_IRQHandler 调用 ===== */
void keypad_isr_handler(void)
{
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(GPIOC, ALL_ROW_PINS);

    if (status) {
        rows_interrupt_disable();
        kp_irq_flag = 1;
    }
}

/* ===== 主循环非阻塞扫描 ===== */
uint8_t keypad_scan(void)
{
    uint8_t raw;
    uint32_t elapsed;

    /* ISR 通知 → 进入消抖状态 */
    if (kp_irq_flag) {
        kp_irq_flag = 0;
        kp_tick     = nowtime;
        kp_state    = STATE_DEBOUNCE;
    }

    switch (kp_state) {

    case STATE_IDLE:
        break;

    case STATE_DEBOUNCE:
        elapsed = (uint32_t)(nowtime - kp_tick);
        if (elapsed < DEBOUNCE_PRESS_TICKS)
            break;

        /* 消抖时间到, 扫描确认 */
        raw = scan_raw();
        if (raw != KEYPAD_KEY_NONE) {
            kp_pending = raw;
            kp_tick    = nowtime;
            kp_state   = STATE_PRESSED;
            return raw;  /* ← 上报按键值 */
        }
        /* 毛刺, 回空闲 */
        rows_interrupt_enable();
        kp_state = STATE_IDLE;
        break;

    case STATE_PRESSED:
        /* 每 2ms 检测一次是否释放 */
        elapsed = (uint32_t)(nowtime - kp_tick);
        if (elapsed < 20)  /* 20 ticks = 2ms */
            break;

        kp_tick = nowtime;
        raw = scan_raw();

        if (raw == KEYPAD_KEY_NONE) {
            kp_tick  = nowtime;
            kp_state = STATE_RELEASE;
        }
        break;

    case STATE_RELEASE:
        elapsed = (uint32_t)(nowtime - kp_tick);
        if (elapsed < DEBOUNCE_RELEASE_TICKS)
            break;

        /* 释放消抖完成 → 回空闲, 重开中断 */
        rows_interrupt_enable();
        kp_state = STATE_IDLE;
        break;
    }

    return KEYPAD_KEY_NONE;
}
