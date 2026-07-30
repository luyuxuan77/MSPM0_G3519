#include "KEY/key.h"
#include "led.h"

uint8_t key1_flag = 0;
uint8_t key2_flag = 0;
uint8_t mode     = 0;
uint8_t run_flag = 0;

volatile uint8_t key1_short = 0;
volatile uint8_t key1_long  = 0;
volatile uint8_t key2_short = 0;
volatile uint8_t key2_long  = 0;

/* internal: press tracking */
static volatile uint32_t k1_tick   = 0;
static volatile uint32_t k2_tick   = 0;
volatile uint8_t  k1_held   = 0;  // 1 = currently pressed
volatile uint8_t  k2_held   = 0;
static volatile uint8_t  k2_reported = 0;

#define LONG_PRESS_MS  800
#define DEBOUNCE_MS    20

void key_init(void)
{
    NVIC_EnableIRQ(KEY_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(KEY_GPIOC_INT_IRQN);
    LED1(1);
    LED2(1);
}

/*
 * ISR: GPIO falling-edge → record press time
 */
void GROUP1_IRQHandler(void)
{
    uint32_t now = nowtime;
    uint32_t irq;

    /* ---- Key1 (PA17) ---- */
    irq = DL_GPIO_getEnabledInterruptStatus(KEY_Key1_PORT, KEY_Key1_PIN);
    if (irq & KEY_Key1_PIN) {
        DL_GPIO_clearInterruptStatus(KEY_Key1_PORT, KEY_Key1_PIN);
        if (now - k1_tick > DEBOUNCE_MS) {
            k1_tick = now;
            k1_held = 1;
            LED1_toggle;
        }
    }

    /* ---- Key2 (PC5) ---- */
    irq = DL_GPIO_getEnabledInterruptStatus(KEY_Key2_PORT, KEY_Key2_PIN);
    if (irq & KEY_Key2_PIN) {
        DL_GPIO_clearInterruptStatus(KEY_Key2_PORT, KEY_Key2_PIN);
        if (now - k2_tick > DEBOUNCE_MS) {
            k2_tick = now;
            k2_held = 1;
            k2_reported = 0;
            LED2_toggle;
        }
    }
}

/*
 * Called from TIMA1 1ms ISR — checks key release and sets short/long flags
 */
void key_tick_1ms(void)
{
    uint32_t now = nowtime;

    if (k1_held) {
        if (DL_GPIO_readPins(KEY_Key1_PORT, KEY_Key1_PIN) != 0) {  // released
            k1_held = 0;
            uint32_t dt = now - k1_tick;
            if (dt > DEBOUNCE_MS) {
                if (dt < LONG_PRESS_MS) key1_short = 1;
                else                    key1_long  = 1;
            }
        }
    }

    if (k2_held) {
        uint32_t dt = now - k2_tick;
        if (!k2_reported && dt >= DEBOUNCE_MS) {
            key2_short = 1;
            k2_reported = 1;
        }
        if (DL_GPIO_readPins(KEY_Key2_PORT, KEY_Key2_PIN) != 0) {
            k2_held = 0;
            k2_reported = 0;
        }
    }
}

void mode_switch()
{
    switch (mode) {
    case 0:
        if (key2_flag) {
            target_round = key1_flag;
            if (target_round > 5) target_round = 5;
            key2_flag = 0;
        }
        break;
    }
}
