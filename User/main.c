/*------------------------------------------------------------------------------
 * 工程名称： core_test  V2.0
 * 说    明： 多级菜单系统 — OLED 128x64 + 4x4 矩阵键盘
 *            2/8 上下选择  # 确认  * 返回
 *----------------------------------------------------------------------------*/
#include "ti_msp_dl_config.h"
#include "bsp.h"
#include "menu.h"

/* ===== system_time (nowtime 100us) ===== */
static inline uint32_t sys_tick_ms(void) { return nowtime / 10U; }
static inline bool sys_elapsed_ms(uint32_t *last, uint32_t iv)
{
    uint32_t n = sys_tick_ms();
    if ((uint32_t)(n - *last) >= iv) { *last = n; return true; }
    return false;
}

/* ================================================================
 *  菜单动作
 * ================================================================ */

/* ── 辅助: 等按键或超时 ── */
static uint8_t wait_key_or_timeout(uint32_t timeout_ms)
{
    uint32_t t = sys_tick_ms();
    while (!sys_elapsed_ms(&t, timeout_ms)) {
        uint8_t k = keypad_scan();
        if (k != KEYPAD_KEY_NONE) return k;
    }
    return KEYPAD_KEY_NONE;
}

static void act_rgb_cycle(void)
{
    static const uint8_t c[][3] = {
        {255,0,0},{0,255,0},{0,0,255},{255,255,0},
        {0,255,255},{255,0,255},{255,255,255},{0,0,0}
    };
    uint8_t i = 0;
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"RGB Cycle");
    OLED_ShowString(0, 2, (u8 *)"* exit");
    while (1) {
        set_RGB(c[i][0], c[i][1], c[i][2]);
        { char b[16]; sprintf(b, "R%3uG%3uB%3u", c[i][0], c[i][1], c[i][2]);
          OLED_ShowString(0, 4, (u8 *)b); }
        if (wait_key_or_timeout(500) == '*') break;
        i = (i + 1) % 8;
    }
    set_RGB(0, 15, 15);
}

static void act_buzzer(void)
{
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"Buzzer Test");
    OLED_ShowString(0, 2, (u8 *)"Beep ~2KHz");
    OLED_ShowString(0, 4, (u8 *)"* exit");
    uint32_t t = sys_tick_ms();
    while (1) {
        if (keypad_scan() == '*') break;
        if (sys_elapsed_ms(&t, 500)) {
            DL_TimerG_startCounter(TIMG0); delay_ms(100);
            DL_TimerG_stopCounter(TIMG0);
        }
    }
    DL_TimerG_stopCounter(TIMG0);
}

static void act_adc_monitor(void)
{
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"ADC Monitor * exit");
    OLED_ShowString(0, 2, (u8 *)"CH0:     CH1:");
    OLED_ShowString(0, 4, (u8 *)"CH2:     CH3:");
    OLED_ShowString(0, 6, (u8 *)"CH4:CH5:CH6:");
    uint32_t t = sys_tick_ms();
    while (1) {
        if (keypad_scan() == '*') return;
        if (!sys_elapsed_ms(&t, 200)) continue;
        char b[16]; uint16_t v;
        v = adc0_read_channel(0); sprintf(b, "%4u", v); OLED_ShowString(32, 2, (u8 *)b);
        v = adc0_read_channel(1); sprintf(b, "%4u", v); OLED_ShowString(88, 2, (u8 *)b);
        v = adc0_read_channel(2); sprintf(b, "%4u", v); OLED_ShowString(32, 4, (u8 *)b);
        v = adc0_read_channel(3); sprintf(b, "%4u", v); OLED_ShowString(88, 4, (u8 *)b);
        v = adc0_read_channel(4); sprintf(b, "%4u", v); OLED_ShowString(32, 6, (u8 *)b);
        v = adc0_read_channel(5); sprintf(b, "%4u", v); OLED_ShowString(64, 6, (u8 *)b);
        v = adc0_read_channel(6); sprintf(b, "%4u", v); OLED_ShowString(96, 6, (u8 *)b);
    }
}

static void act_keypad_test(void)
{
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"Keypad Test");
    OLED_ShowString(0, 2, (u8 *)"Press any key");
    OLED_ShowString(0, 4, (u8 *)"Key:");
    OLED_ShowString(0, 6, (u8 *)"** twice=exit");
    uint32_t star_tick = 0;
    while (1) {
        uint8_t k = keypad_scan();
        if (k != KEYPAD_KEY_NONE) {
            { char b[16]; sprintf(b, "Key:%c 0x%02X", k, k); OLED_ShowString(0, 4, (u8 *)b); }
            printf("Key: %c\r\n", k);
            if (k == '*') {
                if ((uint32_t)(nowtime - star_tick) < 10000) return;  /* 1s 内双击 */
                star_tick = nowtime;
            }
        }
    }
}

static void act_sys_info(void)
{
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"System Info");
    OLED_ShowString(0, 2, (u8 *)"MSPM0G3519 80M");
    OLED_ShowString(0, 4, (u8 *)"OLED SSD1306");
    OLED_ShowString(0, 6, (u8 *)"Keypad 4x4 Int");
    printf("\r\n=== System Info ===\r\n");
    printf("MCU: MSPM0G3519 LQFP-80\r\nCPU: 80MHz\r\nOLED: SSD1306 SPI0\r\n");
    wait_key_or_timeout(3000);
}

static void act_oled_demo(void)
{
    OLED_Clear();
    OLED_ShowString(16, 0, (u8 *)"CORE TEST");
    OLED_ShowString(8,  2, (u8 *)"MSPM0G3519");
    OLED_ShowString(24, 4, (u8 *)"OLED OK");
    OLED_ShowString(8,  6, (u8 *)"* to exit");
    while (keypad_scan() == KEYPAD_KEY_NONE);
}

/* ── Motor 预留 ── */
static void act_m0fwd(void) { OLED_Clear(); OLED_ShowString(0, 3, (u8 *)"M0 Forward"); delay_ms(1000); }
static void act_m0rev(void) { OLED_Clear(); OLED_ShowString(0, 3, (u8 *)"M0 Reverse"); delay_ms(1000); }
static void act_m1fwd(void) { OLED_Clear(); OLED_ShowString(0, 3, (u8 *)"M1 Forward"); delay_ms(1000); }
static void act_m1rev(void) { OLED_Clear(); OLED_ShowString(0, 3, (u8 *)"M1 Reverse"); delay_ms(1000); }

/* ================================================================
 *  菜单树
 * ================================================================ */
static const menu_node_t menu_motor[] = {
    {"M0 Forward",  act_m0fwd, NULL, 0},
    {"M0 Reverse",  act_m0rev, NULL, 0},
    {"M1 Forward",  act_m1fwd, NULL, 0},
    {"M1 Reverse",  act_m1rev, NULL, 0},
};

static const menu_node_t menu_main[] = {
    {"RGB Cycle",     act_rgb_cycle,    NULL, 0},
    {"Buzzer Test",   act_buzzer,       NULL, 0},
    {"Motor Ctrl",    NULL,       menu_motor, 4},
    {"ADC Monitor",   act_adc_monitor,  NULL, 0},
    {"Keypad Test",   act_keypad_test,  NULL, 0},
    {"Sys Info",      act_sys_info,     NULL, 0},
    {"OLED Demo",     act_oled_demo,    NULL, 0},
};

/* ================================================================
 *  main
 * ================================================================ */
int main(void)
{
    SYSCFG_DL_init();
    uart0_init(115200);
    printf("\r\n=== Core Test V2.0 ===\r\n");

    timerg6_pwm_rgb_init();     set_RGB(0, 15, 15);
    DL_TimerG_stopCounter(TIMG0);
    qei_motor0_init();
    qei_motor1_init();
    adc0_init();
    adc1_init();
    key_init();
    TimerA1_init();
    keypad_init();
    OLED_Init();

    printf("Init OK. Keys: 2/8=nav  #=enter  *=back\r\n\r\n");

    static menu_mgr_t g_menu;
    menu_init(&g_menu, menu_main, 7, "CORE TEST MENU");

    while (1) {
        /*
         * 必须高频调用 keypad_scan(), 让中断标志被及时消费。
         * 消抖由 keypad 内部状态机处理, 无性能问题。
         */
        menu_task(&g_menu);
    }
}
