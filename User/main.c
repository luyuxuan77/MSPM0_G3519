/*------------------------------------------------------------------------------
 * CORE_TEST V2.0 — MSPM0G3519 多级菜单系统
 *
 * 硬件: OLED 128x64 (SSD1306) + 4x4 矩阵键盘
 * 操作: 2/8 上下选择  # 确认  * 返回
 *----------------------------------------------------------------------------*/
#include "ti_msp_dl_config.h"
#include "bsp.h"
#include "menu.h"
#include "menu_actions.h"

/* ================================================================
 *  菜单树定义
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
    {"UART1 Monitor", act_uart1_monitor,NULL, 0},
    {"UART4 Monitor", act_uart4_monitor,NULL, 0},
    {"Keypad Test",   act_keypad_test,  NULL, 0},
    {"Sys Info",      act_sys_info,     NULL, 0},
    {"OLED Demo",     act_oled_demo,    NULL, 0},
};

/* ================================================================
 *  main — 初始化 → 菜单主循环
 * ================================================================ */
int main(void)
{
    /* ---- 1. SysConfig 硬件初始化 ---- */
    SYSCFG_DL_init();

    /* ---- 2. 电机 PWM 置安全态 (CCR=0% 占空比, 避免上电瞬间电机抽动) ---- */
    motor_init();

    /* ---- 3. 基础外设 ---- */
    uart0_init(115200);
    printf("\r\n=== Core Test V2.0 ===\r\n");

    uart1_init(115200);
    printf("UART1 (BT)    init OK @ 115200\r\n");

    uart4_init(115200);
    printf("UART4 (Motor) init OK @ 115200\r\n");

    timerg6_pwm_rgb_init();
    set_RGB(0, 15, 15);
    DL_TimerG_stopCounter(TIMG0);   /* 蜂鸣器默认静音 */

    qei_motor0_init();
    qei_motor1_init();
    adc0_init();
    adc1_init();
    key_init();
    keypad_init();
    OLED_Init();

    /* ---- 4. PID + 电机速度环 ---- */
    Pid_Init(&Speed_Pid[0], &L1_PID_Value_Speed);
    Pid_Init(&Speed_Pid[1], &R1_PID_Value_Speed);
    motor_speed_pid_init();

    /* ---- 5. 系统节拍 (100us) — 必须在电机初始化之后使能, 避免 ISR 提前调用 ---- */
    TimerA1_init();

    printf("Init OK. Keys: 2/8=nav  #=enter  *=back\r\n\r\n");

    /* ---- 6. 菜单主循环 (电机控制由 TimerA1 ISR 调度) ---- */
    static menu_mgr_t g_menu;
    menu_init(&g_menu, menu_main, 9, "CORE TEST MENU");

    while (1) {
        menu_task(&g_menu);
    }
}
