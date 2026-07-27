/*------------------------------------------------------------------------------
 * menu_actions.c — 所有菜单叶子节点的动作函数实现
 *
 * 每个函数阻塞执行，用户按 '*' 退出后返回菜单系统。
 * 依赖: bsp.h (聚合所有 BSP 模块)、menu.h (KEYPAD_KEY_NONE)
 *----------------------------------------------------------------------------*/
#include "menu_actions.h"
#include "menu.h"

/* ===== system_time helpers (nowtime is 100us ticks from TimerA1 ISR) ===== */
static inline uint32_t sys_tick_ms(void) { return nowtime / 10U; }

static inline bool sys_elapsed_ms(uint32_t *last, uint32_t iv)
{
    uint32_t n = sys_tick_ms();
    if ((uint32_t)(n - *last) >= iv) { *last = n; return true; }
    return false;
}

static uint8_t wait_key_or_timeout(uint32_t timeout_ms)
{
    uint32_t t = sys_tick_ms();
    while (!sys_elapsed_ms(&t, timeout_ms)) {
        uint8_t k = keypad_scan();
        if (k != KEYPAD_KEY_NONE) return k;
    }
    return KEYPAD_KEY_NONE;
}

/* ================================================================
 *  RGB Cycle
 * ================================================================ */
void act_rgb_cycle(void)
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

/* ================================================================
 *  Buzzer Test
 * ================================================================ */
void act_buzzer(void)
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

/* ================================================================
 *  Motor Test (shared helper)
 * ================================================================ */
static void motor_test_run(uint8_t motor_id, const char *label, int16_t target)
{
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)label);
    OLED_ShowString(0, 2, (u8 *)"Running...");
    OLED_ShowString(0, 4, (u8 *)"Spd:     cm/s");
    OLED_ShowString(0, 6, (u8 *)"PWM:       * exit");

    /* Re-init PID state + encoder baseline + ramp for a clean start */
    motor_speed_pid_init();

    /* Set target speed — ramp will bring it up smoothly from 0 */
    Speed_Pid[0].SetPoint = 0;
    Speed_Pid[1].SetPoint = 0;
    if (motor_id == 0)
        Speed_Pid[0].SetPoint = (float)target;
    else
        Speed_Pid[1].SetPoint = (float)target;

    while (1) {
        if (keypad_scan() == '*') break;

        /* Update display @ ~10 Hz */
        static uint32_t disp_t = 0;
        if (sys_elapsed_ms(&disp_t, 100)) {
            char b[16];
            float speed_cms = get_motor_speed_cm_s(motor_id);
            int pwm = (motor_id == 0) ? pwm0 : pwm1;

            sprintf(b, "%4.1f", (double)speed_cms);
            OLED_ShowString(32, 4, (u8 *)b);

            sprintf(b, "%4d", pwm);
            OLED_ShowString(32, 6, (u8 *)b);
        }
    }

    /* Stop motors */
    Speed_Pid[0].SetPoint = 0;
    Speed_Pid[1].SetPoint = 0;
    motor_speed_pid_init();
}

void act_m0fwd(void) { motor_test_run(0, "M0 Forward",  200); }
void act_m0rev(void) { motor_test_run(0, "M0 Reverse", -200); }
void act_m1fwd(void) { motor_test_run(1, "M1 Forward",  200); }
void act_m1rev(void) { motor_test_run(1, "M1 Reverse", -200); }

/* ================================================================
 *  ADC Monitor (6x8 small font, 8 channels in 3-row grid)
 * ================================================================ */
void act_adc_monitor(void)
{
    OLED_Clear();
    OLED_ShowString_Small(0, 0, (u8 *)"ADC Monitor  *exit");

    uint32_t t = sys_tick_ms();
    while (1) {
        if (keypad_scan() == '*') return;
        if (!sys_elapsed_ms(&t, 200)) continue;

        /* 一次采样8路, 存 g_adc_raw[0..7] */
        adc_sample_all();

        char b[32];

        /* Row 1 (y=2): CH0~CH2 */
        sprintf(b, "0:%4u 1:%4u 2:%4u", g_adc_raw[0], g_adc_raw[1], g_adc_raw[2]);
        OLED_ShowString_Small(0, 2, (u8 *)b);

        /* Row 2 (y=4): CH3~CH5 */
        sprintf(b, "3:%4u 4:%4u 5:%4u", g_adc_raw[3], g_adc_raw[4], g_adc_raw[5]);
        OLED_ShowString_Small(0, 4, (u8 *)b);

        /* Row 3 (y=6): CH6~CH7 */
        sprintf(b, "6:%4u 7:%4u", g_adc_raw[6], g_adc_raw[7]);
        OLED_ShowString_Small(0, 6, (u8 *)b);
    }
}

/* ================================================================
 *  Keypad Test
 * ================================================================ */
void act_keypad_test(void)
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
                if ((uint32_t)(nowtime - star_tick) < 10000) return;
                star_tick = nowtime;
            }
        }
    }
}

/* ================================================================
 *  System Info
 * ================================================================ */
void act_sys_info(void)
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

/* ================================================================
 *  OLED Demo
 * ================================================================ */
void act_oled_demo(void)
{
    OLED_Clear();
    OLED_ShowString(16, 0, (u8 *)"CORE TEST");
    OLED_ShowString(8,  2, (u8 *)"MSPM0G3519");
    OLED_ShowString(24, 4, (u8 *)"OLED OK");
    OLED_ShowString(8,  6, (u8 *)"* to exit");
    while (keypad_scan() == KEYPAD_KEY_NONE);
}

/* ================================================================
 *  UART Monitor — 通用串口监听子界面
 *
 *  显示: 每次接收的数据单独成行, 从左往右显示 ASCII 原文
 *        OLED 128x64, 小字体 6x8, 每行 21 字符, 8 行
 *        Row 0: 标题, Row 1-6: 数据 (每行一次接收批次), Row 7: 统计
 * ================================================================ */

/* ── 行环形缓冲区 (每次接收批次 → 一行) ── */
#define LINE_W    21
#define LINE_MAX  16

typedef struct {
    char     buf[LINE_MAX][LINE_W + 1];
    uint8_t  wr;          /* 下一写入位置 */
    uint8_t  count;       /* 已存储行数 */
    uint32_t total_bytes; /* 累计接收字节数 */
} line_buf_t;

static void line_buf_push(line_buf_t *lb, const char *str)
{
    uint8_t len = 0;
    while (str[len] && len < LINE_W) {
        lb->buf[lb->wr][len] = str[len];
        len++;
    }
    while (len < LINE_W) lb->buf[lb->wr][len++] = ' ';
    lb->buf[lb->wr][LINE_W] = '\0';
    lb->wr = (lb->wr + 1U) % LINE_MAX;
    if (lb->count < LINE_MAX) lb->count++;
}

/* idx=0 → 最新行, idx=count-1 → 最旧行, 越界返回 NULL */
static const char *line_buf_get(const line_buf_t *lb, uint8_t idx)
{
    if (idx >= lb->count) return NULL;
    uint8_t pos = (lb->wr + LINE_MAX - 1U - idx) % LINE_MAX;
    return lb->buf[pos];
}

/* ── 单字节 → ASCII 可显示字符 ── */
static char byte_to_ascii(uint8_t b)
{
    return (b >= 0x20 && b <= 0x7E) ? (char)b : '.';
}

/*
 * 通用 UART 监听子界面 — 自动发送测试数据, 每次接收批次单独成行
 */
static void uart_monitor_run(UART_Regs *inst, const char *name, const char *baud,
                             volatile uint8_t *rb, volatile uint16_t *head,
                             volatile uint16_t *tail, uint16_t buf_sz)
{
    line_buf_t lb = {{{0}}, 0, 0, 0};
    char    title[22];
    char    stat[22];
    uint32_t refresh_tick = 0;
    uint32_t tx_tick      = 0;

    OLED_Clear();
    snprintf(title, sizeof(title), "%s %s *exit", name, baud);
    OLED_ShowString_Small(0, 0, (u8 *)title);

    /* 首次发送测试字符串 */
    {
        char test[20];
        snprintf(test, sizeof(test), "%s TEST\r\n", name);
        if (inst == UART1) {
            uart1_send_string(test);
        } else if (inst == UART4) {
            uart4_send_string(test);
        }
    }

    while (1) {
        if (keypad_scan() == '*') return;

        /* 定时发送测试数据 (每 2 秒) */
        if (sys_elapsed_ms(&tx_tick, 2000)) {
            char test[20];
            snprintf(test, sizeof(test), "%s TEST\r\n", name);
            if (inst == UART1) {
                uart1_send_string(test);
            } else if (inst == UART4) {
                uart4_send_string(test);
            }
        }

        /* ── 刷新 @ 10 Hz ── */
        if (!sys_elapsed_ms(&refresh_tick, 100)) continue;

        /* 从 UART 环形缓冲取出本轮新增字节 → ASCII 文本 */
        char   tmp[256];
        uint16_t tmp_len = 0;
        while (*head != *tail && tmp_len < sizeof(tmp)) {
            uint8_t byte = rb[*tail];
            *tail        = (*tail + 1U) % buf_sz;
            lb.total_bytes++;
            tmp[tmp_len++] = byte_to_ascii(byte);
        }

        /* 本轮新数据按 21 字符拆行, 逐行推入行缓冲 */
        if (tmp_len > 0) {
            uint16_t off = 0;
            while (off < tmp_len) {
                uint16_t chunk = tmp_len - off;
                if (chunk > LINE_W) chunk = LINE_W;
                char line_str[LINE_W + 1];
                uint16_t k;
                for (k = 0; k < chunk; k++) line_str[k] = tmp[off + k];
                line_str[chunk] = '\0';
                line_buf_push(&lb, line_str);
                off += LINE_W;
            }
        }

        /* ── 显示 6 行数据 (最新在上) ── */
        uint8_t r;
        for (r = 0; r < 6; r++) {
            const char *l = line_buf_get(&lb, r);
            if (l) {
                OLED_ShowString_Small(0, 1U + r, (u8 *)l);
            } else {
                OLED_ShowString_Small(0, 1U + r, (u8 *)"                    ");
            }
        }

        /* Row 7: 统计 */
        snprintf(stat, sizeof(stat), "Rx:%05u *exit", lb.total_bytes);
        OLED_ShowString_Small(0, 7, (u8 *)stat);
    }
}

/* ================================================================
 *  UART1 Monitor — 蓝牙串口监听
 * ================================================================ */
void act_uart1_monitor(void)
{
    uart_monitor_run(UART_1_INST, "UART1", "115200",
                     uart1_rx_buf, &uart1_rx_head, &uart1_rx_tail,
                     UART1_RX_BUF_SIZE);
}

/* ================================================================
 *  UART4 Monitor — 步进驱动串口监听
 * ================================================================ */
void act_uart4_monitor(void)
{
    uart_monitor_run(UART_4_INST, "UART4", "115200",
                     uart4_rx_buf, &uart4_rx_head, &uart4_rx_tail,
                     UART4_RX_BUF_SIZE);
}
