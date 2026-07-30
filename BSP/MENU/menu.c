#include "MENU/menu.h"
#include "SPI0_LCD/lcd.h"
#include "KEY/key.h"
#include "ADC0/adc0.h"
#include "UART4_OPI/uart4_opi.h"
#include "INS.h"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wpointer-sign"

#define MENU_ITEM_COUNT      8

/* ===== Global timer (0.01s precision, no-flicker LCD) ===== */
static uint32_t g_timer_base_ms = 0;
static uint32_t g_timer_frozen_cs = 0;
static uint8_t  g_timer_running = 0;

void task_timer_reset(void)
{
    g_timer_base_ms = system_time_get_tick_ms();
    g_timer_frozen_cs = 0;
}

void task_timer_start(void)
{
    g_timer_base_ms = system_time_get_tick_ms();
    g_timer_frozen_cs = 0;
    g_timer_running = 1;
}

void task_timer_stop(void)
{
    if (g_timer_running) {
        g_timer_frozen_cs = (system_time_get_tick_ms() - g_timer_base_ms) / 10U;
    }
    g_timer_running = 0;
}

uint32_t task_timer_get_cs(void)
{
    if (g_timer_running) {
        return (system_time_get_tick_ms() - g_timer_base_ms) / 10U;
    }
    return g_timer_frozen_cs;  /* 停止后冻结, 持续显示 */
}

/*
 * Draw elapsed timer as "XX.XXs" at given position.
 * Uses change-only redraw — no LCD flicker.
 * Call every frame; only draws when value actually changes.
 */
void task_timer_draw(uint16_t x, uint16_t y, uint16_t color, uint16_t bg)
{
    static char s_last[8] = "";
    char buf[8];
    uint32_t cs = task_timer_get_cs();
    uint32_t sec = cs / 100U;
    uint32_t hs  = cs % 100U;
    sprintf(buf, "%2u.%02us", (unsigned)sec, (unsigned)hs);
    if (strcmp(s_last, buf) != 0) {
        LCD_ShowString(x, y, (u8 *)buf, color, bg, 16, 0);
        strcpy(s_last, buf);
    }
}

#define MENU_GRID_COLS       2
#define MENU_CARD_W          132
#define MENU_CARD_H          27
#define MENU_CARD_X0         4
#define MENU_CARD_Y0         58
#define MENU_CARD_X_GAP      8
#define MENU_CARD_Y_GAP      8


uint16_t g_gray_threshold[8] = {100,100,100,100,100,100,100,100};  /* I2C 8-bit: 0=black ~255=white, tune via menu */
int      g_track_speed_cm_s = 20
	;
uint8_t  g_go100_active     = 0;
uint8_t  g_go100_running    = 0;
uint8_t  g_angle_active     = 0;
uint8_t  g_angle_running    = 0;

uint8_t  g_menu_task_id          = MENU_TASK_NONE;
uint8_t  g_menu_running          = 0;
uint8_t  g_menu_recording        = 0;
uint32_t g_menu_task_start_ms    = 0;
uint32_t g_menu_task_elapsed_ms  = 0;
int16_t  g_menu_setpoint_mm      = 0;
uint16_t g_angle_adc_raw         = 0;
int16_t  g_angle_cdeg            = 0;

static uint8_t s_cursor = 0;
static uint8_t s_page_task = MENU_TASK_NONE;
static uint8_t s_dirty = 1;
static uint8_t s_t6_k2_pending = 0;
static uint32_t s_t6_k2_tick = 0;
static uint8_t s_t6_k2_long_done = 0;

#define T6_K2_LONG_MS 900

static const struct {
    uint8_t id;
    const char *text;
} s_menu_items[MENU_ITEM_COUNT] = {
    {MENU_TASK_VIDEO,        "T1 VIDEO"},
    {MENU_TASK_LINE_LAP,     "T2 LINE"},
    {MENU_TASK_BALL_SWING,   "T3 BALL"},
    {MENU_TASK_A_TO_B,       "T4 A-B"},
    {MENU_TASK_FULL_LAP,     "T5 FULL"},
    {MENU_TASK_SETPOINT_LAP, "T6 SETPT"},
    {MENU_TASK_ANGLE_MON,    "ADC ANG"},
    {MENU_TASK_UART_STATUS,  "UART"},
};

static uint8_t is_chassis_task(uint8_t task_id)
{
    return (task_id == MENU_TASK_LINE_LAP ||
            task_id == MENU_TASK_A_TO_B ||
            task_id == MENU_TASK_FULL_LAP ||
            task_id == MENU_TASK_SETPOINT_LAP);
}

static uint8_t opi_task_id_for_menu(uint8_t task_id)
{
    switch (task_id) {
    case MENU_TASK_VIDEO:        return TASK_ID_MONITOR;
    case MENU_TASK_LINE_LAP:     return TASK_ID_LAP_20S;
    case MENU_TASK_BALL_SWING:   return TASK_ID_STATIC_SWING;
    case MENU_TASK_A_TO_B:       return TASK_ID_AB_BALANCE;
    case MENU_TASK_FULL_LAP:     return TASK_ID_LAP_BALANCE;
    case MENU_TASK_SETPOINT_LAP: return TASK_ID_LAP_SETPOINT;
    default:                     return MENU_RECORD_TASK_ID;
    }
}

static const char *task_title(uint8_t task_id)
{
    for (uint8_t i = 0; i < MENU_ITEM_COUNT; i++) {
        if (s_menu_items[i].id == task_id) return s_menu_items[i].text;
    }
    return "TASK";
}

static void motor_stop_all(void)
{
    Speed_Pid[0].SetPoint = 0;
    Speed_Pid[1].SetPoint = 0;
    g_go100_active = 0;
    g_go100_running = 0;
    g_angle_active = 0;
    g_angle_running = 0;
    run_flag = 0;
}

void menu_stop_task(void)
{
    if (g_menu_running) {
        g_menu_task_elapsed_ms = system_time_get_tick_ms() - g_menu_task_start_ms;
    }

    task_timer_stop();  /* 停止全局计时器 */

    g_menu_running = 0;
    motor_stop_all();

    if (g_menu_recording) {
        int16_t setpoint_mm = (g_menu_task_id == MENU_TASK_SETPOINT_LAP) ? g_menu_setpoint_mm : 0;
        opi_task_stop(opi_task_id_for_menu(g_menu_task_id), setpoint_mm);
        g_menu_recording = 0;
    }

    s_dirty = 1;
}

static void menu_start_task(uint8_t task_id)
{
    g_menu_task_id = task_id;
    g_menu_running = 1;
    g_menu_task_start_ms = system_time_get_tick_ms();
    g_menu_task_elapsed_ms = 0;

    task_timer_start();  /* 启动全局计时器 */

    Odometry_Reset();
    motor_speed_pid_init();
    route_finish = 0;

    uint8_t opi_task_id = opi_task_id_for_menu(task_id);

    if (task_id == MENU_TASK_SETPOINT_LAP) {
        opi_task_start(opi_task_id, g_menu_setpoint_mm);
    } else {
        opi_task_start(opi_task_id, 0);
    }
    g_menu_recording = 1;

    if (is_chassis_task(task_id)) {
        target_round = 1;
        run_flag = 1;
    } else {
        motor_stop_all();
        g_menu_running = 1;
    }

    s_dirty = 1;
}

static void draw_main_item(uint8_t idx)
{
    uint8_t col = idx % MENU_GRID_COLS;
    uint8_t row = idx / MENU_GRID_COLS;
    uint16_t x = MENU_CARD_X0 + col * (MENU_CARD_W + MENU_CARD_X_GAP);
    uint16_t y = MENU_CARD_Y0 + row * (MENU_CARD_H + MENU_CARD_Y_GAP);
    uint8_t selected = (idx == s_cursor);
    uint16_t fg = selected ? BLUE : BLACK;

    LCD_Fill(x, y, x + MENU_CARD_W, y + MENU_CARD_H, WHITE);
    LCD_DrawRectangle(x, y, x + MENU_CARD_W, y + MENU_CARD_H, selected ? BLUE : LGRAY);
    LCD_ShowString(x + 8, y + 7, (const u8 *)s_menu_items[idx].text, fg, WHITE, 16, 0);
}

/* ---- Full-redraw tracking (only on page/state switch, not periodic) ---- */
static uint8_t s_need_full_redraw = 1;

static void draw_main_menu(void)
{
    if (s_need_full_redraw) {
        s_need_full_redraw = 0;
        LCD_Fill(0, 36, LCD_W, LCD_H, WHITE);
        LCD_ShowString(0, 40, "BALL CAR MENU", BLACK, WHITE, 16, 1);
        LCD_ShowString(176, 42, "K2:SEL", BLUE, WHITE, 12, 1);

        for (uint8_t idx = 0; idx < MENU_ITEM_COUNT; idx++) {
            draw_main_item(idx);
        }

        LCD_ShowString(0, 200, "K1:enter  K2L:back", BLACK, WHITE, 12, 1);
    }
    /* Cursor changes handled incrementally in key2_short above */
}

/* ---- Change-only helper: clear old area + draw new, no residual pixels ---- */
static void _lcd_str_changed(uint16_t x, uint16_t y, const char *str,
                             uint16_t fc, uint16_t bg, uint8_t size, uint8_t mode,
                             char *cache, uint8_t cache_sz)
{
    if (strncmp(cache, str, cache_sz) != 0) {
        uint8_t cw = (size == 12) ? 6 : (size == 16) ? 8 : 12;  /* char width */
        uint8_t old_w = (uint8_t)strlen(cache) * cw;
        uint8_t new_w = (uint8_t)strlen(str) * cw;
        uint8_t clr_w = (old_w > new_w) ? old_w : new_w;
        if (clr_w > 0) LCD_Fill(x, y, x + clr_w, y + size - 1, bg);
        LCD_ShowString(x, y, (const u8 *)str, fc, bg, size, mode);
        strncpy(cache, str, cache_sz - 1);
        cache[cache_sz - 1] = '\0';
    }
}

/* ---- TASK2 clean page: title + TIME label + centered timer ---- */
static void draw_task2_page(void)
{
    static char s_status[6] = "";

    /* Full redraw only on page entry / task state change */
    if (s_need_full_redraw) {
        s_need_full_redraw = 0;
        LCD_Fill(0, 36, LCD_W, LCD_H, WHITE);
        LCD_ShowString(60, 40, (const u8 *)task_title(MENU_TASK_LINE_LAP), BLACK, WHITE, 20, 1);
        LCD_ShowString(50, 85, (const u8 *)"TIME", BLACK, WHITE, 16, 1);
        LCD_ShowString(0, 175, (const u8 *)"K1:start/stop  K2L:back", BLACK, WHITE, 12, 1);
        s_status[0] = '\0';  /* force status redraw */
        /* Force timer redraw: LCD_Fill erased it, strcmp cache won't retrigger */
        {
            uint32_t cs = task_timer_get_cs();
            char _tb[8];
            sprintf(_tb, "%2u.%02us", (unsigned)(cs / 100U), (unsigned)(cs % 100U));
            LCD_ShowString(50, 108, (const u8 *)_tb, BLUE, WHITE, 16, 0);
        }
    }

    /* RUN/STOP — redraw own area only when changed */
    {
        const char *st = g_menu_running ? "RUN" : "STOP";
        uint16_t c = g_menu_running ? GREEN : RED;
        if (strncmp(s_status, st, sizeof(s_status)) != 0) {
            LCD_Fill(170, 40, 230, 56, WHITE);          /* clear local zone */
            LCD_ShowString(180, 42, (const u8 *)st, c, WHITE, 16, 1);
            strncpy(s_status, st, sizeof(s_status) - 1);
        }
    }

    /* Timer — internal change-only, clears its own area */
    task_timer_draw(50, 108, BLUE, WHITE);
}

static void draw_task_page(void)
{
    char buf[32];
    static char s_title[12] = "";
    static char s_status[6] = "";
    static char s_adc[24] = "";
    static char s_dist[16] = "";
    static char s_rec[32] = "";
    static char s_extra[32] = "";
    static char s_hint[24] = "";

    /* Full redraw only on page entry / task state change */
    if (s_need_full_redraw) {
        s_need_full_redraw = 0;
        LCD_Fill(0, 36, LCD_W, LCD_H, WHITE);
        /* Force timer redraw: LCD_Fill erased it */
        {
            uint32_t cs = task_timer_get_cs();
            char _tb[8];
            sprintf(_tb, "%2u.%02us", (unsigned)(cs / 100U), (unsigned)(cs % 100U));
            LCD_ShowString(0, 65, (const u8 *)_tb, BLUE, WHITE, 16, 0);
        }
    }

    /* Title */
    _lcd_str_changed(0, 40, (const char *)task_title(s_page_task),
                     BLACK, WHITE, 16, 1, s_title, sizeof(s_title));

    /* RUN/STOP */
    {
        const char *st = g_menu_running ? "RUN " : "STOP";
        uint16_t c = g_menu_running ? GREEN : RED;
        _lcd_str_changed(160, 40, st, c, WHITE, 16, 1, s_status, sizeof(s_status));
    }

    /* Timer */
    task_timer_draw(0, 65, BLUE, WHITE);

    /* ADC + Angle */
    sprintf(buf, "ADC:%4u ANG:%+4d", g_angle_adc_raw, g_angle_cdeg);
    _lcd_str_changed(0, 88, buf, MAGENTA, WHITE, 16, 0, s_adc, sizeof(s_adc));

    /* Distance */
    {
        const Odometry_t *odo = Odometry_GetData();
        sprintf(buf, "D:%5.1fcm", odo->total_dist);
        _lcd_str_changed(0, 111, buf, RED, WHITE, 16, 0, s_dist, sizeof(s_dist));
    }

    /* Recording status */
    sprintf(buf, "REC:%s ACK:%03u/%u", g_menu_recording ? "ON " : "OFF",
            g_opi_last_ack_cmd, g_opi_last_ack_status);
    _lcd_str_changed(0, 134, buf, BLUE, WHITE, 12, 0, s_rec, sizeof(s_rec));

    /* Extra line */
    buf[0] = '\0';
    if (s_page_task == MENU_TASK_SETPOINT_LAP) {
        sprintf(buf, "SP:%+4dmm K2:+/L-", g_menu_setpoint_mm);
    } else if (s_page_task == MENU_TASK_UART_STATUS) {
        sprintf(buf, "TA:%u TX:%u", g_opi_task_ack_count, g_opi_sensor_tx_count);
    }
    _lcd_str_changed(0, 150, buf, BLACK, WHITE, 12, 0, s_extra, sizeof(s_extra));

    /* Hint */
    _lcd_str_changed(0, 175, "K1:start/stop  K2L:back",
                     BLACK, WHITE, 12, 1, s_hint, sizeof(s_hint));
}

void menu_init(void)
{
    s_cursor = 0;
    s_page_task = MENU_TASK_NONE;
    s_dirty = 0;
    g_menu_task_id = MENU_TASK_NONE;
    g_menu_running = 0;
    g_menu_recording = 0;
    g_menu_task_elapsed_ms = 0;
    g_menu_setpoint_mm = 0;
    draw_main_menu();
}

void menu_update(void)
{
    if (key2_short) {
        key2_short = 0;
        if (s_page_task == MENU_TASK_NONE) {
            uint8_t old_cursor = s_cursor;

            s_cursor = (s_cursor + 1) % MENU_ITEM_COUNT;
            draw_main_item(old_cursor);
            draw_main_item(s_cursor);
            s_dirty = 0;
        } else if (s_page_task == MENU_TASK_SETPOINT_LAP && !g_menu_running) {
            s_t6_k2_pending = 1;
            s_t6_k2_tick = system_time_get_tick_ms();
            s_t6_k2_long_done = 0;
        }
    }

    if (key2_long) {
        key2_long = 0;
        if (s_page_task != MENU_TASK_NONE) {
            /* T6未运行时K2用于调setpoint, 不触发返回 */
            if (s_page_task == MENU_TASK_SETPOINT_LAP && !g_menu_running) {
                /* K2 reserved for setpoint */
            } else {
                menu_stop_task();
                s_page_task = MENU_TASK_NONE;
                task_timer_reset();   /* 退回主界面, 计时归零待命 */
                s_dirty = 1;
            }
        }
    }

    if (key1_long) {
        key1_long = 0;
        if (s_page_task != MENU_TASK_NONE) {
            menu_stop_task();
            s_page_task = MENU_TASK_NONE;
            task_timer_reset();   /* 退回主界面, 计时归零待命 */
            s_dirty = 1;
        }
    }

    if (key1_short) {
        key1_short = 0;
        s_dirty = 1;
        if (s_page_task == MENU_TASK_NONE) {
            s_page_task = s_menu_items[s_cursor].id;
            g_menu_task_id = s_page_task;
            g_menu_task_elapsed_ms = 0;
            Odometry_Reset();
        } else {
            if (g_menu_running) menu_stop_task();
            else menu_start_task(s_page_task);
        }
    }

    if (g_menu_running && g_menu_task_id == MENU_TASK_BALL_SWING && opi_consume_task3_done()) {
        menu_stop_task();
    }

    if (g_menu_running && is_chassis_task(g_menu_task_id) && run_flag == 0) {
        menu_stop_task();
    }

    if (s_page_task == MENU_TASK_SETPOINT_LAP && !g_menu_running && s_t6_k2_pending) {
        uint32_t now = system_time_get_tick_ms();
        if (k2_held) {
            if (!s_t6_k2_long_done && now - s_t6_k2_tick >= T6_K2_LONG_MS) {
                g_menu_setpoint_mm -= MENU_SETPOINT_STEP_MM;
                if (g_menu_setpoint_mm < MENU_SETPOINT_MIN_MM) g_menu_setpoint_mm = MENU_SETPOINT_MAX_MM;
                s_t6_k2_long_done = 1;
                s_t6_k2_pending = 0;
                s_dirty = 1;
            }
        } else {
            if (!s_t6_k2_long_done) {
                g_menu_setpoint_mm += MENU_SETPOINT_STEP_MM;
                if (g_menu_setpoint_mm > MENU_SETPOINT_MAX_MM) g_menu_setpoint_mm = MENU_SETPOINT_MIN_MM;
                s_dirty = 1;
            }
            s_t6_k2_pending = 0;
        }
    }

    /* Full redraw flag: set by page switch / state change events above */
    if (s_dirty) {
        s_need_full_redraw = 1;
        s_dirty = 0;
    }

    /* Always draw every 50ms — each element decides locally whether to redraw */
    if (s_page_task == MENU_TASK_NONE) {
        draw_main_menu();
    } else if (s_page_task == MENU_TASK_LINE_LAP) {
        draw_task2_page();                /* T2: 简洁, TIME + 居中计时 */
    } else {
        draw_task_page();                /* T1/T3~T8: 完整信息 */
    }
}

#pragma clang diagnostic pop
