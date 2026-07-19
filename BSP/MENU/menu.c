#include "MENU/menu.h"
#include "SPI0_LCD/lcd.h"
#include "KEY/key.h"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wpointer-sign"
/* ===== Global settings ===== */
/* index: pos1(sensor[7])..pos4(sensor[1]) | pos5(sensor[6])..pos8(sensor[0]) */
uint16_t g_gray_threshold[8] = {1000,1000,1000,1000,1000,1000,1000,1000};
int      g_track_speed_cm_s    = 40;

/* ===== State machine ===== */
#define STATE_MAIN              0
#define STATE_TRACK             1
#define STATE_THRESHOLD_SELECT  2
#define STATE_THRESHOLD_ADJUST  3

static uint8_t state  = STATE_MAIN;
static uint8_t cursor = 0;   // main: 0=TRACK,1=THRESH / select: 0..7 sensor index
static uint8_t dirty  = 1;   // 1 = need redraw (only redraw on change)
static uint32_t periodic_tick = 0;

#define MAIN_ITEMS   2
#define SENSOR_COUNT 8

#define THR_MIN   500
#define THR_MAX   2500
#define THR_STEP  50

void menu_init(void)
{
    state  = STATE_MAIN;
    cursor = 0;
}

void menu_update(void)
{
    int x, y, i;

    /* ===== Key2 short press ===== */
    if (key2_short) {
        key2_short = 0;
        dirty = 1;
        switch (state) {
        case STATE_MAIN:
            cursor = (cursor + 1) % MAIN_ITEMS;
            break;
        case STATE_THRESHOLD_SELECT:
            cursor = (cursor + 1) % SENSOR_COUNT;
            break;
        case STATE_THRESHOLD_ADJUST:
            if (g_gray_threshold[cursor] > THR_MIN + THR_STEP)
                g_gray_threshold[cursor] -= THR_STEP;
            else
                g_gray_threshold[cursor] = THR_MIN;
            break;
        }
    }

    /* ===== Key2 long press ===== */
    if (key2_long) {
        key2_long = 0;
        dirty = 1;
        switch (state) {
        case STATE_TRACK:
            run_flag = 0;
            Speed_Pid[0].SetPoint = 0;
            Speed_Pid[1].SetPoint = 0;
            state = STATE_MAIN;
            break;
        case STATE_THRESHOLD_SELECT:
            state = STATE_MAIN;
            break;
        case STATE_THRESHOLD_ADJUST:
            state = STATE_THRESHOLD_SELECT;
            break;
        }
    }

    /* ===== Key1 short press ===== */
    if (key1_short) {
        key1_short = 0;
        dirty = 1;
        switch (state) {
        case STATE_MAIN:
            if (cursor == 0) { state = STATE_TRACK; }
            else             { state = STATE_THRESHOLD_SELECT; cursor = 0; }
            break;
        case STATE_TRACK:
            run_flag = !run_flag;
            break;
        case STATE_THRESHOLD_SELECT:
            state = STATE_THRESHOLD_ADJUST;
            break;
        case STATE_THRESHOLD_ADJUST:
            g_gray_threshold[cursor] += THR_STEP;
            if (g_gray_threshold[cursor] > THR_MAX)
                g_gray_threshold[cursor] = THR_MAX;
            break;
        }
    }

    /* ===== Key1 long press ===== */
    if (key1_long) {
        key1_long = 0;
        dirty = 1;
        switch (state) {
        case STATE_THRESHOLD_ADJUST:
            state = STATE_THRESHOLD_SELECT;
            break;
        }
    }

    /* ===== Periodic refresh for dynamic data (TRACK speed) ===== */
    if (state == STATE_TRACK) {
        if (system_time_get_tick_ms() - periodic_tick > 500) {
            dirty = 1;
            periodic_tick = system_time_get_tick_ms();
        }
    }

    /* ===== Draw (only when dirty) ===== */
    if (!dirty) return;
    dirty = 0;

    LCD_Fill(0, 36, 275, 175, WHITE);

    switch (state) {
    /* ---- MAIN MENU ---- */
    case STATE_MAIN:
        LCD_ShowString(0, 40, "-- MENU --", BLACK, WHITE, 16, 1);
        LCD_ShowString(10, 70, (cursor==0)?">":" ", BLACK, WHITE, 16, 1);
        LCD_ShowString(30, 70, "TRACK", BLACK, WHITE, 16, 1);
        LCD_ShowString(10, 95, (cursor==1)?">":" ", BLACK, WHITE, 16, 1);
        LCD_ShowString(30, 95, "THRESHOLD", BLACK, WHITE, 16, 1);
        LCD_ShowString(0, 130, "K1:enter", BLACK, WHITE, 12, 1);
        LCD_ShowString(0, 145, "K2:switch", BLACK, WHITE, 12, 1);
        break;

    /* ---- TRACK ---- */
    case STATE_TRACK:
        LCD_ShowString(0, 40, "TRACK", BLACK, WHITE, 16, 1);
        LCD_ShowString(0, 65, "SPD:", BLACK, WHITE, 16, 1);
        LCD_ShowIntNum(40, 65, g_track_speed_cm_s, 3, BLACK, WHITE, 16);
        LCD_ShowString(0, 90, "L:", BLACK, WHITE, 16, 1);
        LCD_ShowFloatNum1(18, 90, get_motor_speed_cm_s(0), 4, BLACK, WHITE, 16);
        LCD_ShowString(120, 90, "R:", BLACK, WHITE, 16, 1);
        LCD_ShowFloatNum1(138, 90, get_motor_speed_cm_s(1), 4, BLACK, WHITE, 16);
        LCD_ShowString(0, 115, run_flag ? ">> GO <<" : "-- STOP --",
                       run_flag ? RED : BLACK, WHITE, 12, 1);
        LCD_ShowString(0, 165, "K1:run  K2long:back", BLACK, WHITE, 12, 1);
        break;

    /* ---- THRESHOLD: select sensor ---- */
    case STATE_THRESHOLD_SELECT:
        LCD_ShowString(0, 40, "SELECT SENSOR", BLACK, WHITE, 16, 1);
        /* arrow above selected */
        y = 65;
        for (i = 0; i < 8; i++) {
            x = i * 34;
            if (i >= 4) x += 4;
            LCD_ShowString(x, y, (i==cursor) ? "v" : " ", BLACK, WHITE, 16, 1);
        }
        /* sensor numbers */
        y = 82;
        for (i = 0; i < 8; i++) {
            x = i * 34;
            if (i >= 4) x += 4;
            LCD_ShowIntNum(x, y, i+1, 1, BLACK, WHITE, 16);
        }
        /* current threshold of selected */
        LCD_ShowString(0, 115, "THR:", BLACK, WHITE, 16, 1);
        LCD_ShowIntNum(40, 115, g_gray_threshold[cursor], 4, BLACK, WHITE, 16);
        LCD_ShowString(0, 150, "K1:select  K2:move", BLACK, WHITE, 12, 1);
        LCD_ShowString(0, 165, "K2long:back", BLACK, WHITE, 12, 1);
        break;

    /* ---- THRESHOLD: adjust value ---- */
    case STATE_THRESHOLD_ADJUST:
        LCD_ShowString(0, 40, "SENSOR", BLACK, WHITE, 16, 1);
        LCD_ShowIntNum(60, 40, cursor+1, 1, RED, WHITE, 16);
        LCD_ShowIntNum(0, 70, g_gray_threshold[cursor], 4, RED, WHITE, 24);
        LCD_ShowString(0, 120, "K1:+50  K2:-50", BLACK, WHITE, 12, 1);
        LCD_ShowString(0, 140, "K1long:save  K2long:back", BLACK, WHITE, 12, 1);
        break;
    }
}
#pragma clang diagnostic pop
