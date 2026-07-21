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
static uint8_t state  = STATE_MAIN;
static uint8_t cursor = 0;
static uint8_t dirty  = 1;

#define MAIN_ITEMS   5
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
        case STATE_THRESHOLD_SELECT:
            state = STATE_MAIN;
            break;
        case STATE_THRESHOLD_ADJUST:
            state = STATE_THRESHOLD_SELECT;
            break;
        case STATE_TASK1:
        case STATE_TASK2:
        case STATE_TASK3:
        case STATE_TASK4:
            state = STATE_MAIN;
            break;
        }
    }

    /* ===== Key1 short press ===== */
    if (key1_short) {
        key1_short = 0;
        dirty = 1;
        switch (state) {
        case STATE_MAIN:
            if      (cursor == 0) { state = STATE_THRESHOLD_SELECT; cursor = 0; }
            else if (cursor == 1) { state = STATE_TASK1; }
            else if (cursor == 2) { state = STATE_TASK2; }
            else if (cursor == 3) { state = STATE_TASK3; }
            else                  { state = STATE_TASK4; }
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

    /* ===== Draw (only when dirty) ===== */
    if (!dirty) return;
    dirty = 0;

    LCD_Fill(0, 36, 275, 175, WHITE);

    switch (state) {
    /* ---- MAIN MENU ---- */
    case STATE_MAIN:
        LCD_ShowString(0, 40, "-- MENU --", BLACK, WHITE, 16, 1);
        LCD_ShowString(10, 70,  (cursor==0)?">":" ", BLACK, WHITE, 16, 1);
        LCD_ShowString(30, 70,  "THRESHOLD", BLACK, WHITE, 16, 1);
        LCD_ShowString(10, 90,  (cursor==1)?">":" ", BLACK, WHITE, 16, 1);
        LCD_ShowString(30, 90,  "TASK 1", BLACK, WHITE, 16, 1);
        LCD_ShowString(10, 110, (cursor==2)?">":" ", BLACK, WHITE, 16, 1);
        LCD_ShowString(30, 110, "TASK 2", BLACK, WHITE, 16, 1);
        LCD_ShowString(10, 130, (cursor==3)?">":" ", BLACK, WHITE, 16, 1);
        LCD_ShowString(30, 130, "TASK 3", BLACK, WHITE, 16, 1);
        LCD_ShowString(10, 150, (cursor==4)?">":" ", BLACK, WHITE, 16, 1);
        LCD_ShowString(30, 150, "TASK 4", BLACK, WHITE, 16, 1);
        LCD_ShowString(0, 180, "K1:enter", BLACK, WHITE, 12, 1);
        LCD_ShowString(0, 195, "K2:switch", BLACK, WHITE, 12, 1);
        break;

    /* ---- THRESHOLD: select sensor ---- */
    case STATE_THRESHOLD_SELECT:
        LCD_ShowString(0, 40, "SELECT SENSOR", BLACK, WHITE, 16, 1);
        y = 65;
        for (i = 0; i < 8; i++) {
            x = i * 34;
            if (i >= 4) x += 4;
            LCD_ShowString(x, y, (i==cursor) ? "v" : " ", BLACK, WHITE, 16, 1);
        }
        y = 82;
        for (i = 0; i < 8; i++) {
            x = i * 34;
            if (i >= 4) x += 4;
            LCD_ShowIntNum(x, y, i+1, 1, BLACK, WHITE, 16);
        }
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

    /* ---- TASK 1 ---- */
    case STATE_TASK1:
        LCD_ShowString(0, 50, "TASK 1", BLACK, WHITE, 24, 1);
        LCD_ShowString(0, 95, "Coming soon...", BLACK, WHITE, 16, 1);
        LCD_ShowString(0, 165, "K2long:back", BLACK, WHITE, 12, 1);
        break;

    /* ---- TASK 2 ---- */
    case STATE_TASK2:
        LCD_ShowString(0, 50, "TASK 2", BLACK, WHITE, 24, 1);
        LCD_ShowString(0, 95, "Coming soon...", BLACK, WHITE, 16, 1);
        LCD_ShowString(0, 165, "K2long:back", BLACK, WHITE, 12, 1);
        break;

    /* ---- TASK 3 ---- */
    case STATE_TASK3:
        LCD_ShowString(0, 50, "TASK 3", BLACK, WHITE, 24, 1);
        LCD_ShowString(0, 95, "Coming soon...", BLACK, WHITE, 16, 1);
        LCD_ShowString(0, 165, "K2long:back", BLACK, WHITE, 12, 1);
        break;

    /* ---- TASK 4 ---- */
    case STATE_TASK4:
        LCD_ShowString(0, 50, "TASK 4", BLACK, WHITE, 24, 1);
        LCD_ShowString(0, 95, "Coming soon...", BLACK, WHITE, 16, 1);
        LCD_ShowString(0, 165, "K2long:back", BLACK, WHITE, 12, 1);
        break;
    }
}
#pragma clang diagnostic pop
