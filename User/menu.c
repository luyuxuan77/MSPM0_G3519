#include "menu.h"

/*
 * OLED 128x64, 字体 SIZE=16 (每字符占 2 page)。
 * 为避免重叠, 菜单行按 2 page 间距排列:
 *   page 0: 标题    (占用 0+1)
 *   page 2: 菜单项1 (占用 2+3)
 *   page 4: 菜单项2 (占用 4+5)
 *   page 6: 菜单项3 (占用 6+7)
 * → 最多可见 3 项 + 标题, 底行提示融入第 3 项位置或省略。
 *
 * 为多显示内容, 标题/提示用 page 0/7 单独画, 菜单区 page 1~6 放 3 项。
 * 即: page 0=标题, page 1=item0, page 3=item1, page 5=item2+提示。
 */
#define MENU_VISIBLE_MAX  3U
#define MENU_TEXT_LEN    15U

/* ── 画一行 ── */
static void menu_draw_line(uint8_t page, uint8_t is_cursor, const char *text)
{
    char buf[MENU_TEXT_LEN + 4];
    uint8_t len = 0;
    buf[len++] = is_cursor ? '>' : ' ';
    buf[len++] = ' ';
    while (*text && len < sizeof(buf) - 1) buf[len++] = *text++;
    while (len < sizeof(buf) - 1) buf[len++] = ' ';
    buf[len] = '\0';
    OLED_ShowString(0, page, (u8 *)buf);
}

/* ── 滚动: cursor 步进 1, scroll 步进 1 ── */
static void menu_adjust_scroll(menu_mgr_t *m)
{
    if (m->cursor < m->scroll)
        m->scroll = m->cursor;
    if (m->cursor >= m->scroll + MENU_VISIBLE_MAX)
        m->scroll = m->cursor - MENU_VISIBLE_MAX + 1U;
    if (m->scroll + MENU_VISIBLE_MAX > m->count
        && m->count > MENU_VISIBLE_MAX)
        m->scroll = m->count - MENU_VISIBLE_MAX;
    if (m->scroll >= m->count && m->count > 0)
        m->scroll = m->count - 1U;
}

/* ── 全屏重绘 ── */
static void menu_redraw(menu_mgr_t *m)
{
    uint8_t i, visible_end;

    OLED_Clear();

    /* page 0: 标题 */
    {
        char tb[MENU_TEXT_LEN + 3];
        uint8_t j;
        tb[0] = ' ';
        for (j = 0; j < MENU_TEXT_LEN && m->title[j]; j++)
            tb[j + 1] = m->title[j];
        tb[j + 1] = '\0';
        OLED_ShowString(0, 0, (u8 *)tb);
    }

    /* page 2,4,6: 菜单项 (3 行可见, 各占 2 page, 不重叠标题 0+1) */
    visible_end = m->scroll + MENU_VISIBLE_MAX;
    if (visible_end > m->count) visible_end = m->count;

    for (i = m->scroll; i < visible_end; i++) {
        uint8_t page = 2U + (i - m->scroll) * 2U;  /* 2, 4, 6 */
        menu_draw_line(page, (i == m->cursor), m->items[i].text);
    }

    /*
     * 按键提示: 不占用 page 7 (16px 字体会溢出到 page 8)。
     * 用户需知道: 2/8 上下 #确认 *返回。
     */
}

/* ── 进入子菜单 ── */
static void menu_enter(menu_mgr_t *m)
{
    const menu_node_t *item = &m->items[m->cursor];

    if (item->action) {
        OLED_Clear();
        OLED_ShowString(0, 2, (u8 *)"Running...");
        item->action();

        /*
         * 动作函数内部会高频调用 keypad_scan() 检测 * 退出。
         * 返回后可能仍有按键抖动/回弹残留在键盘中断锁存器中。
         * 延时消抖 + 清空缓冲区，避免残留按键被 menu_task 误读为菜单跳转。
         */
        delay_ms(30);
        while (keypad_scan() != KEYPAD_KEY_NONE) {}

        menu_redraw(m);
        return;
    }

    if (item->submenu && item->count > 0) {
        m->stack_items[m->depth]  = m->items;
        m->stack_count[m->depth]  = m->count;
        m->stack_cursor[m->depth] = m->cursor;
        m->stack_scroll[m->depth] = m->scroll;
        m->depth++;
        m->items  = item->submenu;
        m->count  = item->count;
        m->cursor = 0;
        m->scroll = 0;
        menu_adjust_scroll(m);
        menu_redraw(m);

        /* 进入子菜单后同样清空残留按键 */
        delay_ms(30);
        while (keypad_scan() != KEYPAD_KEY_NONE) {}
    }
}

/* ── 返回 ── */
static void menu_back(menu_mgr_t *m)
{
    if (m->depth == 0) return;
    m->depth--;
    m->items  = m->stack_items[m->depth];
    m->count  = m->stack_count[m->depth];
    m->cursor = m->stack_cursor[m->depth];
    m->scroll = m->stack_scroll[m->depth];
    menu_adjust_scroll(m);
    menu_redraw(m);

    /* 清空 * 键释放时可能残留的抖动按键 */
    delay_ms(30);
    while (keypad_scan() != KEYPAD_KEY_NONE) {}
}

/* ── 按键分发 ── */
static void menu_handle_key(menu_mgr_t *m, uint8_t key)
{
    uint8_t prev_cursor = m->cursor;

    switch (key) {
    case '2': m->cursor = (m->cursor > 0) ? m->cursor - 1U : m->count - 1U; break;
    case '8': m->cursor = (m->cursor < m->count - 1U) ? m->cursor + 1U : 0;  break;
    case '#': menu_enter(m); return;
    case '*': menu_back(m);  return;
    default:
        return;
    }

    if (m->cursor != prev_cursor) {
        uint8_t old_scroll = m->scroll;
        menu_adjust_scroll(m);
        if (m->scroll == old_scroll) {
            menu_draw_line(2U + (prev_cursor - m->scroll) * 2U, false,
                           m->items[prev_cursor].text);
            menu_draw_line(2U + (m->cursor - m->scroll) * 2U, true,
                           m->items[m->cursor].text);
        } else {
            menu_redraw(m);
        }
    }
}

/* ── API ── */

void menu_init(menu_mgr_t *m, const menu_node_t *root, uint8_t root_count,
               const char *title)
{
    m->items  = root;
    m->count  = root_count;
    m->cursor = 0;
    m->scroll = 0;
    m->title  = title;
    m->depth  = 0;
    menu_adjust_scroll(m);
    menu_redraw(m);
}

void menu_task(menu_mgr_t *m)
{
    uint8_t key = keypad_scan();
    if (key != KEYPAD_KEY_NONE) menu_handle_key(m, key);
}
