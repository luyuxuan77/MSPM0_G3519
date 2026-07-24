#ifndef MENU_H
#define MENU_H

#include "bsp.h"

/* ── 菜单项类型 ── */
typedef void (*menu_action_t)(void);

typedef struct menu_node {
    const char             *text;     /* 显示文本 (最多 14 字符) */
    menu_action_t           action;   /* 叶子节点: 执行函数, NULL 则进子菜单 */
    const struct menu_node *submenu;  /* 子菜单列表首地址 */
    uint8_t                 count;    /* 子菜单项数 (0 = 叶子节点) */
} menu_node_t;

/* ── 菜单管理器 ── */
typedef struct {
    const menu_node_t *items;    /* 当前菜单项数组 */
    uint8_t            count;    /* 当前菜单项数 */
    uint8_t            cursor;   /* 光标位置 (0 ~ count-1) */
    uint8_t            scroll;   /* 列表滚动偏移 */
    const char        *title;    /* 当前菜单标题 */

    /* 导航栈 (支持最多 4 级深度) */
    const menu_node_t *stack_items[4];
    uint8_t            stack_count[4];
    uint8_t            stack_cursor[4];
    uint8_t            stack_scroll[4];
    uint8_t            depth;
} menu_mgr_t;

/* ── API ── */
void menu_init(menu_mgr_t *m, const menu_node_t *root, uint8_t root_count,
               const char *title);
void menu_task(menu_mgr_t *m);   /* 主循环每 20~50ms 调用一次 */

#endif
