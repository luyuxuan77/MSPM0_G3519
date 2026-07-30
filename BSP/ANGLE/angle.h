#ifndef __ANGLE_H
#define __ANGLE_H

#include <stdint.h>

/* ============================================================================
 * ANGLE 模块 — 方向环控制
 * ----------------------------------------------------------------------------
 * 分两套独立的方向环, 参数/节拍互不干扰:
 *
 *  1) GO100 航向锁定环  —— 走直线任务用。
 *     每 128ms 调用 Go100_Update() 一次(与速度内环同步),
 *     内部完成: 航向PID + 里程累加 + 编码器纠偏 + 接近减速停车。
 *
 *  2) 通用 HOLD 原地方向环 —— 原地锁航向, 手拨自动转回。
 *     每 10ms 调用 AngleHold_Update() 一次(快节拍, 响应更跟手),
 *     内部完成: 航向PID(带微分滤波) + 迟滞死区 + 接近减速。
 *     (ANGLE_SYSID=1 时改为边前进边开环方波, 输出CSV给MATLAB辨识)
 *
 * 两套环各自的可调参数都在 angle.c 顶部。
 * 启停标志 g_go100_active/running、g_angle_active/running 仍由 menu.c 管理。
 * ============================================================================ */

/* ===== GO 100cm 直线任务 ===== */
/* 复位内部累加器。idle 或每次启动前调用。 */
void Go100_Reset(void);
/* 每 128ms 调用一次(慢环): 里程 + 编码器纠偏 + 减速曲线 + 到距离停车。到达清 g_go100_running。 */
void Go100_Update(void);
/* 每 10ms 调用一次(快航向环): 航向PID, 写 Speed_Pid[0/1].SetPoint。高频纠航向=走直线关键。 */
void Go100_HeadingUpdate(void);

/* ===== 通用 HOLD 原地方向环 ===== */
/* 复位内部状态。idle 时调用。 */
void AngleHold_Reset(void);
/* 快节拍(10ms)调用一次: 原地锁航向差速, 直接写 Speed_Pid[0/1].SetPoint。 */
void AngleHold_Update(void);

#endif
