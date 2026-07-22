/*
 * ============================================================================
 * INS / Odometry 模块 — 编码器 + 陀螺仪 2D 航位推算
 * ============================================================================
 *
 * 【算法】
 *   每个控制周期 (128ms):
 *     dist = (dist_L + dist_R) / 2          — 小车中心平均前进距离
 *     x   += dist * cos(yaw_rad)            — 世界坐标 X 积分
 *     y   += dist * sin(yaw_rad)            — 世界坐标 Y 积分
 *
 * 【数据来源】
 *   距离: motor.c 的 get_motor_distance_cm() → 编码器差分 → cm
 *   航向: IMU.c 的 Yaw 全局变量 → Mahony AHRS 六轴解算 (陀螺积分为主)
 *
 * 【精度说明】
 *   - 编码器距离: ±2~5% (受轮径标定、打滑影响)
 *   - 航向漂移: ~1~2°/min (陀螺零偏, 无磁力计绝对参考)
 *   - 综合定位误差: 行驶距离的 3~8%
 *
 * 【旧版 INS】
 *   旧 INS_Update 使用加速度二次积分, 精度不可接受, 已弃用。
 *   旧 API 保留空实现以确保编译兼容。
 * ============================================================================
 */

#include "INS.h"
#include <math.h>
#include <string.h>

/* ---- 里程计全局状态 ---- */
static Odometry_t g_odometry;

/* ---- 旧版 INS 状态 (保留兼容) ---- */
static INS_Data_t ins_data;

/* ---- Yaw 全局变量 (由 TIMA1 ISR 每 20ms 从 IMU 更新) ---- */
volatile float Roll, Pitch, Yaw;

/* ===================================================================
 * 新版 Odometry API
 * =================================================================== */

/*
 * Odometry_Init() — 初始化里程计
 *
 * 清零所有位置和里程数据。
 * 调用时机: 系统上电后、每次比赛开始前。
 */
void Odometry_Init(void)
{
    memset(&g_odometry, 0, sizeof(g_odometry));
}

/*
 * Odometry_Update() — 里程计主更新
 *
 * 每个控制周期调用一次, 用编码器距离和陀螺航向更新 2D 位置。
 *
 * 参数:
 *   dist_L_cm — 本周期左轮前进距离 (cm), 后退为负
 *   dist_R_cm — 本周期右轮前进距离 (cm), 后退为负
 *   yaw_deg   — 当前 IMU 航向角 (度)
 */
void Odometry_Update(float dist_L_cm, float dist_R_cm, float yaw_deg)
{
    float dist;
    float yaw_rad;

    /* 小车中心平均前进距离 */
    dist = (dist_L_cm + dist_R_cm) * 0.5f;

    /* 航向转弧度 */
    yaw_rad = yaw_deg * 3.1415926f / 180.0f;

    /* 2D 位置积分 */
    g_odometry.x += dist * cosf(yaw_rad);
    g_odometry.y += dist * sinf(yaw_rad);

    /* 累计里程 */
    g_odometry.yaw_deg    = yaw_deg;
    g_odometry.total_dist += (dist >= 0.0f ? dist : -dist);  /* 绝对值里程 */
    g_odometry.dist_L     += (dist_L_cm >= 0.0f ? dist_L_cm : -dist_L_cm);
    g_odometry.dist_R     += (dist_R_cm >= 0.0f ? dist_R_cm : -dist_R_cm);
}

/*
 * Odometry_Reset() — 一键归零
 *
 * 清零位置和里程, 航向保持当前值不变。
 * 用途: 按键触发, 标记新的起点。
 */
void Odometry_Reset(void)
{
    g_odometry.x          = 0.0f;
    g_odometry.y          = 0.0f;
    g_odometry.total_dist = 0.0f;
    g_odometry.dist_L     = 0.0f;
    g_odometry.dist_R     = 0.0f;
    /* yaw_deg 保持不变 — 清零位置但不清零航向 */
}

/*
 * Odometry_GetData() — 获取里程计只读指针
 */
const Odometry_t* Odometry_GetData(void)
{
    return &g_odometry;
}

/* ===================================================================
 * 旧版 INS API (保留空实现, 确保编译兼容)
 * =================================================================== */

void INS_Init(void)
{
    ins_data.ax = 0;
    ins_data.ay = 0;
    ins_data.vx = 0;
    ins_data.vy = 0;
    ins_data.x  = 0;
    ins_data.y  = 0;
}

/*
 * 旧版加速度积分 — 已弃用, 空实现
 *
 * 原因: 消费级 IMU 加速度计噪声导致位置快速发散,
 *       在 10 秒内误差可超过 2.5 米, 不可用于实际导航。
 *       请使用 Odometry_Update() 替代。
 */
void INS_Update(float ax, float ay, float yaw, float dt)
{
    (void)ax;
    (void)ay;
    (void)yaw;
    (void)dt;
    /* 已弃用 — 不做任何计算 */
}

void IMU_get(void)
{
    /* 已弃用 — 请使用 IMU_getYawPitchRoll() 或 Odometry API */
}

float IMU_yaw(void)
{
    return Yaw;
}

INS_Data_t *INS_GetData(void)
{
    /* 返回旧版结构 (始终为零) — 请使用 Odometry_GetData() */
    return &ins_data;
}
