#include "angle.h"
#include "bsp.h"      /* Yaw, nowtime, Speed_Pid, 电机/里程接口, menu 标志 */

/* menu.c 里的启停标志 */
extern uint8_t g_go100_active, g_go100_running;
extern uint8_t g_angle_active, g_angle_running;

/* ============================================================================
 * 一、GO 100cm 直线任务参数 (128ms 节拍)
 * ----------------------------------------------------------------------------
 * 只改 GO100_TARGET_CM(在 menu.h), 其余按百分比自动缩放。
 * 冲过头→调大 STOP_LEAD_PERCENT; 没到就停→调小。
 * 偏一侧→加大 YAW_KP; 越纠越偏→ STEER_SIGN 改 -1。
 * ============================================================================ */
#define GO100_CRUISE_CM_S          40.0f
#define GO100_DECEL_START_PERCENT  0.60f
#define GO100_CREEP_CM_S           9.8f
#define GO100_STOP_LEAD_PERCENT    0.02f   /* 0.07→0.02: 内环提速后刹车滑行变短(~2cm), 提前量随之减小 */
#define GO100_DECEL_START_CM       (GO100_TARGET_CM * GO100_DECEL_START_PERCENT)
#define GO100_STOP_LEAD            (GO100_TARGET_CM * GO100_STOP_LEAD_PERCENT)
#define GO100_STOP_CM              (GO100_TARGET_CM - GO100_STOP_LEAD)

/* 航向环: 现在 10ms 快跑(与 HOLD 同节拍), 用 MATLAB 驾驶态辨识增益(正是走直线工况)。
   微分只在 Yaw 真更新(20ms)时算, ×50, 消除采样错配锯齿。
   还偏→加大 KP; 摆→减小 KP 或调小 D_LPF; 慢慢偏不回→加大 KI。 */
#define GO100_YAW_KP               6.3721f  /* MATLAB 驾驶态辨识(wc=3) */
#define GO100_YAW_KI               0.0175f
#define GO100_YAW_KD               0.9286f
#define GO100_YAW_D_LPF            0.7742f  /* 微分低通 */
#define GO100_YAW_DB               0.8f     /* 航向死区(度): 误差小于此不纠, 消除走直线时的微摆。
                                              还微摆→调大(1.2); 会明显偏→调小(0.4) */
#define GO100_YAW_I_LIMIT          40.0f
#define GO100_YAW_STEER_LIMIT      120.0f   /* 航向输出限幅(快环下放开, 纠偏有力) */
#define GO100_STEER_SIGN           (+1.0f)
#define GO100_TOTAL_STEER_LIM      150.0f

static float   g_go100_cruise      = GO100_CRUISE_CM_S;
static float   g_go100_start_yaw   = 0;
static float   g_go100_yaw_isum    = 0;
static float   g_go100_last_yaw_err= 0;
static float   g_go100_dfilt       = 0;   /* 航向 D: 滤波后微分 */
static float   g_go100_last_yaw    = 0;   /* 检测 yaw 是否真更新 */
static float   g_go100_steer       = 0;   /* 编码器纠偏(128ms环更新) */
static float   g_go100_dist_L      = 0;
static float   g_go100_dist_R      = 0;
static float   g_go100_dist_isum   = 0;
static uint8_t g_go100_init        = 0;

void Go100_Reset(void)
{
    g_go100_init = 0;
    g_go100_steer = 0;
    g_go100_cruise = GO100_CRUISE_CM_S;
    g_go100_yaw_isum = 0;
    g_go100_last_yaw_err = 0;
    g_go100_dfilt = 0;
}

/*
 * Go100_Update() — 每 128ms 调用一次(慢环)。
 * 只管: 里程累加 + 编码器纠偏 + 减速曲线 + 到距离停车。
 * 不写 SetPoint(交给快航向环), 但到达目标时清 running 并停车。
 */
void Go100_Update(void)
{
    float dL = get_motor_distance_cm(0);
    float dR = get_motor_distance_cm(1);
    Odometry_Update(dL, dR, Yaw);

    if (!(g_go100_active && g_go100_running)) {
        Go100_Reset();
        return;
    }
    if (!g_go100_init) return;   /* 等快航向环先锁基准(快环10ms先跑, 通常已就绪) */

    const Odometry_t* odo = Odometry_GetData();

    /* --- 编码器慢纠偏(I-only): 只补轮径差, 增益小免得跟航向环打架 --- */
    g_go100_dist_L += dL;
    g_go100_dist_R += dR;
    float diff = g_go100_dist_L - g_go100_dist_R;
    g_go100_dist_isum += diff * 0.08f;
    if (g_go100_dist_isum >  10.0f) g_go100_dist_isum =  10.0f;
    if (g_go100_dist_isum < -10.0f) g_go100_dist_isum = -10.0f;
    g_go100_steer = g_go100_dist_isum;

    /* --- 减速停车(到达则清 running, 快环随即停止输出) --- */
//    if (odo->total_dist >= GO100_STOP_CM) {
//        g_go100_running = 0;
//        g_go100_cruise = 0;
//        Speed_Pid[0].SetPoint = 0;
//        Speed_Pid[1].SetPoint = 0;
//    } else if (odo->total_dist >= GO100_DECEL_START_CM) {
//        float ratio = (GO100_STOP_CM - odo->total_dist)
//                    / (GO100_STOP_CM - GO100_DECEL_START_CM);
//        if (ratio < 0.0f) ratio = 0.0f;
//        if (ratio > 1.0f) ratio = 1.0f;
//        g_go100_cruise = GO100_CREEP_CM_S
//           + (GO100_CRUISE_CM_S - GO100_CREEP_CM_S) * ratio;
//    } else {
//        g_go100_cruise = GO100_CRUISE_CM_S;
//    }
}

/*
 * Go100_HeadingUpdate() — 每 10ms 调用一次(快航向环)。
 * 用当前 g_go100_cruise(慢环给的速度) + 航向PID + 编码器纠偏, 写 SetPoint。
 * 高频纠航向是走直线的关键(128ms 太粗会飘)。
 */
void Go100_HeadingUpdate(void)
{
    if (!(g_go100_active && g_go100_running))
        return;   /* 未启动, 不动 */

    if (!g_go100_init) {
        /* 快环自初始化, 按下K1后10ms内就锁基准、立刻输出, 起步零延迟 */
        g_go100_dist_L = 0;
        g_go100_dist_R = 0;
        g_go100_dist_isum = 0;
        g_go100_yaw_isum = 0;
        g_go100_last_yaw_err = 0;
        g_go100_dfilt = 0;
        g_go100_last_yaw = Yaw;
        g_go100_steer = 0;
        g_go100_cruise = GO100_CRUISE_CM_S;
        g_go100_start_yaw = Yaw;   /* 锁定当前航向 */
        g_go100_init = 1;
    }

    float sp = cm_s_to_speed_counts(g_go100_cruise);

    float yaw_err = g_go100_start_yaw - Yaw;
    if (yaw_err >  180.0f) yaw_err -= 360.0f;
    if (yaw_err < -180.0f) yaw_err += 360.0f;

    float yaw_steer;
    if (yaw_err > -GO100_YAW_DB && yaw_err < GO100_YAW_DB) {
        /* 航向死区: 基本正了就不纠, 消除微摆(编码器纠偏仍保留) */
        yaw_steer = 0.0f;
        g_go100_dfilt = 0.0f;
        g_go100_last_yaw_err = yaw_err;
        g_go100_last_yaw = Yaw;
    } else {
        g_go100_yaw_isum += yaw_err * GO100_YAW_KI;
        if (g_go100_yaw_isum >  GO100_YAW_I_LIMIT) g_go100_yaw_isum =  GO100_YAW_I_LIMIT;
        if (g_go100_yaw_isum < -GO100_YAW_I_LIMIT) g_go100_yaw_isum = -GO100_YAW_I_LIMIT;

        /* 微分只在 Yaw 真更新(20ms)时算, ×50, 消除采样错配锯齿 */
        if (Yaw != g_go100_last_yaw) {
            float d_raw = (yaw_err - g_go100_last_yaw_err) * 50.0f;
            g_go100_dfilt += (d_raw - g_go100_dfilt) * GO100_YAW_D_LPF;
            g_go100_last_yaw_err = yaw_err;
            g_go100_last_yaw = Yaw;
        }

        yaw_steer = yaw_err * GO100_YAW_KP + g_go100_yaw_isum + g_go100_dfilt * GO100_YAW_KD;
    }
    if (yaw_steer >  GO100_YAW_STEER_LIMIT) yaw_steer =  GO100_YAW_STEER_LIMIT;
    if (yaw_steer < -GO100_YAW_STEER_LIMIT) yaw_steer = -GO100_YAW_STEER_LIMIT;

    float total_steer = (yaw_steer + g_go100_steer) * GO100_STEER_SIGN;
    if (total_steer >  GO100_TOTAL_STEER_LIM) total_steer =  GO100_TOTAL_STEER_LIM;
    if (total_steer < -GO100_TOTAL_STEER_LIM) total_steer = -GO100_TOTAL_STEER_LIM;

    Speed_Pid[0].SetPoint = sp + total_steer;  /* +steer → 左转 */
    Speed_Pid[1].SetPoint = sp - total_steer;
}

/* ============================================================================
 * 二、通用 HOLD 原地方向环参数 (10ms 快节拍)
 * ----------------------------------------------------------------------------
 * 增益来自 MATLAB 辨识(wc=8)。调参速查 :
 *   回正慢→加大 STEER_LIMIT; 到位摆→调小 STEER_MIN 或调大 SLOW_ZONE;
 *   高频抖→调小 D_LPF 或减小 KD; 末端轻微动→调大 DEADBAND;
 *   手拨不回正→调小 RELOCK; 发散→ STEER_SIGN 改 -1。
 * ============================================================================ */
#define ANGLE_YAW_KP        6.3721f
#define ANGLE_YAW_KI        0.0175f
#define ANGLE_YAW_KD        0.9286f
#define ANGLE_D_LPF         0.7742f   /* 微分低通(小=平滑抗噪) */
#define ANGLE_YAW_I_LIMIT      60.0f
#define ANGLE_STEER_LIMIT      250.0f    /* 远处快转轮速上限(150→250, 快准狠。≈25cm/s) */
#define ANGLE_SLOW_ZONE        45.0f     /* 恢复到"大幅+小幅、结果正"那次的值 */
#define ANGLE_STEER_MIN        12.0f     /* 恢复到"大幅+小幅、结果正"那次的值 */
#define ANGLE_STEER_SIGN       (+1.0f)
#define ANGLE_DEADBAND         1.2f      /* 3.0→1.2: 死区收窄, 停得更正(新增益阻尼好, 内环32ms, 撑得住) */
#define ANGLE_RELOCK           3.0f      /* 9→3: 迟滞间隔随之收窄, 手拨响应更灵敏(须>DEADBAND) */

/* 系统辨识采集开关: 1=边前进边开环方波(输出CSV给MATLAB); 0=正常原地保持 */
#define ANGLE_SYSID            0
#define ANGLE_SYSID_FWD_CM_S   20.0f
#define ANGLE_SYSID_STEER_AMP  25.0f
#define ANGLE_SYSID_PERIOD     1500u

static float    g_angle_start_yaw    = 0;
static float    g_angle_yaw_isum     = 0;
static float    g_angle_last_yaw_err = 0;
static float    g_angle_dfilt        = 0;
static float    g_angle_last_yaw     = 0;
static uint8_t  g_angle_locked       = 0;
static uint8_t  g_angle_init         = 0;
static uint32_t g_angle_start_tick   = 0;

void AngleHold_Reset(void)
{
    g_angle_init = 0;
    g_angle_yaw_isum = 0;
    g_angle_last_yaw_err = 0;
    g_angle_dfilt = 0;
    g_angle_locked = 0;
}

/*
 * AngleHold_Update() — 快节拍(10ms)调用。原地锁航向差速, 手拨自动转回。
 */
void AngleHold_Update(void)
{
    if (!g_angle_init) {
        g_angle_start_yaw = Yaw;
        g_angle_yaw_isum = 0;
        g_angle_last_yaw_err = 0;
        g_angle_dfilt = 0;
        g_angle_last_yaw = Yaw;
        g_angle_locked = 0;
        g_angle_start_tick = nowtime;
        g_angle_init = 1;
    }

#if ANGLE_SYSID
    /* 采集模式: 边前进边开环方波转向(辨识 steer→yaw), 输出CSV */
    float fwd = cm_s_to_speed_counts(ANGLE_SYSID_FWD_CM_S);
    float steer = ANGLE_SYSID_STEER_AMP;
    if (((nowtime - g_angle_start_tick) / ANGLE_SYSID_PERIOD) & 1u)
        steer = -ANGLE_SYSID_STEER_AMP;
    Speed_Pid[0].SetPoint = fwd + steer;
    Speed_Pid[1].SetPoint = fwd - steer;
    static uint8_t sysid_div = 0;
    if (++sysid_div >= 2) {           /* 20ms 输出一行, 对齐 IMU */
        sysid_div = 0;
        printf("%lu,%.2f,%.2f,%.2f,%.2f\r\n",
               (unsigned long)nowtime, fwd, Yaw, 0.0f, steer);
    }
#else
    /* 正常模式: 原地方向环保持 */
    float yaw_err = g_angle_start_yaw - Yaw;
    if (yaw_err >  180.0f) yaw_err -= 360.0f;
    if (yaw_err < -180.0f) yaw_err += 360.0f;

    float abs_err = (yaw_err < 0) ? -yaw_err : yaw_err;
    /* 迟滞死区 */
    if (g_angle_locked) {
        if (abs_err > ANGLE_RELOCK) g_angle_locked = 0;
    } else {
        if (abs_err < ANGLE_DEADBAND) g_angle_locked = 1;
    }

    float steer;
    if (g_angle_locked) {
        steer = 0.0f;
        g_angle_yaw_isum = 0.0f;
        g_angle_dfilt = 0.0f;
        g_angle_last_yaw_err = yaw_err;
        g_angle_last_yaw = Yaw;
    } else {
        g_angle_yaw_isum += yaw_err * ANGLE_YAW_KI;
        if (g_angle_yaw_isum >  ANGLE_YAW_I_LIMIT) g_angle_yaw_isum =  ANGLE_YAW_I_LIMIT;
        if (g_angle_yaw_isum < -ANGLE_YAW_I_LIMIT) g_angle_yaw_isum = -ANGLE_YAW_I_LIMIT;
        /* 微分只在 Yaw 真更新时算(yaw 50Hz), 消除采样错配的锯齿噪声。dt=20ms→×50 */
        if (Yaw != g_angle_last_yaw) {
            float yaw_deriv_raw = (yaw_err - g_angle_last_yaw_err) * 50.0f;
            g_angle_dfilt += (yaw_deriv_raw - g_angle_dfilt) * ANGLE_D_LPF;
            g_angle_last_yaw_err = yaw_err;
            g_angle_last_yaw = Yaw;
        }
        steer = (yaw_err * ANGLE_YAW_KP + g_angle_yaw_isum
               + g_angle_dfilt * ANGLE_YAW_KD) * ANGLE_STEER_SIGN;
        /* 接近减速: 补偿速度内环128ms滞后, 近目标慢到冲不过死区 */
        float lim = ANGLE_STEER_LIMIT;
        if (abs_err < ANGLE_SLOW_ZONE) {
            lim = ANGLE_STEER_MIN
                + (ANGLE_STEER_LIMIT - ANGLE_STEER_MIN) * (abs_err / ANGLE_SLOW_ZONE);
        }
        if (steer >  lim) steer =  lim;
        if (steer < -lim) steer = -lim;
    }

    Speed_Pid[0].SetPoint =  steer;  /* +steer → 左转 */
    Speed_Pid[1].SetPoint = -steer;
#endif
}
