#include "MOTOR/motor.h"

/*
 * ===========================================================================
 * 电机速度控制模块 — PID 速度环 + 低通滤波 + 启动斜坡
 * ===========================================================================
 * 
 * 【硬件平台】MSPM0G3519, TIMG12 四路 PWM, QEI 编码器测速
 * 【PWM 范围】0~1000 (周期), SPEED_PWM_MAX=800 (PID 输出限幅)
 * 【速度单位】编码器计数/控制周期, 实测范围 0~700 (物理上限)
 * 
 * ---------------------------------------------------------------------------
 * 一、信号处理链
 * ---------------------------------------------------------------------------
 * 
 *   编码器原始计数
 *        |
 *   speed_Mx = cnt_curr - cnt_last    (差分得到原始速度)
 *        |
 *   filtered_speed_Mx = 0.3*raw + 0.7*prev   (一阶低通滤波, 平滑量化噪声)
 *        |
 *   ramp_setpoint 替代 SetPoint      (冷启动时从 0 逐步逼近目标, 避免尖峰)
 *        |
 *   Pid_control()                     (位置式 PID, Mode=0)
 *        |
 *   Pid_OutLimit(±800)               (输出限幅)
 *        |
 *   set_motor_speed(pwm1, pwm2, 0, 0)  (驱动 TIMG12 CCR, 控制方向和占空比)
 * 
 * ---------------------------------------------------------------------------
 * 二、低通滤波 (filtered_speed)
 * ---------------------------------------------------------------------------
 * 
 * 公式: filtered = 0.3 * raw_speed + 0.7 * filtered_prev
 * 
 * 编码器差分测速存在量化跳变 (相邻周期可差 50+), 直接送入 PID 会导致
 * D 项放大噪声。一阶 IIR 低通滤波平滑信号, 让 D 项真正起到阻尼作用。
 * 
 * 滤波系数选择:
 *   alpha=0.3 (新值权重): 平衡平滑度和响应速度。太小则滞后大,
 *                        太大则滤不干净。可根据控制周期微调。
 * ---------------------------------------------------------------------------
 * 三、启动斜坡 (ramp_setpoint)
 * ---------------------------------------------------------------------------
 * 
 * 问题: 冷启动时目标速度从 0 跳到 100/200/900, PID 输出瞬间饱和,
 *       电机全速启动 → 过冲 → 急刹 → 振荡, 前 10~15 周期不稳定。
 * 
 * 方案: 用一个虚拟的 ramp_setpoint 替代真实 SetPoint, 每周期指数逼近:
 *       ramp += (target - ramp) * rate
 *       rate=0.25: ~9周期到90%目标, ~16周期到99%
 * 
 * 锁定机制:
 *   - 启动后 ramp 从 0 开始逼近目标
 *   - 接近目标 (差值<1.0) 后 ramp_done=1, 永久锁定
 *   - 锁定后 control.c 的转向差速可立即生效, 不再经过斜坡
 *   - motor_speed_pid_init() 重置 ramp_done=0, 下次启动重新斜坡
 * 
 * 调整 ramp 速率:
 *   - 增大 rate (如 0.3): 启动更快, 但尖峰可能重现
 *   - 减小 rate (如 0.1): 更平滑, 但用户可能感觉"提速慢"
 *   - 当前 0.25 是在 100/200/900 三档验证后的折中值
 * ---------------------------------------------------------------------------
 * 四、变量作用域修正 (pwm1, pwm2)
 * ---------------------------------------------------------------------------
 * 
 * 原代码中 pwm1/pwm2 在 motor_control_update() 内声明为局部变量,
 * 但 pid_speed() 函数需要读取它们来驱动电机。改为文件级 static 变量。
 * ---------------------------------------------------------------------------
 * 五、使用指南
 * ---------------------------------------------------------------------------
 * 
 * 初始化顺序:
 *   1. motor_init()          — 配置 GPIO/PWM 硬件
 *   2. Pid_Init(&Speed_Pid[0/1], ...)  — 加载 PID 参数
 *   3. motor_speed_pid_init()  — 初始化编码器基准 + 重置斜坡
 *   4. Speed_Pid[0/1].SetPoint = 目标速度  — main.c 或 control.c 设置
 *   5. 主循环中周期性调用:
 *        motor_control_update()  — 读取编码器 → 滤波 → PID → 更新 pwm1/pwm2
 *        Pid_Speed()             — 将 pwm1/pwm2 写入 PWM 硬件
 * 
 * 如需调整控制频率:
 *   - 在 main.c 的主循环或定时器中断中调整 motor_control_update() 调用间隔
 *   - 频率越高 → 速度测量分辨率越低 (编码器计数少) → 噪声越大
 *   - 频率越低 → 响应越慢, 但速度测量更稳定
 *   - 当前参数针对 10ms~20ms 控制周期调优
 * ===========================================================================
 */

#define PH1(en) (en) ? DL_GPIO_setPins(MOTOR_PH1_PORT, MOTOR_PH1_PIN) : DL_GPIO_clearPins(MOTOR_PH1_PORT, MOTOR_PH1_PIN)  // 左前轮方向: en=1正转(PH1高), en=0反转(PH1低)
#define PH2(en) (en) ? DL_GPIO_setPins(MOTOR_PH2_PORT, MOTOR_PH2_PIN) : DL_GPIO_clearPins(MOTOR_PH2_PORT, MOTOR_PH2_PIN)  // 右前轮方向
#define PH3(en) (en) ? DL_GPIO_setPins(MOTOR_PH3_PORT, MOTOR_PH3_PIN) : DL_GPIO_clearPins(MOTOR_PH3_PORT, MOTOR_PH3_PIN)  // 左后轮方向
#define PH4(en) (en) ? DL_GPIO_setPins(MOTOR_PH4_PORT, MOTOR_PH4_PIN) : DL_GPIO_clearPins(MOTOR_PH4_PORT, MOTOR_PH4_PIN)  // 右后轮方向



#define NFAULT1 DL_GPIO_readPins(MOTOR_nfault1_PORT, MOTOR_nfault1_PIN)  // 左前轮 DRV8870 nFAULT: 0=故障 1=正常
#define NFAULT2 DL_GPIO_readPins(MOTOR_nfault2_PORT, MOTOR_nfault2_PIN)  // 右前轮故障
#define NFAULT3 DL_GPIO_readPins(MOTOR_nfault3_PORT, MOTOR_nfault3_PIN)  // 左后轮故障
#define NFAULT4 DL_GPIO_readPins(MOTOR_nfault4_PORT, MOTOR_nfault4_PIN)  // 右后轮故障

static uint32_t last_cnt_M1 = 0;  // 左轮上次编码器计数值 (差分测速用)
static uint32_t last_cnt_M2 = 0;  // 右轮上次编码器计数值

static int16_t speed_M1 = 0;  // 左轮原始速度 (编码器计数/控制周期)
static int16_t speed_M2 = 0;  // 右轮原始速度 (负号修正: 右轮编码器方向与左轮相反)

static uint8_t motor_pid_init_flag = 0;  // PID初始化完成标志: 0=跳过控制 1=允许运行


#define SPEED_PWM_MAX 900  // PID输出限幅 (PWM周期1000, 留100余量给刹车和转向叠加)
/**
 * motor_init() — 电机硬件初始化
 *
 * 功能: 配置四路电机的 nSLEEP/PH 引脚和 PWM 输出。
 *
 * 执行步骤:
 *   1. 初始化 nSLEEP 引脚为输出高电平 (唤醒电机驱动芯片)
 *      - 使用 DL_GPIO_initDigitalOutput() 配置 IOMUX
 *      - SysConfig 未自动配置此引脚, 需手动初始化
 *   2. 初始化 PH 引脚为低电平 (全刹车状态, 防止上电抖动)
 *   3. 清零 PWM 占空比 (CCR=1000 对应 0% 占空比)
 *   4. 启动 TIMG12 计数器 (SysConfig 配置为 STOP 状态)
 *
 * 调用时机: 系统上电时调用一次 (main.c 初始化阶段)
 *
 * 硬件依赖:
 *   MOTOR_nsleep1~4: DRV8870 nSLEEP 引脚
 *   MOTOR_PH1~4:     DRV8870 PH/IN2 引脚 (方向)
 *   MOTOR_PWM_INST:  TIMG12, 四路 CCR (CC0/CC1/CC2/CC3)
 *   PWM 周期 1000 (SysConfig 配置)
 *
 * 注意: 必须在 motor_speed_pid_init() 之前调用
 */
void motor_init(void)
{
    // [注释已恢复，见文件头部的完整说明]
    DL_GPIO_initDigitalOutput(MOTOR_nsleep1_IOMUX);  // Step1: 初始化nSLEEP为数字输出 (唤醒DRV8870)
    DL_GPIO_initDigitalOutput(MOTOR_nsleep2_IOMUX);
    DL_GPIO_initDigitalOutput(MOTOR_nsleep3_IOMUX);
    DL_GPIO_initDigitalOutput(MOTOR_nsleep4_IOMUX);
    DL_GPIO_setPins(MOTOR_nsleep1_PORT, MOTOR_nsleep1_PIN);  // nSLEEP=高电平, 使能电机 (低电平=休眠)
    DL_GPIO_setPins(MOTOR_nsleep2_PORT, MOTOR_nsleep2_PIN);
    DL_GPIO_setPins(MOTOR_nsleep3_PORT, MOTOR_nsleep3_PIN);
    DL_GPIO_setPins(MOTOR_nsleep4_PORT, MOTOR_nsleep4_PIN);

    // [注释已恢复，见文件头部的完整说明]
    DL_GPIO_clearPins(MOTOR_PH1_PORT, MOTOR_PH1_PIN);  // Step2: PH=0, 全刹车状态 (防止上电抖动)
    DL_GPIO_clearPins(MOTOR_PH2_PORT, MOTOR_PH2_PIN);
    DL_GPIO_clearPins(MOTOR_PH3_PORT, MOTOR_PH3_PIN);
    DL_GPIO_clearPins(MOTOR_PH4_PORT, MOTOR_PH4_PIN);

    // [注释已恢复，见文件头部的完整说明]
    // [注释已恢复，见文件头部的完整说明]
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, 1000, DL_TIMER_CC_0_INDEX);  // Step3: CCR=1000→占空比0% (周期1000)
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, 1000, DL_TIMER_CC_1_INDEX);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, 1000, DL_TIMER_CC_2_INDEX);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, 1000, DL_TIMER_CC_3_INDEX);

    // [注释已恢复，见文件头部的完整说明]
    DL_TimerG_startCounter(MOTOR_PWM_INST);  // Step4: 启动TIMG12计数器
}
/* [注释已恢复，见文件头部的完整说明] */
/* */
/**
 * set_motor_speed() — 设置四路电机转速和方向
 *
 * 功能: 根据 PID 输出的有符号 PWM 值, 控制电机方向和占空比。
 *
 * 参数:
 *   m1 — 左前轮 PWM [-1000,+1000]
 *        >0: 正转, 占空比=(1000-m1)/1000
 *        <0: 反转, 占空比=(1000+m1)/1000
 *        =0: 刹车, CCR=999
 *   m2 — 右前轮 PWM (同上)
 *   M1, M2 — 后轮 PWM (当前传0, 未使用)
 *
 * 调用时机: 由 Pid_Speed() 每个控制周期调用
 *
 * 硬件: PHx 宏控制 DRV8870 方向, DL_TimerG_setCaptureCompareValue 设置 CCR
 *   CCR=1000 → 0%, CCR=0 → 100%, CCR=999 → 刹车
 *   四路共用 TIMG12, 周期寄存器相同
 */
void set_motor_speed(int m1, int m2, int M1, int M2)
{
    if (m1 > 0)  // m1>0 → 正转: PH1高电平, 占空比=(1000-m1)/1000
        PH1(1), DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, 1000-m1, DL_TIMER_CC_0_INDEX);  // PH1=高(正转), CC0通道
	else if (m1 == 0)  // m1=0 → 刹车: PH3低电平, CCR=999(最大制动)
		PH3(0), DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, 999, DL_TIMER_CC_0_INDEX);  // PH3=低(刹车)
    else
        PH1(0), DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, 1000+m1, DL_TIMER_CC_0_INDEX);  // m1<0 → 反转: PH1低, CCR=1000-|m1|

    if (m2 > 0)  // 右前轮正转 — 注意: 右轮PH逻辑与左轮相反 (机械安装方向)
        PH2(0), DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, 1000-m2, DL_TIMER_CC_1_INDEX);  // PH2=低(正转), CC1通道
	else if (m2 == 0)  // 右前轮刹车
		PH2(0), DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, 999, DL_TIMER_CC_1_INDEX);  // 刹车
    else
        PH2(1), DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, 1000+m2, DL_TIMER_CC_1_INDEX);  // m2<0 → 反转: PH2高, CC1通道

    if (M1 > 0)  // 左后轮正转 (当前未使用, M1始终为0)
        PH3(0), DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, M1, DL_TIMER_CC_2_INDEX);  // CC2通道
    else
        PH3(1), DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, -M1, DL_TIMER_CC_2_INDEX);  // M1<0反转

    if (M2 > 0)  // 右后轮正转 (当前未使用)
        PH4(0), DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, M2, DL_TIMER_CC_3_INDEX);  // CC3通道
    else
        PH4(1), DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, -M2, DL_TIMER_CC_3_INDEX);  // M2<0反转
}



/**
 * get_motor_nfault() — 读取四路电机故障标志
 *
 * 功能: 读取 DRV8870 nFAULT 引脚, 打包为 1 字节。
 *       nFAULT=0 表示过流/过温故障。
 *
 * 返回值: bit0~bit3 对应 4 路电机, 1=正常, 0=故障
 *   全正常返回 0x0F
 *
 * 注意: DRV8870 nFAULT 为开漏输出, 需外部上拉。
 *   故障后需拉低 nSLEEP 清除。
 */
uint8_t get_motor_nfault(void)
{
    uint8_t nfault_data = 0;  // 故障状态字节, 低4位对应4路电机: 1=正常 0=故障

    if (NFAULT1 != 0)  // 左前轮 nFAULT引脚为高=正常 (DRV8870开漏输出, 需外部上拉)
        nfault_data |= 1 << 0;  // bit0=1 左前轮正常

    if (NFAULT2 != 0)  // 右前轮
        nfault_data |= 1 << 1;  // bit1=1 右前轮正常

    if (NFAULT3 != 0)  // 左后轮
        nfault_data |= 1 << 2;  // bit2=1 左后轮正常

    if (NFAULT4 != 0)  // 右后轮
        nfault_data |= 1 << 3;  // bit3=1 右后轮正常

    return nfault_data;  // 返回值: 0x0F=全部正常, 某位为0=对应电机故障
}


/* [注释已恢复，见文件头部的完整说明] */
/**
 * get_motor_qei_cnt() — 读取编码器计数值
 *
 * 功能: 返回左右电机 QEI 编码器当前计数值。
 *
 * 参数:
 *   cnt — uint32_t[2], cnt[0]=左轮, cnt[1]=右轮
 *
 * 调用时机: motor_control_update() 每个控制周期调用
 *
 * 速度计算: 调用者做差分 speed = cnt_curr - cnt_last
 *
 * 硬件:
 *   QEI_M0_INST: 左轮编码器 Timer
 *   QEI_M1_INST: 右轮编码器 Timer
 */
void get_motor_qei_cnt(uint32_t *cnt)
{
    *cnt     = DL_Timer_getTimerCount(QEI_M0_INST);  // cnt[0]=左轮编码器计数值 (QEI_M0_INST)
    *(cnt+1) = DL_Timer_getTimerCount(QEI_M1_INST);  // cnt[1]=右轮编码器计数值 (QEI_M1_INST)
}

/* [注释已恢复，见文件头部的完整说明] */
// [注释已恢复，见文件头部的完整说明]
// [注释已恢复，见文件头部的完整说明]
static int32_t  enc_delta_M1 = 0;  // 左轮编码器增量 (预留, 当前未使用)
static int32_t  enc_delta_M2 = 0;  // 右轮编码器增量 (预留)
static uint32_t PWM_L_1 = 0;  // 左轮PWM中间值 (预留, 当前未使用)
static uint32_t PWM_L_2 = 0;  // 右轮PWM中间值 (预留)
static int pwm1 = 0;  // 左前轮PWM输出 [文件级全局变量] motor_control_update计算, Pid_Speed写入硬件
static int pwm2 = 0;  // 右前轮PWM输出
static float filtered_speed_M1 = 0;  // 左轮滤波后速度: filtered=0.3*raw+0.7*prev (一阶IIR低通, alpha=0.3)
static float filtered_speed_M2 = 0;  // 右轮滤波后速度
	static float ramp_setpoint_M1 = 0;
	static float ramp_setpoint_M2 = 0;
	static uint8_t ramp_active = 0;
	static uint8_t ramp_done = 0;

/**
 * motor_speed_pid_init() — 速度 PID 初始化 + 编码器基准校准
 *
 * 功能:
 *   1. 读取当前编码器计数值作为基准 (last_cnt_Mx)
 *   2. 清零所有 Speed_Pid 状态 (SumError/Error 等)
 *   3. 重置启动斜坡 (ramp_active=0, ramp_done=0)
 *   4. 设置 motor_pid_init_flag=1, 允许 motor_control_update 运行
 *
 * 调用时机:
 *   - 系统初始化时调用一次 (Pid_Init 之后)
 *   - 每次停车后重新启动时调用 (重置斜坡)
 *
 * 注意:
 *   - 必须在 Pid_Init() 之后调用
 *   - 调用前确保电机静止 (编码器基准在此时采样)
 */
void motor_speed_pid_init(void)
{
    uint32_t cnt[2];

    get_motor_qei_cnt(cnt);  // Step1: 读取当前编码器计数值作为基准

    last_cnt_M1 = cnt[0];  // Step2: 保存左轮编码器基准值 (后续差分测速的起点)
    last_cnt_M2 = cnt[1];  // 保存右轮基准值


    for(int i=0;i<4;i++)
    {
        Speed_Pid[i].SetPoint = 0;    // 清零目标值 (由main.c/control.c后续设置)
        Speed_Pid[i].ActualValue = 0;  // 清零输出值
        Speed_Pid[i].SumError = 0;    // 清零积分累加器 (避免残留积分造成启动冲击)
        Speed_Pid[i].LastError = 0;   // 清零上次误差 (D项计算用)
        Speed_Pid[i].PrevError = 0;   // 清零上上次误差 (增量式PID用)
    }

    motor_pid_init_flag = 1;  // Step3: 设置就绪标志, 允许 motor_control_update() 开始运行

	ramp_active = 0;  // Step4: 重置斜坡 — 下次启动从0开始ramp
	ramp_done = 0;     // 重置斜坡完成标志
}


/* [注释已恢复，见文件头部的完整说明] */
/**
 * motor_control_update() — 速度 PID 主循环 (每控制周期调用)
 *
 * 流水线:
 *   编码器 → 差分测速 → 低通滤波(alpha=0.3) → 启动斜坡(rate=0.25)
 *   → 暂存 SetPoint → 替换为 ramp 值 → Pid_control(位置式) → 恢复 SetPoint
 *   → Pid_OutLimit(+-800) → 更新 pwm1/pwm2 → printf 调试输出
 *
 * 启动斜坡:
 *   ramp += (target - ramp) * 0.25, 从 0 指数逼近
 *   到达后 ramp_done=1 永久锁定, control.c 转向差速即时生效
 *
 * 调参入口:
 *   PID:      pid.c PID_Value_Speed[0]/[1]
 *   滤波:     alpha=0.3 (filtered = 0.3*raw + 0.7*prev)
 *   斜坡速率: rate=0.25
 *   PWM上限:  SPEED_PWM_MAX=800
 *
 * 输出:
 *   pwm1/pwm2 全局变量, 由 Pid_Speed() 写入硬件
 *   printf: "SetPoint,speed_M1,speed_M2"
 *
 * 注意:
 *   - 计算与输出分离: 本函数计算, Pid_Speed() 写硬件
 *   - 控制频率 10~20ms, 改频率需同步调 alpha
 */
void motor_control_update(void)
{
    uint32_t cnt[2];


    if(motor_pid_init_flag==0)  // PID未初始化 → 直接返回, 不执行任何控制
        return;  // 提前退出, 等待 motor_speed_pid_init() 设置标志位


    get_motor_qei_cnt(cnt);  // Step1: 读取当前编码器计数值作为基准


    // [2/9] 差分测速: 原始速度 = 当前计数 - 上次计数
    speed_M1 = (int16_t)(cnt[0]-last_cnt_M1);  // 左轮原始速度 (编码器计数/控制周期)
    speed_M2 = -(int16_t)(cnt[1]-last_cnt_M2);  // 右轮 (负号: 右轮编码器方向与左轮相反)


    last_cnt_M1 = cnt[0];  // Step2: 保存左轮编码器基准值 (后续差分测速的起点)
    last_cnt_M2 = cnt[1];  // 保存右轮基准值
	// [3/9] 一阶低通滤波: 平滑编码器量化噪声, 让D项真正起到阻尼作用 (alpha=0.3)
	filtered_speed_M1 = 0.3f * speed_M1 + 0.7f * filtered_speed_M1;  // 左轮: 30%新值+70%旧值
	filtered_speed_M2 = 0.3f * speed_M2 + 0.7f * filtered_speed_M2;  // 右轮

	// [4/9] 启动斜坡: ramp从0指数逼近目标, 抑制冷启动尖峰 (rate=0.25)
	if (!ramp_done) {  // 斜坡未完成 → 用ramp值替代真实SetPoint
	if (!ramp_active) {  // 首次进入 → 初始化ramp从0开始
		ramp_setpoint_M1 = 0;  // 左轮ramp起始值为0
		ramp_setpoint_M2 = 0;  // 右轮ramp起始值为0
		ramp_active = 1;  // 标记斜坡已激活
	}
	ramp_setpoint_M1 += (Speed_Pid[0].SetPoint - ramp_setpoint_M1) * 0.50f;
	ramp_setpoint_M2 += (Speed_Pid[1].SetPoint - ramp_setpoint_M2) * 0.50f;
	if (Speed_Pid[0].SetPoint - ramp_setpoint_M1 < 0.5f) ramp_setpoint_M1 = Speed_Pid[0].SetPoint;
	if (Speed_Pid[1].SetPoint - ramp_setpoint_M2 < 0.5f) ramp_setpoint_M2 = Speed_Pid[1].SetPoint;
	if (ramp_setpoint_M1 > Speed_Pid[0].SetPoint - 1.0f && ramp_setpoint_M2 > Speed_Pid[1].SetPoint - 1.0f) ramp_done = 1;
	}

	// [5/9] 备份原始SetPoint (control.c 设置的转弯差速), 斜坡期间用ramp值替代
	float orig_sp1 = Speed_Pid[0].SetPoint;  // 备份左轮原始目标
	float orig_sp2 = Speed_Pid[1].SetPoint;  // 备份右轮原始目标
	if (!ramp_done) {  // 斜坡未完成 → 用ramp值替代真实SetPoint
	Speed_Pid[0].SetPoint = ramp_setpoint_M1;
	Speed_Pid[1].SetPoint = ramp_setpoint_M2;
	}

	// [6/9] 位置式PID计算 (用滤波后的速度作为反馈)
	Pid_control(&Speed_Pid[0], (float)filtered_speed_M1, 0);  // 左轮PID: Mode=0(位置式)
	Pid_control(&Speed_Pid[1], (float)filtered_speed_M2, 0);  // 右轮PID

	// [7/9] 恢复原始SetPoint — control.c的转向差速不受ramp影响
	Speed_Pid[0].SetPoint = orig_sp1;  // 恢复左轮原始目标 (来自control.c或main.c)
	Speed_Pid[1].SetPoint = orig_sp2;  // 恢复右轮原始目标



    // [8/9] 输出限幅: 将PID输出钳位到 ±SPEED_PWM_MAX(800)

    pwm1 = Pid_OutLimit(  // 左轮限幅
        &Speed_Pid[0],
        SPEED_PWM_MAX);

    pwm2 = Pid_OutLimit(  // 右轮限幅
        &Speed_Pid[1],
        SPEED_PWM_MAX);


//	printf("%d,%d\r\n",
//	pwm1,
//	pwm2);

//	printf("%d,%d,%d\r\n",
//	(int)Speed_Pid[0].SetPoint,
//	speed_M1,
//	speed_M2);
}

/**
 * print_date() — 调试打印 (编码器增量)
 *
 * 功能: printf 输出 enc_delta_M1/M2。
 *   注意: enc_delta 当前未被 motor_control_update 更新,
 *   实际调试数据由 motor_control_update 中的 printf 输出。
 *   本函数保留用于离线调试。
 */
void print_date()
{
			printf("%d,%d\r\n",
			enc_delta_M1,
			enc_delta_M2);
//			PWM_L_1,
//			PWM_L_2
	
	delay_ms(200);
}
/**
 * Pid_Speed() — 将 PID 计算结果写入 PWM 硬件
 *
 * 功能: 读取 pwm1/pwm2 全局变量, 调用 set_motor_speed() 写入 TIMG12。
 *       后轮 M1/M2 当前未使用 (传0)。
 *
 * 调用时机: 紧接 motor_control_update() 之后 (同周期)
 *
 * 输入: pwm1(左前轮), pwm2(右前轮), 范围 [-800,+800]
 *
 * 注意: 必须与 motor_control_update() 同周期配对调用
 */
void Pid_Speed()
{
	    set_motor_speed(  // 将PID计算结果写入PWM硬件
        pwm1,  // 左前轮 PWM [-800,+800]
        pwm2,  // 右前轮 PWM
        0,     // 左后轮 — 当前未使用
        0);    // 右后轮 — 当前未使用
}
/*
 * ===========================================================================
 * Real Speed Calculation -- encoder pulses to physical speed(cm/s) / distance(cm)
 * ===========================================================================
 *
 * Hardware: MG513P30_12V, gear ratio 1:30, encoder 13 PPR, wheel dia 65mm
 * QEI: DL_TIMER_QEI_MODE_2_INPUT = 4x resolution, 4 counts per encoder pulse
 *
 * Formula:
 *   wheel circumference = PI * 6.5 = 20.42 cm
 *   counts per wheel rev = 13 PPR * 4(QEI) * 30(gear) = 1560
 *   cm per count = 20.42 / 1560 ~ 0.01309 cm/count
 *   speed(cm/s) = cm_per_count / 0.128s ~ 0.102 cm/s per count/period
 *
 * Does NOT modify existing PID logic.
 * ===========================================================================
 */

/*
 * Get distance traveled this period (cm)
 * motor_id: 0=left wheel, 1=right wheel
 */
float get_motor_distance_cm(int motor_id)
{
    float filtered;
    if (motor_id == 0) {
        filtered = filtered_speed_M1;
    } else {
        filtered = filtered_speed_M2;
    }
    return filtered * WHEEL_CIRCUMFERENCE_CM
           / ENCODER_PULSES_PER_WHEEL_REV;
}

/*
 * Get real speed (cm/s)
 * motor_id: 0=left wheel, 1=right wheel
 */
float get_motor_speed_cm_s(int motor_id)
{
    float filtered;
    if (motor_id == 0) {
        filtered = filtered_speed_M1;
    } else {
        filtered = filtered_speed_M2;
    }
    return filtered * WHEEL_CIRCUMFERENCE_CM
           / (ENCODER_PULSES_PER_WHEEL_REV * SPEED_PERIOD_SEC);
}

/*
 * Convert real speed (cm/s) to encoder counts per 128ms period.
 * Use this to translate human-readable speed to PID SetPoint units.
 * Formula: counts = cm_s * pulses_per_wheel_rev * period / circumference
 *                = cm_s * 1560 * 0.128 / 20.42
 *                = cm_s * 9.78
 */
float cm_s_to_speed_counts(float cm_s)
{
    return cm_s * ENCODER_PULSES_PER_WHEEL_REV * SPEED_PERIOD_SEC
           / WHEEL_CIRCUMFERENCE_CM;
}
