#include "pid.h"

// [注释已恢复，见文件头部的完整说明]
// [注释已恢复，见文件头部的完整说明]
PID_TypeDef Position_position_Pid[4];  // 位置控制-位置环(外环) PID 实例, [0..3]对应4个电机
PID_TypeDef	Position_speed_Pid[4];      // 位置控制-速度环(内环) PID 实例, [0..3]对应4个电机
PID_TypeDef Speed_Pid[4];               // 独立速度控制 PID 实例, [0]左前 [1]右前 [2]左后 [3]右后

// [注释已恢复，见文件头部的完整说明]
// [注释已恢复，见文件头部的完整说明]
// [注释已恢复，见文件头部的完整说明]
// [注释已恢复，见文件头部的完整说明]
// [注释已恢复，见文件头部的完整说明]
// [注释已恢复，见文件头部的完整说明]
#if	POSITION_Position_PID_TYPE == 0  // 位置式PID: 直接计算绝对输出量
p_i_d_Value_TypeDef PID_Value_Position_position[4] = {{ 0.48, 0.0, 0.1 },  // Kp=0.48, Ki=0(无积分), Kd=0.1, 
													  { 0.48, 0.0, 0.1 },  // Kp=0.48, Ki=0(无积分), Kd=0.1,
													  { 0.48, 0.0, 0.1 },  // Kp=0.48, Ki=0(无积分), Kd=0.1,
													  { 0.48, 0.0, 0.1 }};  // Kp=0.48, Ki=0(无积分), Kd=0.1	//
#else                                  // 增量式PID: 在上一拍输出基础上叠加增量
p_i_d_Value_TypeDef PID_Value_Position_position[4] = {{ 0.0 , 0.0 , 0.0 },
													  { 0.0 , 0.0 , 0.0 },
													  { 0.0 , 0.0 , 0.0 },
													  { 0.0 , 0.0 , 0.0 }};	//
#endif
// [注释已恢复，见文件头部的完整说明]
// [注释已恢复，见文件头部的完整说明]
// [注释已恢复，见文件头部的完整说明]
// [注释已恢复，见文件头部的完整说明]
// [注释已恢复，见文件头部的完整说明]
// [注释已恢复，见文件头部的完整说明]
#if	POSITION_Speed_PID_TYPE == 0  // 位置式: 位置控制内环速度PID
p_i_d_Value_TypeDef PID_Value_Position_speed[4] = {{ 0 , 0 , 0.0 },
												   { 0 , 0 , 0.0 },
												   { 0 , 0 , 0.0 },
												   { 0 , 0 , 0.0 }};	//
#else                                  // 增量式PID: 在上一拍输出基础上叠加增量
p_i_d_Value_TypeDef PID_Value_Position_speed[4] = {{ 0.0 , 0.0 , 0.0 },
												   { 0.0 , 0.0 , 0.0 },
												   { 0.0 , 0.0 , 0.0 },
												   { 0.0 , 0.0 , 0.0 }};	//
#endif


// [注释已恢复，见文件头部的完整说明]
// [注释已恢复，见文件头部的完整说明]
// [注释已恢复，见文件头部的完整说明]
// [注释已恢复，见文件头部的完整说明]
// [注释已恢复，见文件头部的完整说明]
// [注释已恢复，见文件头部的完整说明]
/*
 * ===========================================================================
 * 速度环 PID 参数 (Speed_PID_Type == 0, 位置式PID)
 * ===========================================================================
 * 
 * 适用范围: MSPM0G3519 四轮小车，速度 0~700 (编码器计数/控制周期)
 * PWM 上限: 800 (PWM 周期 1000)
 * 控制周期: 由 motor_control_update() 调用频率决定
 * 
 * 调参历程 (目标速度 100 实测数据):
 *   原始: Kp=1.3, Ki=0.255, Kd=0.1  → 稳态 46~104, 持续振荡, 18+周期勉强稳定
 *   最终: Kp=0.85, Ki=0.25, Kd=0.60 → 稳态 99~101, 7周期稳定, 过冲 15% 以内
 * 
 * 各参数作用:
 *   Kp (Proportion): 响应速度。太大→过冲振荡，太小→提速慢。
 *   Ki (Integral):   消除稳态误差。太大→积分饱和振荡，太小→无法到达目标。
 *   Kd (Derivative): 阻尼项，抑制速度突变。编码器噪声大时不宜过大 (0.3~0.6)。
 *   
 * 本系统编码器速度信号噪声较大，核心策略:
 *   1. motor.c 中低通滤波平滑速度信号
 *   2. Kd 控制在 0.5~0.6，既抑制振荡又不放大噪声
 *   3. motor.c 中启动斜坡(ramp)分担了抑制冷启动尖峰的任务
 * 
 * 各电机对应关系:
 *   PID_Value_Speed[0] = 左前轮 L1: Kp=0.85, Ki=0.25, Kd=0.60
 *   PID_Value_Speed[1] = 右前轮 R1: Kp=0.80, Ki=0.25, Kd=0.55
 *   PID_Value_Speed[2] = 左后轮 L2: 未使用 (保留原始参数)
 *   PID_Value_Speed[3] = 右后轮 R2: 未使用 (保留原始参数)
 * 
 * 如需重新调参，建议流程:
 *   1. Ki=0, Kd=0, 逐步增大 Kp 直到速度能到达目标但略有振荡
 *   2. 加入 Ki, 从小到大, 直到稳态误差消除
 *   3. 加入 Kd, 从小到适中, 消除残余振荡
 *   4. 不同速度档位(100/200/500/700)分别验证
 *   5. 高速时如达不到目标, 检查 SumError 上限 (见 Pid_control)
 * ===========================================================================
 */
#if	SPEED_PID_TYPE == 0  // 位置式PID (当前使用): 输出 = Kp*Err + Ki*SumErr + Kd*ΔErr
p_i_d_Value_TypeDef PID_Value_Speed[4] = {
										  { 1.20, 0.50, 0.40 },  // [0] 左前轮 L1: Kp=0.85  Ki=0.25  Kd=0.60
									      { 1.15, 0.50, 0.40 },  // [1] 右前轮 R1: Kp=0.80  Ki=0.25  Kd=0.55 (右轮机械特性不同, 参数略低)
										  { 1.18888, 0.2555, 0.1 }, // [2] 左后轮 L2: 未使用, 保留原始参数
										  { 1.18888, 0.2555, 0.1 }}; // [3] 右后轮 R2: 未使用,保留原始参数
#else                      // 增量式PID: 输出 += Kp*ΔErr + Ki*Err + Kd*(ΔErr-ΔErr_prev)
p_i_d_Value_TypeDef PID_Value_Speed[4] = {{ 0.0 , 0.0 , 0.0 },
										  { 0.0 , 0.0 , 0.0 },
										  { 0.0 , 0.0 , 0.0 },
										  { 0.0 , 0.0 , 3 }};	//
#endif

/**************************************************************************
 * 函数: Pid_Init()
 * 
 * 功能: PID 结构体初始化，将 Kp/Ki/Kd 加载到 PID 实例，清零所有状态变量。
 * 
 * 调用时机:
 *   - 系统启动时调用一次 (main.c)
 *   - 切换控制模式后调用 (如从位置控制切换到速度控制)
 *   - 注意: 调用后会清零 SumError/LastError/PrevError，相当于"重置PID记忆"
 * 
 * 参数:
 *   PID        — 指向 PID_TypeDef 实例的指针 (如 &Speed_Pid[0])
 *   pid_Value  — 指向 p_i_d_Value_TypeDef 的指针 (如 &L1_PID_Value_Speed)
 *                 包含 KP/KI/KD 三个 float 值
 * 
 * 返回值: 无
 * 
 * 配置:
 *   KP/KI/KD 的值在 pid.c 顶部 PID_Value_* 数组中静态定义
 *   位置式/增量式由 POSITION_PID_TYPE / SPEED_PID_TYPE 宏控制
 * 
 * 使用示例:
 *   Pid_Init(&Speed_Pid[0], &L1_PID_Value_Speed);  // 左轮速度环
 *   Pid_Init(&Speed_Pid[1], &R1_PID_Value_Speed);  // 右轮速度环
 **************************************************************************/
void Pid_Init( PID_TypeDef* PID, p_i_d_Value_TypeDef* pid_Value )
{
	PID->SetPoint   = 0.0;           // 目标值清零, 由 main.c/control.c 后续设置
	PID->ActualValue = 0.0;          // 本次输出值清零
	PID->SumError   = 0.0;           // 积分累加器清零 (避免残留积分造成启动冲击)
	PID->Error       = 0.0;           // 本次误差清零
	PID->LastError   = 0.0;           // 上次误差清零 (D项计算用)
	PID->PrevError   = 0.0;           // 上上次误差清零 (增量式PID用)
	PID->Proportion  = pid_Value->KP;  // 从参数结构体加载 Kp
	PID->Integral    = pid_Value->KI;  // 从参数结构体加载 Ki
	PID->Derivative  = pid_Value->KD;  // 从参数结构体加载 Kd
}

/**************************************************************************
 * 函数: Pid_control()
 * 
 * 功能: PID 闭环控制核心算法，根据目标值(SetPoint)和反馈值计算控制输出。
 * 
 * 调用时机:
 *   - 每个控制周期调用一次 (由 motor_control_update() 调用)
 *   - 调用前需确保 SetPoint 已更新 (由 main.c/control.c 实时设置)
 * 
 * 参数:
 *   PID            — PID 实例指针
 *   Feedback_value — 反馈值/实际测量值 (float, 编码器速度)
 *   Mode           — 算法模式: 0=位置式PID, 1=增量式PID
 * 
 * 返回值: PID 计算输出值 (int32_t, 未限幅)
 * 
 * 算法详解 (Mode 0, 位置式):
 *   Error = SetPoint - Feedback_value
 *   SumError += Error                    // 积分累加
 *   SumError = clamp(SumError, ±4000)    // 积分限幅, 防止饱和
 *   Output = Kp*Error + Ki*SumError + Kd*(Error - LastError)
 * 
 * 算法详解 (Mode 1, 增量式):
 *   Output += Kp*(Error - LastError) + Ki*Error + Kd*(Error - 2*LastError + PrevError)
 *   (增量式无 SumError 累积, 天然抗积分饱和)
 * 
 * 积分限幅 (±4000) 说明:
 *   Ki=0.25 时 I 项最大贡献 = 0.25*4000 = 1000 PWM
 *   但 Pid_OutLimit 将输出钳位在 ±800, 实际不会超过
 *   低速时 SumError 远小于 4000, 不受影响
 *   高速时 (>500) 需更大的 SumError 才能提供足够 PWM 推力
 *   如调整 Ki, 需同步检查此限幅是否合适:
 *     I_max = Ki × 4000, 应 ≥ PWM_MAX(800) 以保证高速可达
 * 
 * 注意事项:
 *   - 本函数不修改 SetPoint, 调用者可随时更新目标值
 *   - 本函数不做输出限幅, 需调用 Pid_OutLimit() 限幅
 *   - 位置式 PID 对积分饱和敏感, 已通过 SumError 限幅缓解
 *   - 编码器速度噪声大时, Kd 不宜过大 (建议 <0.6)
 * 
 * 使用示例:
 *   // 速度环 (位置式)
 *   Pid_control(&Speed_Pid[0], (float)filtered_speed_M1, 0);
 *   Pid_OutLimit(&Speed_Pid[0], 800);
 * 
 *   // 转向环 (增量式)
 *   Pid_control(&Grayscale_Direction_Pid, offset, 1);
 *   Pid_OutLimit(&Grayscale_Direction_Pid, 100);
 **************************************************************************/
int32_t Pid_control( PID_TypeDef* PID, float Feedback_value, unsigned char Mode )
{
	PID->Error = PID->SetPoint - Feedback_value;  // Step1: 计算本次误差 (目标值 - 实际反馈值)
	if (Mode == 0)  // 位置式PID — 速度环用此模式, 直接计算绝对输出
	{
		PID->SumError += PID->Error;  // Step2a: 积分累加 (误差持续存在→积分持续增长→消除稳态误差)
			if ( PID->SumError > 4000)// 积分限幅 ±4000: Ki=0.25时I项最大贡献=1000PWM(输出限幅800兜底)。高速需大SumError提供足够PWM，低速不触及此上限。
		{
			PID->SumError = 4000;
		}
		else if (PID->SumError < -4000)
		{
			PID->SumError = -4000;
		}
		
		PID->ActualValue = PID->Proportion * PID->Error
							+ PID->Integral * PID->SumError
							+ PID->Derivative * (PID->Error - PID->LastError);
		PID->LastError = PID->Error;  // Step4b: 滚动更新上次误差
	}
	else  // 增量式PID — 转向环用此模式, 输出平滑无积分饱和
	{
		PID->ActualValue += PID->Proportion * (PID->Error - PID->LastError)
							+ PID->Integral * PID->Error
							+ PID->Derivative * (PID->Error - 2 * PID->LastError + PID->PrevError);
		PID->PrevError = PID->LastError;  // Step3b: 滚动更新上上次误差
		PID->LastError = PID->Error;  // Step4b: 滚动更新上次误差
	}
//	printf("  value %f\r\n",PID->ActualValue);
	return PID->ActualValue;  // 返回未限幅的原始输出, 由调用者做 Pid_OutLimit
}

/**************************************************************************
 * 函数: Pid_OutLimit()
 * 
 * 功能: 对 PID 计算结果进行输出限幅，防止 PWM 超出硬件范围。
 *       直接修改 PID->ActualValue, 同时返回限幅后的值。
 * 
 * 调用时机:
 *   - 紧接在 Pid_control() 之后调用
 *   - 不同控制环使用不同的 limit 值 (速度环 800, 转向环 100)
 * 
 * 参数:
 *   PID   — PID 实例指针 (读取/修改 PID->ActualValue)
 *   limit — 限幅绝对值, 输出范围 [-limit, +limit]
 * 
 * 返回值: 限幅后的 PID 输出值 (int32_t)
 * 
 * 配置:
 *   速度环: limit = SPEED_PWM_MAX (800), PWM 周期 1000
 *   转向环: limit = 100 (转向微调范围)
 * 
 * 注意事项:
 *   - 此函数直接修改 PID 结构体内部的 ActualValue
 *   - 返回值同时赋值给外部变量 (pwm1/pwm2)
 *   - limit 值不应超过 PWM 硬件上限 1000
 * 
 * 使用示例:
 *   pwm1 = Pid_OutLimit(&Speed_Pid[0], SPEED_PWM_MAX);
 *   if (pwm1 == SPEED_PWM_MAX) 
 **************************************************************************/
int32_t Pid_OutLimit( PID_TypeDef* PID, int32_t limit)
{
	if ( PID->ActualValue > limit)  // 正向超限 → 钳位到 +limit
	{
		PID->ActualValue = limit;     // 正向钳位
	}
	else if (PID->ActualValue < -limit)  // 反向超限 → 钳位到 -limit
	{
		PID->ActualValue = -limit;          // 反向钳位
	}
	return PID->ActualValue;  // 返回未限幅的原始输出, 由调用者做 Pid_OutLimit
}

