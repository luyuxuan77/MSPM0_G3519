#include "MOTOR/motor.h"

/*
 * ===========================================================================
 * Motor Speed Control Module — PID speed loop + low-pass filter + ramp start
 * ===========================================================================
 *
 * Ported from Car_sum project. Adapted for CORE_TEST V2.0 hardware:
 *   - 2 motors (M0, M1) on TIMG14 CC0/CC1
 *   - PWM period = 4000 (SysConfig), MOTOR_PWM_MAX = 3600
 *   - QEI on TIMG9 (M0) / TIMG8 (M1)
 *   - Direction GPIO: PA31 (M0), PB0 (M1)
 *   - No DRV8870 nSLEEP/nFAULT pins on this board
 *
 * ---------------------------------------------------------------------------
 * Signal Processing Chain
 * ---------------------------------------------------------------------------
 *
 *   Raw encoder counts
 *        |
 *   speed_Mx = cnt_curr - cnt_last      (differential speed measurement)
 *        |
 *   filtered_speed_Mx = 0.3*raw + 0.7*prev  (1st-order IIR low-pass, alpha=0.3)
 *        |
 *   ramp_setpoint replaces SetPoint     (cold-start ramp: 0 -> target)
 *        |
 *   Pid_control()                       (positional PID, Mode=0)
 *        |
 *   Pid_OutLimit(+/-3600)              (output clamp)
 *        |
 *   set_motor_speed(pwm0, pwm1)        (write TIMG14 CCR, set direction)
 *
 * ---------------------------------------------------------------------------
 * Low-Pass Filter (filtered_speed)
 * ---------------------------------------------------------------------------
 *
 * Formula: filtered = 0.3 * raw_speed + 0.7 * filtered_prev
 *
 * Encoder differential speed has quantization jumps (50+ counts between
 * adjacent periods). Feeding raw speed into PID causes D-term noise
 * amplification. First-order IIR low-pass smooths the signal.
 *
 * Filter coefficient selection:
 *   alpha=0.3 (new value weight): balances smoothness and response speed.
 *   Too small -> excessive lag. Too large -> insufficient filtering.
 *
 * ---------------------------------------------------------------------------
 * Startup Ramp (ramp_setpoint)
 * ---------------------------------------------------------------------------
 *
 * Problem: When starting from standstill, target speed jumps from 0 to
 * 100/200/900. PID output instantly saturates -> motor goes full speed ->
 * overshoot -> hard brake -> oscillation for 10-15 periods.
 *
 * Solution: Virtual ramp_setpoint replaces actual SetPoint, approaching
 * target exponentially each period:
 *     ramp += (target - ramp) * rate
 *     rate=0.20: ~9 periods to 90%, ~16 periods to 99%
 *
 * Lock mechanism:
 *   - On start, ramp begins from 0 toward target
 *   - When close to target (delta < 1.0), ramp_done=1 permanently
 *   - After lock, steering differential from external code takes effect immediately
 *   - motor_speed_pid_init() resets ramp_done=0 for next startup
 *
 * ---------------------------------------------------------------------------
 * Variable Scope (pwm0, pwm1)
 * ---------------------------------------------------------------------------
 *
 * pwm0/pwm1 are file-level static globals:
 *   - Computed by motor_control_update()
 *   - Read by Pid_Speed() to write to PWM hardware
 *   - This separation allows compute vs. output to run at different rates
 *
 * ---------------------------------------------------------------------------
 * Usage Guide
 * ---------------------------------------------------------------------------
 *
 * Initialization sequence:
 *   1. motor_init()               — configure GPIO/PWM hardware
 *   2. Pid_Init(&Speed_Pid[0/1], ...) — load PID parameters
 *   3. motor_speed_pid_init()     — init encoder baseline + reset ramp
 *   4. Speed_Pid[0/1].SetPoint = target_speed  — set by application
 *   5. Periodic calls:
 *        motor_control_update()   — read encoder -> filter -> PID -> update pwm0/pwm1
 *        Pid_Speed()              — write pwm0/pwm1 to PWM hardware
 *
 * To adjust control frequency:
 *   - Change the call intervals in TimerA1 ISR or main loop
 *   - Higher frequency -> lower speed resolution (fewer encoder counts) -> more noise
 *   - Lower frequency -> slower response, but more stable speed measurement
 *   - Current parameters tuned for ~128 ms control period
 * ===========================================================================
 */

/* ---- File-scope state ---- */
static uint32_t last_cnt_M0 = 0;        /* M0 encoder count at last sample */
static uint32_t last_cnt_M1 = 0;        /* M1 encoder count at last sample */

static int16_t speed_M0 = 0;            /* M0 raw speed (counts/period) */
static int16_t speed_M1 = 0;            /* M1 raw speed (sign-corrected) */

static uint8_t motor_pid_init_flag = 0; /* 0=skip control, 1=allow PID */

int pwm0 = 0;                           /* M0 PWM output [-3600, +3600] (extern) */
int pwm1 = 0;                           /* M1 PWM output [-3600, +3600] (extern) */

static float filtered_speed_M0 = 0;     /* M0 filtered speed (IIR low-pass) */
static float filtered_speed_M1 = 0;     /* M1 filtered speed */

static float ramp_setpoint_M0 = 0;      /* M0 ramp virtual setpoint */
static float ramp_setpoint_M1 = 0;      /* M1 ramp virtual setpoint */
static uint8_t ramp_active = 0;         /* 1 = ramp in progress */
static uint8_t ramp_done = 0;           /* 1 = ramp finished, permanently locked */

/* ===========================================================================
 * motor_init() — Motor hardware initialization
 * ===========================================================================
 *
 * Steps:
 *   1. Set direction pins low (brake state, prevents startup jerk)
 *   2. Set CCR to MOTOR_PWM_STOP (0% duty)
 *   3. Start PWM counter
 *
 * Must be called before motor_speed_pid_init().
 */
void motor_init(void)
{
    /* Step 1: Direction pins low */
    DL_GPIO_clearPins(Motor_DIR_M0_PH_PORT, Motor_DIR_M0_PH_PIN);
    DL_GPIO_clearPins(Motor_DIR_M1_PH_PORT, Motor_DIR_M1_PH_PIN);

    /* Step 2: CCR = stop value (0% duty) */
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, MOTOR_PWM_STOP,
                                      DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, MOTOR_PWM_STOP,
                                      DL_TIMER_CC_1_INDEX);

    /* Step 3: Start PWM counter (SysConfig set DL_TIMER_START, but just in case) */
    DL_TimerG_startCounter(MOTOR_PWM_INST);
}

/* ===========================================================================
 * set_motor_speed() — Set 2-motor speed and direction
 * ===========================================================================
 *
 * Parameters:
 *   m0 — M0 PWM [-4000, +4000]
 *        >0: forward, CCR = 4000 - m0
 *        =0: brake,  CCR = 3999
 *        <0: reverse, CCR = 4000 + m0 (i.e., 4000 - |m0|)
 *   m1 — M1 PWM (same logic)
 *
 * Hardware:
 *   M0_PH/M1_PH macros control direction (DRV8870 IN2 / PH pin).
 *   DL_TimerG_setCaptureCompareValue sets CCR duty cycle.
 *
 * PWM resolution: CCR = 0 -> 100%, CCR = 4000 -> 0%, CCR = 3999 -> brake.
 */
void set_motor_speed(int m0, int m1)
{
    /* ---- M0 (Motor 0, CC0 on PB28) ---- */
    if (m0 > 0)
        M0_PH(1), DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST,
                    MOTOR_PWM_PERIOD - m0, DL_TIMER_CC_0_INDEX);
    else if (m0 == 0)
        M0_PH(0), DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST,
                    MOTOR_PWM_STOP, DL_TIMER_CC_0_INDEX);
    else /* m0 < 0 */
        M0_PH(0), DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST,
                    MOTOR_PWM_PERIOD + m0, DL_TIMER_CC_0_INDEX);

    /* ---- M1 (Motor 1, CC1 on PB13) ---- */
    if (m1 > 0)
        M1_PH(1), DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST,
                    MOTOR_PWM_PERIOD - m1, DL_TIMER_CC_1_INDEX);
    else if (m1 == 0)
        M1_PH(0), DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST,
                    MOTOR_PWM_STOP, DL_TIMER_CC_1_INDEX);
    else /* m1 < 0 */
        M1_PH(0), DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST,
                    MOTOR_PWM_PERIOD + m1, DL_TIMER_CC_1_INDEX);
}

/* ===========================================================================
 * get_motor_qei_cnt() — Read encoder counts
 * ===========================================================================
 *
 * Returns current QEI counter values.
 *   cnt[0] = M0 (TIMG9), cnt[1] = M1 (TIMG8)
 *
 * Caller computes differential: speed = cnt_curr - cnt_last.
 */
void get_motor_qei_cnt(uint32_t *cnt)
{
    *cnt       = DL_Timer_getTimerCount(QEI_M0_INST);   /* M0 encoder */
    *(cnt + 1) = DL_Timer_getTimerCount(QEI_M1_INST);   /* M1 encoder */
}

/* ===========================================================================
 * motor_speed_pid_init() — Speed PID initialization + encoder baseline
 * ===========================================================================
 *
 * 1. Read current encoder counts as baseline
 * 2. Clear all Speed_Pid state (SumError, Error, etc.)
 * 3. Reset startup ramp (ramp_active=0, ramp_done=0)
 * 4. Set motor_pid_init_flag=1 to enable motor_control_update()
 *
 * Call: once at boot (after Pid_Init), and after each stop before re-start.
 */
void motor_speed_pid_init(void)
{
    uint32_t cnt[2];

    get_motor_qei_cnt(cnt);
    last_cnt_M0 = cnt[0];
    last_cnt_M1 = cnt[1];

    for (int i = 0; i < 4; i++)
    {
        Speed_Pid[i].SetPoint    = 0;
        Speed_Pid[i].ActualValue = 0;
        Speed_Pid[i].SumError    = 0;
        Speed_Pid[i].LastError   = 0;
        Speed_Pid[i].PrevError   = 0;
    }

    motor_pid_init_flag = 1;

    ramp_active = 0;
    ramp_done   = 0;

    /* Clear PWM outputs to prevent residual values from causing a jerk */
    pwm0 = 0;
    pwm1 = 0;
}

/* ===========================================================================
 * motor_control_update() — Speed PID main loop (call every control period)
 * ===========================================================================
 *
 * Pipeline:
 *   Encoder -> differential speed -> low-pass filter (alpha=0.3)
 *   -> startup ramp (rate=0.20) -> stash SetPoint -> replace with ramp
 *   -> Pid_control (positional) -> restore SetPoint -> Pid_OutLimit(+/-3600)
 *   -> update pwm0/pwm1
 *
 * Startup ramp:
 *   ramp += (target - ramp) * 0.20, exponential approach from 0
 *   Once arrived, ramp_done=1 permanently — external steering differentials
 *   take effect immediately.
 *
 * Output: pwm0/pwm1 global variables, consumed by Pid_Speed().
 */
void motor_control_update(void)
{
    uint32_t cnt[2];

    if (motor_pid_init_flag == 0)
        return;

    /* [1/7] Read encoder counts */
    get_motor_qei_cnt(cnt);

    /* [2/7] Differential speed measurement */
    speed_M0 = (int16_t)(cnt[0] - last_cnt_M0);
    speed_M1 = (int16_t)(cnt[1] - last_cnt_M1);
    /* NOTE: If M1 encoder polarity is opposite M0, negate speed_M1 here:
     *   speed_M1 = -(int16_t)(cnt[1] - last_cnt_M1);
     */

    last_cnt_M0 = cnt[0];
    last_cnt_M1 = cnt[1];

    /* [3/7] 1st-order low-pass filter: smooth encoder quantization noise */
    filtered_speed_M0 = 0.3f * speed_M0 + 0.7f * filtered_speed_M0;
    filtered_speed_M1 = 0.3f * speed_M1 + 0.7f * filtered_speed_M1;

    /* [4/7] Startup ramp: ramp from 0 to target, suppresses cold-start spikes */
    if (!ramp_done)
    {
        if (!ramp_active)
        {
            ramp_setpoint_M0 = 0;
            ramp_setpoint_M1 = 0;
            ramp_active = 1;
        }
        ramp_setpoint_M0 += (Speed_Pid[0].SetPoint - ramp_setpoint_M0) * 0.20f;
        ramp_setpoint_M1 += (Speed_Pid[1].SetPoint - ramp_setpoint_M1) * 0.20f;

        /* Snap to target when very close */
        if (Speed_Pid[0].SetPoint - ramp_setpoint_M0 < 0.5f)
            ramp_setpoint_M0 = Speed_Pid[0].SetPoint;
        if (Speed_Pid[1].SetPoint - ramp_setpoint_M1 < 0.5f)
            ramp_setpoint_M1 = Speed_Pid[1].SetPoint;

        if (ramp_setpoint_M0 >= Speed_Pid[0].SetPoint - 0.5f &&
            ramp_setpoint_M1 >= Speed_Pid[1].SetPoint - 0.5f)
            ramp_done = 1;
    }

    /* [5/7] Stash original SetPoint, substitute ramp during startup */
    float orig_sp0 = Speed_Pid[0].SetPoint;
    float orig_sp1 = Speed_Pid[1].SetPoint;

    if (!ramp_done)
    {
        Speed_Pid[0].SetPoint = ramp_setpoint_M0;
        Speed_Pid[1].SetPoint = ramp_setpoint_M1;
    }

    /* [6/7] Positional PID (using filtered speed as feedback) */
    Pid_control(&Speed_Pid[0], (float)filtered_speed_M0, 0);
    Pid_control(&Speed_Pid[1], (float)filtered_speed_M1, 0);

    /* Restore original SetPoint */
    Speed_Pid[0].SetPoint = orig_sp0;
    Speed_Pid[1].SetPoint = orig_sp1;

    /* [7/7] Output limiting */
    pwm0 = Pid_OutLimit(&Speed_Pid[0], MOTOR_PWM_MAX);
    pwm1 = Pid_OutLimit(&Speed_Pid[1], MOTOR_PWM_MAX);
}

/* ===========================================================================
 * Pid_Speed() — Write PID outputs to PWM hardware
 * ===========================================================================
 *
 * Reads pwm0/pwm1 globals, calls set_motor_speed() to write TIMG14 CCRs.
 * Call immediately after motor_control_update() (same or next period).
 *
 * Guard: returns immediately if motor_pid_init_flag is not set.
 */
void Pid_Speed(void)
{
    if (!motor_pid_init_flag)
        return;

    set_motor_speed(pwm0, pwm1);
}

/* ===========================================================================
 * Physical Speed / Distance Conversion
 * ===========================================================================
 *
 * Hardware: MG513P30_12V, gear ratio 1:30, encoder 13 PPR, wheel dia 65 mm
 * QEI: 4x resolution -> 4 counts per encoder pulse
 *
 * Counts per wheel revolution = 13 PPR * 4 (QEI) * 30 (gear) = 1560
 * Wheel circumference = PI * 6.5 = 20.42 cm
 * cm per count = 20.42 / 1560 ~ 0.01309 cm/count
 * speed (cm/s) = counts/period * 20.42 / (1560 * 0.128)
 *              = counts/period * 0.102
 * ===========================================================================
 */

/*
 * Get distance traveled this period (cm).
 * motor_id: 0 = M0, 1 = M1
 */
float get_motor_distance_cm(int motor_id)
{
    float filtered;
    if (motor_id == 0)
        filtered = filtered_speed_M0;
    else
        filtered = filtered_speed_M1;

    return filtered * WHEEL_CIRCUMFERENCE_CM
           / ENCODER_PULSES_PER_WHEEL_REV;
}

/*
 * Get real speed (cm/s).
 * motor_id: 0 = M0, 1 = M1
 */
float get_motor_speed_cm_s(int motor_id)
{
    float filtered;
    if (motor_id == 0)
        filtered = filtered_speed_M0;
    else
        filtered = filtered_speed_M1;

    return filtered * WHEEL_CIRCUMFERENCE_CM
           / (ENCODER_PULSES_PER_WHEEL_REV * SPEED_PERIOD_SEC);
}

/*
 * Convert real speed (cm/s) to encoder counts per 128 ms period.
 * Use this to set PID SetPoint in human-readable units.
 *
 * Formula: counts = cm_s * 1560 * 0.128 / 20.42 ~ cm_s * 9.78
 */
float cm_s_to_speed_counts(float cm_s)
{
    return cm_s * ENCODER_PULSES_PER_WHEEL_REV * SPEED_PERIOD_SEC
           / WHEEL_CIRCUMFERENCE_CM;
}
