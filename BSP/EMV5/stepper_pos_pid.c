#include "bsp.h"
#include "yuntai.h"
#include "stepper_pos_pid.h"

#define STEPPER_DT     0.01f      /* control period: 10 ms */

/* read encoder → update actual_pulses. returns true on success */
bool stepper_pos_pid_sync_encoder(StepperPosPID_t *ctrl)
{
    uart3_count = 0;
    Emm_V5_Read_Sys_Params(ctrl->addr, S_CPOS);

    /* wait up to 15ms for 8-byte reply */
    uint32_t t0 = system_time_get_tick_ms();
    while (uart3_count < 8 && (system_time_get_tick_ms() - t0) < 15U);

    for (uint8_t i = 0; (i + 7) < uart3_count; i++) {
        if (uart3_rec_data[i]   == ctrl->addr &&
            uart3_rec_data[i+1] == 0x36 &&
            uart3_rec_data[i+7] == 0x6B) {
            uint8_t  dir = uart3_rec_data[i+2];
            uint32_t raw = ((uint32_t)uart3_rec_data[i+3] << 24)
                         | ((uint32_t)uart3_rec_data[i+4] << 16)
                         | ((uint32_t)uart3_rec_data[i+5] <<  8)
                         |  (uint32_t)uart3_rec_data[i+6];
            float deg = (float)raw * 0.005f;
            if (dir) deg = -deg;
            /* deg * ppr/360 = pulses. ppr/360 is pulses-per-degree */
            ctrl->actual_pulses = (int32_t)(deg * (float)ctrl->ppr / 360.0f);
            uart3_count = 0;
            return true;
        }
    }
    uart3_count = 0;
    return false;
}

void stepper_pos_pid_init(StepperPosPID_t *ctrl, uint8_t addr, uint16_t ppr,
                          float kp, float ki, float kd)
{
    memset(ctrl, 0, sizeof(*ctrl));
    ctrl->addr          = addr;
    ctrl->ppr           = ppr;
    ctrl->deadzone      = 3;
    ctrl->max_speed     = 500;
    ctrl->min_speed     = 10;
    ctrl->acc           = 0;
    ctrl->is_done       = 1;
    ctrl->current_speed = 0.0f;
    ctrl->max_acc       = 3000.0f;   /* RPM/s — tune up for tracking, down for scan */
    ctrl->enc_sync_ms   = 0U;        /* sync only when stopped — see is_done block */
    ctrl->enc_sync_tick = 0U;

    ctrl->pid.Proportion = kp;
    ctrl->pid.Integral   = ki;
    ctrl->pid.Derivative = kd;
}

void stepper_pos_pid_set_target(StepperPosPID_t *ctrl, int32_t target_pulses)
{
    ctrl->target_pulses = target_pulses;
    ctrl->pid.SumError  = 0.0f;
    /* keep current_speed — prevents velocity jump when target updates continuously */
    ctrl->is_done = 0;
}

void stepper_pos_pid_update(StepperPosPID_t *ctrl)
{
    int32_t error = ctrl->target_pulses - ctrl->actual_pulses;
    float   max_delta = ctrl->max_acc * STEPPER_DT;

    if (ABS(error) <= ctrl->deadzone) {
        if (ctrl->current_speed > max_delta) {
            ctrl->current_speed -= max_delta;
        } else if (ctrl->current_speed < -max_delta) {
            ctrl->current_speed += max_delta;
        } else {
            ctrl->current_speed = 0.0f;
            Emm_V5_Stop_Now(ctrl->addr, 0);
            ctrl->pid.SumError = 0.0f;
            ctrl->is_done = 1;
            stepper_pos_pid_sync_encoder(ctrl);
            return;
        }
        uint8_t dir = (error >= 0) ? 0U : 1U;
        int32_t step = (int32_t)(fabsf(ctrl->current_speed) * (float)ctrl->ppr / 60.0f * STEPPER_DT);
        if (step < 1) step = 1;
        Emm_V5_Pos_Control(ctrl->addr, dir,
                           (uint16_t)fabsf(ctrl->current_speed),
                           ctrl->acc, (uint32_t)step, 0, false);   /* raF=0: relative */
        ctrl->actual_pulses += (error >= 0) ? step : -step;
        return;
    }

    /* --- PID: output = demanded speed (RPM) --- */
    ctrl->pid.SetPoint = (float)ctrl->target_pulses;
    int32_t out = Pid_control(&ctrl->pid, (float)ctrl->actual_pulses, 0);

    float speed_demand = fabsf((float)out);
    if (speed_demand < (float)ctrl->min_speed) speed_demand = (float)ctrl->min_speed;
    if (speed_demand > (float)ctrl->max_speed) speed_demand = (float)ctrl->max_speed;

    float delta = speed_demand - ctrl->current_speed;
    if (delta >  max_delta) delta =  max_delta;
    if (delta < -max_delta) delta = -max_delta;
    ctrl->current_speed += delta;

    uint8_t dir = (error > 0) ? 0U : 1U;

    /* raF=0 relative: send a small step each cycle.
       Each command is small (~10 pulses), so the motor executes it quickly
       and is ready for the next command — no queue buildup. */
    int32_t step = (int32_t)(ctrl->current_speed * (float)ctrl->ppr / 60.0f * STEPPER_DT);
    if (step < 1) step = 1;
    int32_t abs_err = ABS(error);
    if (step > abs_err) step = abs_err;

    Emm_V5_Pos_Control(ctrl->addr, dir, (uint16_t)ctrl->current_speed,
                       ctrl->acc, (uint32_t)step, 0, false);   /* raF=0: relative */

    ctrl->actual_pulses += (error > 0) ? step : -step;
    ctrl->is_done = 0;
}

uint8_t stepper_pos_pid_is_done(StepperPosPID_t *ctrl)
{
    return ctrl->is_done;
}

void stepper_pos_pid_reset_position(StepperPosPID_t *ctrl)
{
    ctrl->actual_pulses  = 0;
    ctrl->target_pulses  = 0;
    ctrl->current_speed  = 0.0f;
    ctrl->pid.SumError   = 0.0f;
    ctrl->pid.LastError  = 0.0f;
    ctrl->pid.PrevError  = 0.0f;
    ctrl->is_done = 1;
}
