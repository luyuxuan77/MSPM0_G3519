#ifndef STEPPER_POS_PID_H
#define STEPPER_POS_PID_H

#include <stdint.h>
#include "pid.h"

/*
 * stepper_pos_pid — MCU-side position loop for Emm V5.0 closed-loop stepper
 *
 * Feedback  : MCU-tracked pulse count (motor's internal encoder ensures accuracy)
 * Output    : speed (RPM) via Emm_V5_Pos_Control absolute-mode each 10 ms tick
 * Smoothing : output speed is rate-limited by max_acc to prevent velocity jumps
 *             when the target changes rapidly (visual tracking / trajectory)
 */

typedef struct {
    PID_TypeDef pid;
    int32_t  target_pulses;   /* desired absolute pulse position */
    int32_t  actual_pulses;   /* MCU-tracked position */
    uint8_t  addr;            /* motor UART address */
    int32_t  deadzone;        /* stop when |error| <= deadzone (pulses) */
    uint16_t max_speed;       /* RPM upper limit */
    uint16_t min_speed;       /* RPM lower limit */
    uint8_t  acc;             /* motor command acceleration byte */
    uint8_t  is_done;         /* 1 = within deadzone and stopped */
    uint16_t ppr;             /* pulses per revolution (motor-specific) */
    float    current_speed;   /* smoothed output speed (RPM), persists across cycles */
    float    max_acc;         /* acceleration limit (RPM/s), default 3000 */
    uint32_t enc_sync_tick;   /* last encoder sync timestamp */
    uint16_t enc_sync_ms;     /* how often to sync from encoder (ms), 0=disable */
} StepperPosPID_t;

/* Initialize controller. ppr: yaw=1764, pitch=1961 */
void stepper_pos_pid_init(StepperPosPID_t *ctrl, uint8_t addr, uint16_t ppr,
                          float kp, float ki, float kd);

/* Set new target. Does NOT reset current_speed — preserves velocity continuity. */
void stepper_pos_pid_set_target(StepperPosPID_t *ctrl, int32_t target_pulses);

/* Run one control cycle. Call every 10 ms. */
void stepper_pos_pid_update(StepperPosPID_t *ctrl);

/* Returns 1 when motor has stopped within deadzone of target. */
uint8_t stepper_pos_pid_is_done(StepperPosPID_t *ctrl);

/* Reset tracked position to zero (call after mechanical home). */
void stepper_pos_pid_reset_position(StepperPosPID_t *ctrl);

/* Sync actual_pulses from encoder. Returns true if read succeeded. */
bool stepper_pos_pid_sync_encoder(StepperPosPID_t *ctrl);

#endif /* STEPPER_POS_PID_H */
