#ifndef UART4_OPI_H
#define UART4_OPI_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"

void uart4_opi_init(void);
void uart4_opi_process(void);

void uart4_opi_send_task_control(uint8_t task_id, uint8_t action, int16_t setpoint_mm);
void opi_task_start(uint8_t task_id, int16_t setpoint_mm);
void opi_task_stop(uint8_t task_id, int16_t setpoint_mm);
uint8_t opi_consume_task3_done(void);
void uart4_opi_start_record(uint8_t task_id);
void uart4_opi_stop_record(uint8_t task_id);
void uart4_opi_send_sensor_data(int16_t accel_mg, int16_t angle_cdeg, int16_t wheel_accel);

extern volatile uint8_t  g_opi_last_ack_cmd;
extern volatile uint8_t  g_opi_last_ack_status;
extern volatile uint16_t g_opi_task_ack_count;
extern volatile uint16_t g_opi_sensor_ack_count;
extern volatile uint16_t g_opi_sensor_tx_count;
extern volatile uint8_t  g_opi_last_task_id;
extern volatile uint8_t  g_opi_last_task_action;
extern volatile int16_t  g_opi_last_accel_mg;
extern volatile int16_t  g_opi_last_angle_cdeg;
extern volatile int16_t  g_opi_last_wheel_accel;
extern volatile uint8_t  g_opi_done_flag;
extern volatile uint8_t  g_opi_done_task_id;
extern volatile uint8_t  g_opi_done_status;
extern volatile int16_t  g_target_dx;
extern volatile int16_t  g_target_dy;
extern volatile uint16_t g_target_dist;
extern volatile uint8_t  g_new_target;

#endif /* UART4_OPI_H */
