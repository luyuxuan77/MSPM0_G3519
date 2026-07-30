#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>

#define FRAME_HEAD_H    0xAA
#define FRAME_HEAD_L    0x55
#define FRAME_TAIL      0xFE
#define MAX_PAYLOAD_LEN 32
#define MAX_FRAME_LEN   38

// 香橙派 -> M0
#define CMD_TARGET_POS    0x01
#define CMD_SHOOT         0x02
#define CMD_SET_TASK      0x03
#define CMD_PING          0x10
#define CMD_TASK_DONE     0x11

// M0 -> 香橙派 / 香橙派 -> M0 ACK
#define CMD_ACK           0x81
#define CMD_TASK_CONTROL  0x82
#define CMD_SENSOR_DATA   0x83
#define CMD_PONG          0x90

#define TASK_ACTION_STOP   0
#define TASK_ACTION_START  1

#define TASK_ID_MONITOR       1
#define TASK_ID_LAP_20S       2
#define TASK_ID_STATIC_SWING  3
#define TASK_ID_AB_BALANCE    4
#define TASK_ID_LAP_BALANCE   5
#define TASK_ID_LAP_SETPOINT  6

typedef enum {
    STATE_WAIT_HEAD_H = 0,
    STATE_WAIT_HEAD_L,
    STATE_WAIT_LEN,
    STATE_WAIT_DATA,
    STATE_WAIT_CHECKSUM,
    STATE_WAIT_TAIL
} parser_state_t;

typedef struct {
    uint8_t cmd;
    uint8_t payload[MAX_PAYLOAD_LEN];
    uint8_t payload_len;
} frame_t;

typedef struct {
    parser_state_t state;
    uint8_t buf[MAX_PAYLOAD_LEN + 2];
    uint8_t buf_idx;
    uint8_t expected_len;
} parser_ctx_t;

void parser_init(parser_ctx_t *ctx);
int parser_feed_byte(parser_ctx_t *ctx, uint8_t byte, frame_t *out);
uint8_t build_frame(uint8_t cmd, const uint8_t *payload, uint8_t payload_len, uint8_t *out_buf);
uint8_t build_pong(uint8_t *out_buf);
uint8_t build_ack(uint8_t ack_cmd, uint8_t status, uint8_t *out_buf);
uint8_t build_task_control(uint8_t task_id, uint8_t action, int16_t setpoint_mm, uint8_t *out_buf);
uint8_t build_sensor_data(int16_t accel_mg, int16_t angle_cdeg, int16_t wheel_accel,
                          int16_t reserved, uint8_t *out_buf);

#endif
