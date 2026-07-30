#include "protocol.h"
#include <stddef.h>

static uint8_t calc_checksum(const uint8_t *data, uint8_t len)
{
    uint8_t xor_val = 0;
    for (uint8_t i = 0; i < len; i++) {
        xor_val ^= data[i];
    }
    return xor_val;
}

void parser_init(parser_ctx_t *ctx)
{
    ctx->state = STATE_WAIT_HEAD_H;
    ctx->buf_idx = 0;
    ctx->expected_len = 0;
}

int parser_feed_byte(parser_ctx_t *ctx, uint8_t byte, frame_t *out)
{
    switch (ctx->state) {
    case STATE_WAIT_HEAD_H:
        if (byte == FRAME_HEAD_H)
            ctx->state = STATE_WAIT_HEAD_L;
        break;

    case STATE_WAIT_HEAD_L:
        if (byte == FRAME_HEAD_L) {
            ctx->state = STATE_WAIT_LEN;
            ctx->buf_idx = 0;
        } else if (byte != FRAME_HEAD_H) {
            ctx->state = STATE_WAIT_HEAD_H;
        }
        break;

    case STATE_WAIT_LEN:
        if (byte >= 1 && byte <= (MAX_PAYLOAD_LEN + 1)) {
            ctx->expected_len = byte;
            ctx->buf[0] = byte;
            ctx->buf_idx = 1;
            ctx->state = STATE_WAIT_DATA;
        } else {
            ctx->state = STATE_WAIT_HEAD_H;
        }
        break;

    case STATE_WAIT_DATA:
        ctx->buf[ctx->buf_idx++] = byte;
        if (ctx->buf_idx == 1 + ctx->expected_len) {
            ctx->state = STATE_WAIT_CHECKSUM;
        }
        break;

    case STATE_WAIT_CHECKSUM:
        {
            uint8_t expected = calc_checksum(ctx->buf, 1 + ctx->expected_len);
            if (byte == expected) {
                ctx->state = STATE_WAIT_TAIL;
            } else {
                ctx->state = STATE_WAIT_HEAD_H;
            }
        }
        break;

    case STATE_WAIT_TAIL:
        if (byte == FRAME_TAIL) {
            out->cmd = ctx->buf[1];
            out->payload_len = ctx->expected_len - 1;
            for (uint8_t i = 0; i < out->payload_len; i++) {
                out->payload[i] = ctx->buf[2 + i];
            }
            ctx->state = STATE_WAIT_HEAD_H;
            return 1;
        }
        ctx->state = STATE_WAIT_HEAD_H;
        break;
    }
    return 0;
}

uint8_t build_frame(uint8_t cmd, const uint8_t *payload, uint8_t payload_len, uint8_t *out_buf)
{
    uint8_t idx = 0;
    uint8_t len = 1 + payload_len;

    out_buf[idx++] = FRAME_HEAD_H;
    out_buf[idx++] = FRAME_HEAD_L;
    out_buf[idx++] = len;
    out_buf[idx++] = cmd;
    for (uint8_t i = 0; i < payload_len; i++) {
        out_buf[idx++] = payload[i];
    }
    uint8_t checksum = calc_checksum(&out_buf[2], len + 1);
    out_buf[idx++] = checksum;
    out_buf[idx++] = FRAME_TAIL;

    return idx;
}

uint8_t build_pong(uint8_t *out_buf)
{
    return build_frame(CMD_PONG, NULL, 0, out_buf);
}

uint8_t build_ack(uint8_t ack_cmd, uint8_t status, uint8_t *out_buf)
{
    uint8_t payload[2] = {ack_cmd, status};
    return build_frame(CMD_ACK, payload, 2, out_buf);
}

uint8_t build_task_control(uint8_t task_id, uint8_t action, int16_t setpoint_mm, uint8_t *out_buf)
{
    uint8_t payload[4];

    payload[0] = task_id;
    payload[1] = action;
    payload[2] = (uint8_t)((setpoint_mm >> 8) & 0xFF);
    payload[3] = (uint8_t)(setpoint_mm & 0xFF);

    return build_frame(CMD_TASK_CONTROL, payload, 4, out_buf);
}

uint8_t build_sensor_data(int16_t accel_mg, int16_t angle_cdeg, int16_t wheel_accel,
                          int16_t reserved, uint8_t *out_buf)
{
    uint8_t payload[8];

    payload[0] = (uint8_t)((accel_mg >> 8) & 0xFF);
    payload[1] = (uint8_t)(accel_mg & 0xFF);
    payload[2] = (uint8_t)((angle_cdeg >> 8) & 0xFF);
    payload[3] = (uint8_t)(angle_cdeg & 0xFF);
    payload[4] = (uint8_t)((wheel_accel >> 8) & 0xFF);
    payload[5] = (uint8_t)(wheel_accel & 0xFF);
    payload[6] = (uint8_t)((reserved >> 8) & 0xFF);
    payload[7] = (uint8_t)(reserved & 0xFF);

    return build_frame(CMD_SENSOR_DATA, payload, 8, out_buf);
}
