#include "UART4_OPI/uart4_opi.h"
#include "protocol.h"
#include "ti_msp_dl_config.h"
#include <ti/driverlib/driverlib.h>

/* ---------------- RX Ring Buffer ---------------- */
#define RX_BUF_SIZE 128

static volatile uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

static parser_ctx_t g_parser;
static uint8_t g_tx_buf[MAX_FRAME_LEN];

volatile uint8_t  g_opi_last_ack_cmd     = 0;
volatile uint8_t  g_opi_last_ack_status  = 0xFF;
volatile uint16_t g_opi_task_ack_count   = 0;
volatile uint16_t g_opi_sensor_ack_count = 0;
volatile uint16_t g_opi_sensor_tx_count  = 0;
volatile uint8_t  g_opi_last_task_id     = 0;
volatile uint8_t  g_opi_last_task_action = TASK_ACTION_STOP;
volatile int16_t  g_opi_last_accel_mg    = 0;
volatile int16_t  g_opi_last_angle_cdeg  = 0;
volatile int16_t  g_opi_last_wheel_accel = 0;
volatile uint8_t  g_opi_done_flag        = 0;
volatile uint8_t  g_opi_done_task_id     = 0;
volatile uint8_t  g_opi_done_status      = 0;
volatile int16_t  g_target_dx            = 0;
volatile int16_t  g_target_dy            = 0;
volatile uint16_t g_target_dist          = 0;
volatile uint8_t  g_new_target           = 0;

static inline uint16_t ring_next(uint16_t idx)
{
    return (idx + 1) % RX_BUF_SIZE;
}

static inline void rx_ring_put(uint8_t byte)
{
    uint16_t next = ring_next(rx_head);
    if (next == rx_tail) return;
    rx_buf[rx_head] = byte;
    rx_head = next;
}

static inline bool rx_ring_get(uint8_t *byte)
{
    if (rx_head == rx_tail) return false;
    *byte = rx_buf[rx_tail];
    rx_tail = ring_next(rx_tail);
    return true;
}

static volatile uint8_t s_uart4_opi_ready = 0;

/* ---------------- TX (bounded, never blocks menu) ----------- */
static void uart4_send(const uint8_t *data, uint8_t len)
{
    if (!s_uart4_opi_ready) return;

    for (uint8_t i = 0; i < len; i++) {
        uint32_t timeout = 20000;
        while (DL_UART_isBusy(UART_4_INST) && timeout--) {
        }
        if (timeout == 0) {
            return;
        }
        DL_UART_transmitData(UART_4_INST, data[i]);
    }
}

/* ---------------- Public API -------------------- */
void uart4_opi_init(void)
{
    parser_init(&g_parser);
    rx_head = rx_tail = 0;

    g_opi_last_ack_cmd = 0;
    g_opi_last_ack_status = 0xFF;
    g_opi_task_ack_count = 0;
    g_opi_sensor_ack_count = 0;
    g_opi_sensor_tx_count = 0;
    g_opi_last_task_id = 0;
    g_opi_last_task_action = TASK_ACTION_STOP;
    g_opi_last_accel_mg = 0;
    g_opi_last_angle_cdeg = 0;
    g_opi_last_wheel_accel = 0;
    g_opi_done_flag = 0;
    g_opi_done_task_id = 0;
    g_opi_done_status = 0;
    g_target_dx = 0;
    g_target_dy = 0;
    g_target_dist = 0;
    g_new_target = 0;

    DL_UART_Main_disable(UART_4_INST);
    DL_UART_Main_enableInterrupt(UART_4_INST, DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enable(UART_4_INST);
    NVIC_ClearPendingIRQ(UART_4_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_4_INST_INT_IRQN);
    s_uart4_opi_ready = 1;
}

void uart4_opi_process(void)
{
    uint8_t byte;
    while (rx_ring_get(&byte)) {
        frame_t temp;
        if (parser_feed_byte(&g_parser, byte, &temp)) {
            uint8_t tx_len = 0;

            switch (temp.cmd) {
            case CMD_TARGET_POS:
                if (temp.payload_len >= 6) {
                    g_target_dx = (int16_t)((temp.payload[0] << 8) | temp.payload[1]);
                    g_target_dy = (int16_t)((temp.payload[2] << 8) | temp.payload[3]);
                    g_target_dist = (uint16_t)((temp.payload[4] << 8) | temp.payload[5]);
                    g_new_target = 1;
                    tx_len = build_ack(CMD_TARGET_POS, 0, g_tx_buf);
                } else {
                    tx_len = build_ack(CMD_TARGET_POS, 1, g_tx_buf);
                }
                break;

            case CMD_SHOOT:
            case CMD_SET_TASK:
                tx_len = build_ack(temp.cmd, 0, g_tx_buf);
                break;

            case CMD_ACK:
                if (temp.payload_len >= 2) {
                    g_opi_last_ack_cmd = temp.payload[0];
                    g_opi_last_ack_status = temp.payload[1];
                    if (temp.payload[0] == CMD_TASK_CONTROL) {
                        g_opi_task_ack_count++;
                    } else if (temp.payload[0] == CMD_SENSOR_DATA) {
                        g_opi_sensor_ack_count++;
                    }
                }
                break;

            case CMD_TASK_DONE:
                if (temp.payload_len >= 2) {
                    g_opi_done_task_id = temp.payload[0];
                    g_opi_done_status = temp.payload[1];
                    g_opi_done_flag = 1;
                    tx_len = build_ack(CMD_TASK_DONE, 0, g_tx_buf);
                } else {
                    tx_len = build_ack(CMD_TASK_DONE, 1, g_tx_buf);
                }
                break;

            case CMD_PING:
                tx_len = build_pong(g_tx_buf);
                break;

            default:
                break;
            }

            if (tx_len > 0) {
                uart4_send(g_tx_buf, tx_len);
            }
        }
    }
}

void uart4_opi_send_task_control(uint8_t task_id, uint8_t action, int16_t setpoint_mm)
{
    uint8_t tx_len = build_task_control(task_id, action, setpoint_mm, g_tx_buf);

    g_opi_last_task_id = task_id;
    g_opi_last_task_action = action;
    uart4_send(g_tx_buf, tx_len);
}

void opi_task_start(uint8_t task_id, int16_t setpoint_mm)
{
    uart4_opi_send_task_control(task_id, TASK_ACTION_START, setpoint_mm);
}

void opi_task_stop(uint8_t task_id, int16_t setpoint_mm)
{
    uart4_opi_send_task_control(task_id, TASK_ACTION_STOP, setpoint_mm);
}

uint8_t opi_consume_task3_done(void)
{
    if (!g_opi_done_flag) return 0;

    g_opi_done_flag = 0;
    return (g_opi_done_task_id == TASK_ID_STATIC_SWING && g_opi_done_status == 0);
}

void uart4_opi_start_record(uint8_t task_id)
{
    uart4_opi_send_task_control(task_id, TASK_ACTION_START, 0);
}

void uart4_opi_stop_record(uint8_t task_id)
{
    uart4_opi_send_task_control(task_id, TASK_ACTION_STOP, 0);
}

void uart4_opi_send_sensor_data(int16_t accel_mg, int16_t angle_cdeg, int16_t wheel_accel)
{
    uint8_t tx_len = build_sensor_data(accel_mg, angle_cdeg, wheel_accel, 0, g_tx_buf);

    g_opi_last_accel_mg = accel_mg;
    g_opi_last_angle_cdeg = angle_cdeg;
    g_opi_last_wheel_accel = wheel_accel;
    g_opi_sensor_tx_count++;
    uart4_send(g_tx_buf, tx_len);
}

/* ---------------- ISR --------------------------- */
void UART4_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_4_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        while (DL_UART_Main_isRXFIFOEmpty(UART_4_INST) == false) {
            rx_ring_put(DL_UART_receiveData(UART_4_INST));
        }
        break;
    default:
        break;
    }
}
