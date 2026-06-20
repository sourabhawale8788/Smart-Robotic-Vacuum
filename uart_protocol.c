/*
 * uart_protocol.c — UART Communication Protocol Implementation
 * =============================================================
 * Platform : ESP32 (ESP-IDF v5.x, bare-metal, no Arduino)
 *
 * Uses the ESP-IDF UART driver with a hardware FIFO + DMA buffer.
 * Ring buffer approach ensures no bytes are dropped even under
 * moderate CPU load from sensor polling and motor control tasks.
 */

#include "uart_protocol.h"

#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "UART_PROTO";

/* ─────────────────────────────────────────────────────────
 * Initialisation
 * ───────────────────────────────────────────────────────── */

esp_err_t uart_protocol_init(void)
{
    const uart_config_t uart_cfg = {
        .baud_rate  = PROTO_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    esp_err_t ret;

    ret = uart_driver_install(
        PROTO_UART_NUM,
        PROTO_BUF_SIZE * 2,   /* RX ring buffer */
        PROTO_BUF_SIZE,       /* TX ring buffer */
        0,                    /* queue size (0 = no event queue) */
        NULL,                 /* queue handle */
        0                     /* interrupt flags */
    );
    if (ret != ESP_OK) return ret;

    ret = uart_param_config(PROTO_UART_NUM, &uart_cfg);
    if (ret != ESP_OK) return ret;

    ret = uart_set_pin(
        PROTO_UART_NUM,
        PROTO_UART_TX,          /* TX */
        PROTO_UART_RX,          /* RX */
        UART_PIN_NO_CHANGE,     /* RTS — not used */
        UART_PIN_NO_CHANGE      /* CTS — not used */
    );
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "UART%d @ %d baud | TX=GPIO%d RX=GPIO%d",
             PROTO_UART_NUM, PROTO_UART_BAUD,
             PROTO_UART_TX,  PROTO_UART_RX);
    return ESP_OK;
}

/* ─────────────────────────────────────────────────────────
 * Receive
 * ───────────────────────────────────────────────────────── */

/*
 * Internal ring buffer for incoming bytes.
 * We read byte-by-byte and accumulate lines, then parse.
 */
static char s_line_buf[PROTO_BUF_SIZE];
static int  s_line_pos = 0;

bool uart_protocol_receive(char *buf, size_t buf_len, TickType_t timeout_ticks)
{
    if (!buf || buf_len == 0) return false;

    TickType_t deadline = xTaskGetTickCount() + timeout_ticks;
    uint8_t byte;

    while (xTaskGetTickCount() < deadline) {
        /* Non-blocking 1-byte read with short tick timeout */
        int len = uart_read_bytes(
            PROTO_UART_NUM, &byte, 1,
            pdMS_TO_TICKS(5)
        );
        if (len <= 0) continue;

        if (byte == '\n' || byte == '\r') {
            if (s_line_pos == 0) continue;   /* skip blank lines */

            s_line_buf[s_line_pos] = '\0';
            s_line_pos = 0;

            /* Validate and strip "CMD:" prefix */
            if (strncmp(s_line_buf, "CMD:", 4) == 0) {
                const char *cmd = s_line_buf + 4;
                size_t cmd_len  = strlen(cmd);
                if (cmd_len == 0 || cmd_len >= buf_len) continue;

                memcpy(buf, cmd, cmd_len + 1);
                ESP_LOGD(TAG, "RX CMD: '%s'", buf);
                return true;
            } else {
                /* Unknown frame type — log and discard */
                ESP_LOGD(TAG, "Discarding unknown frame: '%s'", s_line_buf);
            }
        } else {
            /* Accumulate byte */
            if (s_line_pos < (int)sizeof(s_line_buf) - 1) {
                s_line_buf[s_line_pos++] = (char)byte;
            } else {
                /* Buffer overflow — reset and discard */
                ESP_LOGW(TAG, "RX buffer overflow — discarding.");
                s_line_pos = 0;
            }
        }
    }

    return false;   /* timeout */
}

/* ─────────────────────────────────────────────────────────
 * Transmit helpers
 * ───────────────────────────────────────────────────────── */

void uart_protocol_send_telemetry(
    uint16_t front_cm, uint16_t left_cm,
    uint16_t right_cm, uint8_t speed)
{
    char frame[64];
    int len = snprintf(frame, sizeof(frame),
                       "TEL:%u,%u,%u,%u\n",
                       front_cm, left_cm, right_cm, (unsigned)speed);
    uart_write_bytes(PROTO_UART_NUM, frame, len);
}

void uart_protocol_send_event(const char *event)
{
    if (!event) return;
    char frame[64];
    int len = snprintf(frame, sizeof(frame), "EVT:%s\n", event);
    uart_write_bytes(PROTO_UART_NUM, frame, len);
    ESP_LOGI(TAG, "TX EVT: %s", event);
}
