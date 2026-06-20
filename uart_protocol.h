/*
 * uart_protocol.h — UART Communication Protocol Interface
 * =========================================================
 * Platform : ESP32 (ESP-IDF, bare-metal, no Arduino)
 *
 * Manages UART2 for full-duplex communication with Raspberry Pi.
 *
 * Frame format (RPi → ESP32):  "CMD:<command>\n"
 * Frame format (ESP32 → RPi):  "TEL:<f>,<l>,<r>,<spd>\n"
 *                               "EVT:<event>\n"
 *
 * UART2 is used (UART0 reserved for ESP-IDF console/debug).
 */

#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"

/* ── UART2 Hardware Configuration ─────────── */
#define PROTO_UART_NUM    UART_NUM_2
#define PROTO_UART_BAUD   115200
#define PROTO_UART_TX     17      /* GPIO17 → connects to RPi RX */
#define PROTO_UART_RX     16      /* GPIO16 → connects to RPi TX */
#define PROTO_BUF_SIZE    256

/* ── API ──────────────────────────────────── */

/**
 * @brief  Initialise UART2 with the protocol settings.
 * @return ESP_OK on success.
 */
esp_err_t uart_protocol_init(void);

/**
 * @brief  Block until a full "CMD:<command>\n" frame is received,
 *         or until timeout_ticks elapses.
 *
 * The "CMD:" prefix is stripped; only the command string is written
 * into buf. The trailing newline is stripped.
 *
 * @param buf           Destination buffer.
 * @param buf_len       Size of destination buffer.
 * @param timeout_ticks FreeRTOS tick timeout (use portMAX_DELAY to block forever).
 * @return true  if a valid command was received.
 * @return false on timeout or framing error.
 */
bool uart_protocol_receive(char *buf, size_t buf_len, TickType_t timeout_ticks);

/**
 * @brief  Send a telemetry frame to the Raspberry Pi.
 *
 * Transmits: "TEL:<front_cm>,<left_cm>,<right_cm>,<speed>\n"
 *
 * @param front_cm  Fused front obstacle distance.
 * @param left_cm   Fused left obstacle distance.
 * @param right_cm  Fused right obstacle distance.
 * @param speed     Current motor PWM duty 0–255.
 */
void uart_protocol_send_telemetry(uint16_t front_cm, uint16_t left_cm,
                                  uint16_t right_cm, uint8_t speed);

/**
 * @brief  Send an event notification to the Raspberry Pi.
 *
 * Transmits: "EVT:<event_name>\n"
 * Example events: READY, COLLISION, BUMPER_HIT, LOW_BATTERY
 *
 * @param event  Null-terminated event name string.
 */
void uart_protocol_send_event(const char *event);

#endif /* UART_PROTOCOL_H */
