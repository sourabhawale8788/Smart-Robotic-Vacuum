/*
 * main.c — ESP32 Smart Vacuum Firmware Entry Point
 * =================================================
 * Project  : Smart Robotic Vacuum Cleaner with Object Detection
 * Platform : ESP32 DevKit V1 (bare-metal ESP-IDF v5.x, no Arduino)
 * Author   : [Your Team]
 * Date     : 2024–2025
 *
 * Description:
 *   Firmware entry point. Initialises all subsystems:
 *     1. UART2 (communication with Raspberry Pi)
 *     2. Motor control via LEDC PWM peripheral
 *     3. Sensor fusion (ultrasonic HC-SR04 + IR GP2Y0A)
 *     4. GPIO interrupt handlers for IR bump detection
 *     5. FreeRTOS tasks for sensor polling + telemetry TX
 *
 * Build & Flash:
 *   cd src/esp32
 *   idf.py set-target esp32
 *   idf.py build
 *   idf.py -p /dev/ttyUSB0 flash monitor
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"

#include "motor_control.h"
#include "sensor_fusion.h"
#include "uart_protocol.h"

static const char *TAG = "MAIN";

/* ─────────────────────────────────────────────────────────
 * FreeRTOS task stack sizes & priorities
 * ───────────────────────────────────────────────────────── */
#define TASK_STACK_UART     2048
#define TASK_STACK_SENSOR   2048
#define TASK_STACK_TELEMETRY 1536

#define TASK_PRIO_UART      5   /* highest — real-time command handling */
#define TASK_PRIO_SENSOR    4
#define TASK_PRIO_TELEMETRY 3

/* Telemetry transmit interval (ms) */
#define TELEMETRY_PERIOD_MS 50

/* ─────────────────────────────────────────────────────────
 * Shared state (written by sensor task, read by telemetry task)
 * Protected by lightweight mutex implemented via task notification
 * or simply volatile for single-writer/single-reader with 32-bit
 * values on ESP32 (guaranteed atomic read of aligned 32-bit).
 * ───────────────────────────────────────────────────────── */
volatile sensor_data_t g_sensor_data = {0};

/* ─────────────────────────────────────────────────────────
 * RTOS Task: UART Command Handler
 *   Blocks on uart_protocol_receive(), parses command frames
 *   from the Raspberry Pi, and dispatches motor commands.
 * ───────────────────────────────────────────────────────── */
static void task_uart_handler(void *arg)
{
    ESP_LOGI(TAG, "UART handler task started.");
    char cmd_buf[64];

    for (;;) {
        /* Blocking receive — wakes only when a full "CMD:...\n" frame arrives */
        if (uart_protocol_receive(cmd_buf, sizeof(cmd_buf), pdMS_TO_TICKS(100))) {
            motor_dispatch_command(cmd_buf);
        }
    }
}

/* ─────────────────────────────────────────────────────────
 * RTOS Task: Sensor Polling
 *   Fires ultrasonic triggers, reads echoes, reads IR ADC,
 *   runs weighted fusion, and writes to g_sensor_data.
 *   Emergency stop if front distance < EMERGENCY_STOP_CM.
 * ───────────────────────────────────────────────────────── */
#define SENSOR_POLL_PERIOD_MS   10
#define EMERGENCY_STOP_CM       12

static void task_sensor_poll(void *arg)
{
    ESP_LOGI(TAG, "Sensor poll task started.");
    sensor_data_t reading;

    for (;;) {
        sensor_fusion_read(&reading);

        /* Atomically update shared snapshot */
        g_sensor_data = reading;

        /* Hard-coded emergency stop — bypasses UART latency */
        if (reading.front_cm < EMERGENCY_STOP_CM) {
            ESP_LOGW(TAG, "EMERGENCY: front_cm=%d — STOP", reading.front_cm);
            motor_emergency_stop();
            uart_protocol_send_event("COLLISION");
            /* Pause sensor polling to avoid storm of STOP commands */
            vTaskDelay(pdMS_TO_TICKS(300));
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_POLL_PERIOD_MS));
    }
}

/* ─────────────────────────────────────────────────────────
 * RTOS Task: Telemetry Transmitter
 *   Reads g_sensor_data snapshot and sends
 *   "TEL:<front>,<left>,<right>,<speed>\n" to Raspberry Pi.
 * ───────────────────────────────────────────────────────── */
static void task_telemetry_tx(void *arg)
{
    ESP_LOGI(TAG, "Telemetry TX task started.");

    for (;;) {
        sensor_data_t snap = g_sensor_data;  /* read snapshot */
        uint8_t speed      = motor_get_speed();

        uart_protocol_send_telemetry(
            snap.front_cm,
            snap.left_cm,
            snap.right_cm,
            speed
        );

        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
    }
}

/* ─────────────────────────────────────────────────────────
 * app_main — ESP-IDF entry point (equivalent to main())
 * ───────────────────────────────────────────────────────── */
void app_main(void)
{
    ESP_LOGI(TAG, "═══════════════════════════════════════");
    ESP_LOGI(TAG, " Smart Robotic Vacuum — ESP32 Firmware");
    ESP_LOGI(TAG, " Build: %s %s", __DATE__, __TIME__);
    ESP_LOGI(TAG, "═══════════════════════════════════════");

    /* 1. Initialise UART2 for Raspberry Pi communication */
    ESP_ERROR_CHECK(uart_protocol_init());
    ESP_LOGI(TAG, "[1/3] UART2 initialised.");

    /* 2. Initialise PWM motor control (LEDC peripheral) */
    ESP_ERROR_CHECK(motor_control_init());
    ESP_LOGI(TAG, "[2/3] Motor control initialised.");

    /* 3. Initialise sensor fusion (ultrasonic + IR) */
    ESP_ERROR_CHECK(sensor_fusion_init());
    ESP_LOGI(TAG, "[3/3] Sensor fusion initialised.");

    /* Notify Raspberry Pi that ESP32 is ready */
    uart_protocol_send_event("READY");

    /* 4. Launch FreeRTOS tasks */
    xTaskCreate(task_uart_handler,  "uart_handler",  TASK_STACK_UART,
                NULL, TASK_PRIO_UART,      NULL);

    xTaskCreate(task_sensor_poll,   "sensor_poll",   TASK_STACK_SENSOR,
                NULL, TASK_PRIO_SENSOR,    NULL);

    xTaskCreate(task_telemetry_tx,  "telemetry_tx",  TASK_STACK_TELEMETRY,
                NULL, TASK_PRIO_TELEMETRY, NULL);

    ESP_LOGI(TAG, "All tasks launched — system running.");
    /* app_main returns; the scheduler takes over */
}
