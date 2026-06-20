/*
 * sensor_fusion.h — Ultrasonic + IR Sensor Fusion Interface
 * ===========================================================
 * Platform : ESP32 (ESP-IDF, bare-metal, no Arduino)
 *
 * Combines HC-SR04 ultrasonic ranging and GP2Y0A21 IR proximity
 * sensors into a single fused distance reading for each direction.
 *
 * Fusion formula (weighted average):
 *   fused_cm = (W_US * us_cm + W_IR * ir_cm) / (W_US + W_IR)
 *   W_US = 6, W_IR = 4  (ultrasonic weighted higher — lower noise)
 *
 * HC-SR04 timing:
 *   - Trigger: 10 µs HIGH pulse on TRIG pin
 *   - Echo: pulse width proportional to distance
 *     distance_cm = echo_duration_µs / 58
 *
 * GP2Y0A21YK0F ADC→distance curve (linearised):
 *   voltage = adc_raw * (3.3 / 4095.0)     [12-bit ADC]
 *   distance_cm = 27.728 * pow(voltage, -1.2045)
 */

#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include "esp_err.h"
#include <stdint.h>

/* ── HC-SR04 GPIO Pins ────────────────────── */
#define SONAR_FRONT_TRIG   26
#define SONAR_FRONT_ECHO   27
#define SONAR_LEFT_TRIG    14
#define SONAR_LEFT_ECHO    12
#define SONAR_RIGHT_TRIG   13
#define SONAR_RIGHT_ECHO   5

/* ── GP2Y0A21 ADC Channels ────────────────── */
#define IR_FRONT_LEFT_CH   ADC1_CHANNEL_4   /* GPIO32 */
#define IR_FRONT_RIGHT_CH  ADC1_CHANNEL_5   /* GPIO33 */
#define IR_LEFT_CH         ADC1_CHANNEL_6   /* GPIO34 */
#define IR_RIGHT_CH        ADC1_CHANNEL_7   /* GPIO35 */

/* ── Fusion weights ────────────────────────── */
#define FUSION_WEIGHT_US   6
#define FUSION_WEIGHT_IR   4

/* ── Distance clamp ─────────────────────────
 * Values beyond RANGE_MAX_CM are unreliable.
 * HC-SR04 reliable range: 2–400 cm
 * GP2Y0A21 reliable range: 10–80 cm
 */
#define RANGE_MAX_CM       200
#define RANGE_MIN_CM       2

/* ── Shared data structure ──────────────────
 * Written by sensor task, read by telemetry task and emergency handler.
 * All fields are uint16_t — atomic reads on 32-bit ESP32 aligned access.
 */
typedef struct {
    uint16_t front_cm;   /* Fused front distance */
    uint16_t left_cm;    /* Fused left distance  */
    uint16_t right_cm;   /* Fused right distance */
    uint16_t ir_fl_cm;   /* Raw IR front-left    */
    uint16_t ir_fr_cm;   /* Raw IR front-right   */
} sensor_data_t;

/* ── API ──────────────────────────────────── */

/**
 * @brief  Configure GPIO for HC-SR04 and ADC1 for GP2Y0A sensors.
 * @return ESP_OK on success.
 */
esp_err_t sensor_fusion_init(void);

/**
 * @brief  Perform a full sensor poll cycle and write results.
 *
 * Triggers each ultrasonic, reads all IR channels, applies
 * weighted fusion, and populates *out.
 *
 * @param out  Pointer to sensor_data_t to fill.
 */
void sensor_fusion_read(sensor_data_t *out);

/**
 * @brief  Trigger a single HC-SR04 and return distance in cm.
 *
 * Blocks for up to ~38 ms (maximum echo timeout at 400 cm range).
 * Returns RANGE_MAX_CM if no echo received (object too far or error).
 *
 * @param trig_gpio  Trigger GPIO number.
 * @param echo_gpio  Echo GPIO number.
 * @return           Distance in centimetres.
 */
uint16_t sensor_hcsr04_measure(int trig_gpio, int echo_gpio);

/**
 * @brief  Read one GP2Y0A21 channel and return distance in cm.
 *
 * Averages 8 ADC samples, applies the inverse power-law curve,
 * and clamps to [RANGE_MIN_CM, 80].
 *
 * @param adc_channel  ADC1 channel (adc1_channel_t).
 * @return             Distance in centimetres.
 */
uint16_t sensor_ir_measure(int adc_channel);

#endif /* SENSOR_FUSION_H */
