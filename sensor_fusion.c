/*
 * sensor_fusion.c — Ultrasonic + IR Sensor Fusion Implementation
 * ===============================================================
 * Platform : ESP32 (ESP-IDF v5.x, bare-metal, no Arduino)
 *
 * HC-SR04 measurement uses esp_timer_get_time() for microsecond precision
 * without blocking a FreeRTOS tick — giving deterministic sub-10ms reads.
 *
 * GP2Y0A21 uses the ESP32 12-bit ADC1 with multi-sample averaging
 * to reduce ADC noise inherent in the ESP32 SAR ADC.
 */

#include "sensor_fusion.h"

#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"   /* ets_delay_us() — busy-wait µs delay */

static const char *TAG = "SENSOR";

/* ─────────────────────────────────────────────────────────
 * Initialisation
 * ───────────────────────────────────────────────────────── */

esp_err_t sensor_fusion_init(void)
{
    /* ── HC-SR04 TRIG pins (output) ────────── */
    gpio_config_t trig_cfg = {
        .pin_bit_mask = (1ULL << SONAR_FRONT_TRIG) |
                        (1ULL << SONAR_LEFT_TRIG)  |
                        (1ULL << SONAR_RIGHT_TRIG),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&trig_cfg);
    if (ret != ESP_OK) return ret;

    /* Start with TRIG LOW */
    gpio_set_level(SONAR_FRONT_TRIG, 0);
    gpio_set_level(SONAR_LEFT_TRIG,  0);
    gpio_set_level(SONAR_RIGHT_TRIG, 0);

    /* ── HC-SR04 ECHO pins (input, no pull) ── */
    gpio_config_t echo_cfg = {
        .pin_bit_mask = (1ULL << SONAR_FRONT_ECHO) |
                        (1ULL << SONAR_LEFT_ECHO)  |
                        (1ULL << SONAR_RIGHT_ECHO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&echo_cfg);
    if (ret != ESP_OK) return ret;

    /* ── ADC1 for GP2Y0A IR sensors ─────────
     * 12-bit resolution, 11dB attenuation → input range 0–3.9V
     */
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(IR_FRONT_LEFT_CH,  ADC_ATTEN_DB_11);
    adc1_config_channel_atten(IR_FRONT_RIGHT_CH, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(IR_LEFT_CH,        ADC_ATTEN_DB_11);
    adc1_config_channel_atten(IR_RIGHT_CH,       ADC_ATTEN_DB_11);

    ESP_LOGI(TAG, "Sensor fusion initialised "
                  "(3× HC-SR04, 4× GP2Y0A21, fusion W_US=%d W_IR=%d)",
             FUSION_WEIGHT_US, FUSION_WEIGHT_IR);
    return ESP_OK;
}

/* ─────────────────────────────────────────────────────────
 * HC-SR04 single measurement
 * ───────────────────────────────────────────────────────── */

#define ECHO_TIMEOUT_US  38000   /* 38 ms → ~400 cm; beyond = no object */

uint16_t sensor_hcsr04_measure(int trig_gpio, int echo_gpio)
{
    /* 10 µs trigger pulse */
    gpio_set_level(trig_gpio, 0);
    ets_delay_us(2);
    gpio_set_level(trig_gpio, 1);
    ets_delay_us(10);
    gpio_set_level(trig_gpio, 0);

    /* Wait for ECHO to go HIGH (with timeout) */
    int64_t t0 = esp_timer_get_time();
    while (gpio_get_level(echo_gpio) == 0) {
        if ((esp_timer_get_time() - t0) > ECHO_TIMEOUT_US) {
            return RANGE_MAX_CM;   /* no echo received */
        }
    }

    /* Measure HIGH pulse duration */
    int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(echo_gpio) == 1) {
        if ((esp_timer_get_time() - echo_start) > ECHO_TIMEOUT_US) {
            return RANGE_MAX_CM;
        }
    }
    int64_t echo_end = esp_timer_get_time();

    uint32_t duration_us = (uint32_t)(echo_end - echo_start);

    /*
     * distance_cm = duration_µs / 58
     * (Speed of sound = 343 m/s; round trip ÷ 2 → 58 µs/cm)
     */
    uint16_t dist_cm = (uint16_t)(duration_us / 58);

    if (dist_cm < RANGE_MIN_CM) return RANGE_MIN_CM;
    if (dist_cm > RANGE_MAX_CM) return RANGE_MAX_CM;
    return dist_cm;
}

/* ─────────────────────────────────────────────────────────
 * GP2Y0A21 IR single measurement
 * ───────────────────────────────────────────────────────── */

#define IR_SAMPLE_COUNT  8

uint16_t sensor_ir_measure(int adc_channel)
{
    /* Average multiple ADC samples to reduce ESP32 ADC noise */
    uint32_t sum = 0;
    for (int i = 0; i < IR_SAMPLE_COUNT; i++) {
        sum += adc1_get_raw((adc1_channel_t)adc_channel);
    }
    uint32_t raw_avg = sum / IR_SAMPLE_COUNT;

    /*
     * Convert ADC raw to voltage (12-bit, 3.3V reference, 11dB atten):
     *   voltage = raw * 3.3 / 4095.0
     *
     * GP2Y0A21YK0F inverse power-law calibration:
     *   distance_cm = 27.728 * voltage^(-1.2045)
     * (Derived from Sharp datasheet output voltage vs distance curve)
     */
    float voltage  = (float)raw_avg * (3.3f / 4095.0f);

    if (voltage < 0.1f) {
        return 80;   /* voltage too low = object too far for GP2Y range */
    }

    float dist_cm = 27.728f * powf(voltage, -1.2045f);

    if (dist_cm < RANGE_MIN_CM) return RANGE_MIN_CM;
    if (dist_cm > 80)           return 80;  /* GP2Y reliable max */
    return (uint16_t)dist_cm;
}

/* ─────────────────────────────────────────────────────────
 * Weighted sensor fusion
 * ───────────────────────────────────────────────────────── */

static uint16_t _fuse(uint16_t us_cm, uint16_t ir_cm)
{
    /*
     * Weighted average: ultrasonic weighted higher (less noise at range).
     * Formula: (W_US * us + W_IR * ir) / (W_US + W_IR)
     */
    return (uint16_t)(
        (FUSION_WEIGHT_US * (uint32_t)us_cm + FUSION_WEIGHT_IR * (uint32_t)ir_cm)
        / (FUSION_WEIGHT_US + FUSION_WEIGHT_IR)
    );
}

/* ─────────────────────────────────────────────────────────
 * Full poll cycle
 * ───────────────────────────────────────────────────────── */

void sensor_fusion_read(sensor_data_t *out)
{
    if (!out) return;

    /* HC-SR04 readings — sequential (each trigger needs quiet line) */
    uint16_t us_front = sensor_hcsr04_measure(SONAR_FRONT_TRIG, SONAR_FRONT_ECHO);
    vTaskDelay(pdMS_TO_TICKS(5));   /* brief gap between sonar triggers */
    uint16_t us_left  = sensor_hcsr04_measure(SONAR_LEFT_TRIG,  SONAR_LEFT_ECHO);
    vTaskDelay(pdMS_TO_TICKS(5));
    uint16_t us_right = sensor_hcsr04_measure(SONAR_RIGHT_TRIG, SONAR_RIGHT_ECHO);

    /* GP2Y0A readings */
    uint16_t ir_fl = sensor_ir_measure(IR_FRONT_LEFT_CH);
    uint16_t ir_fr = sensor_ir_measure(IR_FRONT_RIGHT_CH);
    uint16_t ir_l  = sensor_ir_measure(IR_LEFT_CH);
    uint16_t ir_r  = sensor_ir_measure(IR_RIGHT_CH);

    /* Fuse front (use average of FL and FR IR for front IR) */
    uint16_t ir_front_avg = (ir_fl + ir_fr) / 2;
    out->front_cm = _fuse(us_front, ir_front_avg);
    out->left_cm  = _fuse(us_left,  ir_l);
    out->right_cm = _fuse(us_right, ir_r);
    out->ir_fl_cm = ir_fl;
    out->ir_fr_cm = ir_fr;

    ESP_LOGD(TAG,
             "Fused: F=%dcm L=%dcm R=%dcm | "
             "US: F=%d L=%d R=%d | IR: FL=%d FR=%d L=%d R=%d",
             out->front_cm, out->left_cm, out->right_cm,
             us_front, us_left, us_right,
             ir_fl, ir_fr, ir_l, ir_r);
}
