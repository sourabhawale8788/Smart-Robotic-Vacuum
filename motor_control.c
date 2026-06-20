/*
 * motor_control.c — PWM DC Motor Driver Implementation
 * =====================================================
 * Platform : ESP32 (ESP-IDF v5.x, bare-metal, no Arduino)
 *
 * Implements PWM-based speed control and direction control for
 * a four-wheel differential-drive robot using two L298N H-bridges.
 *
 * The LEDC peripheral is used instead of the older mcpwm peripheral
 * because LEDC supports up to 8 independent channels with independent
 * frequency and duty control — ideal for dual H-bridge PWM.
 */

#include "motor_control.h"

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "MOTOR";

/* ─────────────────────────────────────────────────────────
 * Internal state
 * ───────────────────────────────────────────────────────── */
static volatile uint8_t s_current_speed = 180;  /* default duty 0–255 */

/* ─────────────────────────────────────────────────────────
 * Helpers: direction GPIO control
 * ───────────────────────────────────────────────────────── */

static inline void _dir_pins(int in1, int in2, int in3, int in4)
{
    gpio_set_level(MOTOR_IN1_GPIO, in1);
    gpio_set_level(MOTOR_IN2_GPIO, in2);
    gpio_set_level(MOTOR_IN3_GPIO, in3);
    gpio_set_level(MOTOR_IN4_GPIO, in4);
}

/* ─────────────────────────────────────────────────────────
 * Initialisation
 * ───────────────────────────────────────────────────────── */

esp_err_t motor_control_init(void)
{
    /* ── Direction GPIO ───────────────────── */
    gpio_config_t dir_cfg = {
        .pin_bit_mask = (1ULL << MOTOR_IN1_GPIO) |
                        (1ULL << MOTOR_IN2_GPIO) |
                        (1ULL << MOTOR_IN3_GPIO) |
                        (1ULL << MOTOR_IN4_GPIO) |
                        (1ULL << VACUUM_RELAY_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&dir_cfg);
    if (ret != ESP_OK) return ret;

    /* All direction pins low = coasting stop */
    _dir_pins(0, 0, 0, 0);
    gpio_set_level(VACUUM_RELAY_GPIO, 0);

    /* ── LEDC Timer (shared by both channels) ─ */
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = (ledc_timer_bit_t)MOTOR_PWM_RES_BITS,
        .freq_hz         = MOTOR_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK) return ret;

    /* ── LEDC Channel — Left Motors ────────── */
    ledc_channel_config_t ch_left = {
        .channel    = LEDC_CHANNEL_0,
        .duty       = 0,
        .gpio_num   = MOTOR_L_PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0,
    };
    ret = ledc_channel_config(&ch_left);
    if (ret != ESP_OK) return ret;

    /* ── LEDC Channel — Right Motors ───────── */
    ledc_channel_config_t ch_right = {
        .channel    = LEDC_CHANNEL_1,
        .duty       = 0,
        .gpio_num   = MOTOR_R_PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0,
    };
    ret = ledc_channel_config(&ch_right);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "Motor control initialised. PWM=%dHz res=%dbit",
             MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
    return ESP_OK;
}

/* ─────────────────────────────────────────────────────────
 * Private: apply duty cycle to both LEDC channels
 * ───────────────────────────────────────────────────────── */

static void _apply_duty(uint8_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

/* ─────────────────────────────────────────────────────────
 * Movement functions
 * ───────────────────────────────────────────────────────── */

void motor_forward(void)
{
    _dir_pins(1, 0, 1, 0);    /* IN1=H IN2=L IN3=H IN4=L → both bridges forward */
    _apply_duty(s_current_speed);
    ESP_LOGD(TAG, "FWD  duty=%d", s_current_speed);
}

void motor_reverse(void)
{
    _dir_pins(0, 1, 0, 1);    /* IN1=L IN2=H IN3=L IN4=H → both bridges reverse */
    _apply_duty(s_current_speed);
    ESP_LOGD(TAG, "REV  duty=%d", s_current_speed);
}

void motor_turn_left(void)
{
    /*
     * Pivot left: right side drives forward, left side drives reverse.
     * This gives a tight in-place turn. For a gentler arc, set left to 0.
     */
    _dir_pins(0, 1, 1, 0);
    _apply_duty(s_current_speed);
    ESP_LOGD(TAG, "LEFT duty=%d", s_current_speed);
}

void motor_turn_right(void)
{
    /* Pivot right: left side drives forward, right side reverses. */
    _dir_pins(1, 0, 0, 1);
    _apply_duty(s_current_speed);
    ESP_LOGD(TAG, "RIGHT duty=%d", s_current_speed);
}

void motor_stop(void)
{
    _dir_pins(0, 0, 0, 0);
    _apply_duty(0);
    ESP_LOGD(TAG, "STOP");
}

void motor_emergency_stop(void)
{
    /*
     * ISR-safe: set duty directly without FreeRTOS blocking call.
     * ledc_set_duty + ledc_update_duty are register writes — safe in ISR.
     */
    gpio_set_level(MOTOR_IN1_GPIO, 0);
    gpio_set_level(MOTOR_IN2_GPIO, 0);
    gpio_set_level(MOTOR_IN3_GPIO, 0);
    gpio_set_level(MOTOR_IN4_GPIO, 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

void motor_set_speed(uint8_t duty)
{
    s_current_speed = duty;
    /* Duty will be applied on the next direction command */
    ESP_LOGD(TAG, "Speed set to %d", duty);
}

uint8_t motor_get_speed(void)
{
    return s_current_speed;
}

void motor_vacuum_set(bool enable)
{
    gpio_set_level(VACUUM_RELAY_GPIO, enable ? 1 : 0);
    ESP_LOGI(TAG, "Vacuum relay %s", enable ? "ON" : "OFF");
}

/* ─────────────────────────────────────────────────────────
 * Command dispatcher — parses ASCII command from UART
 * ───────────────────────────────────────────────────────── */

void motor_dispatch_command(const char *cmd)
{
    if (!cmd) return;

    ESP_LOGI(TAG, "CMD → %s", cmd);

    if      (strcmp(cmd, "FWD")      == 0) { motor_forward();    }
    else if (strcmp(cmd, "REV")      == 0) { motor_reverse();    }
    else if (strcmp(cmd, "LEFT")     == 0) { motor_turn_left();  }
    else if (strcmp(cmd, "RIGHT")    == 0) { motor_turn_right(); }
    else if (strcmp(cmd, "STOP")     == 0) { motor_stop();       }
    else if (strcmp(cmd, "CLEAN:ON") == 0) { motor_vacuum_set(true);  }
    else if (strcmp(cmd, "CLEAN:OFF")== 0) { motor_vacuum_set(false); }
    else if (strncmp(cmd, "SPD:", 4) == 0) {
        int val = atoi(cmd + 4);
        if (val >= 0 && val <= 255) {
            motor_set_speed((uint8_t)val);
        } else {
            ESP_LOGW(TAG, "SPD out of range: %d", val);
        }
    }
    else {
        ESP_LOGW(TAG, "Unknown command: '%s'", cmd);
    }
}
