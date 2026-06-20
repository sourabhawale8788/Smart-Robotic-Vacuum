/*
 * motor_control.h — PWM DC Motor Driver Interface
 * =================================================
 * Platform : ESP32 (ESP-IDF, bare-metal, no Arduino)
 *
 * Uses the LEDC (LED Control) peripheral to generate PWM signals
 * for the L298N H-bridge motor driver.
 *
 * Wiring (L298N × 2):
 *   Bridge 1 (Left motors):   ENA=GPIO18, IN1=GPIO21, IN2=GPIO22
 *   Bridge 2 (Right motors):  ENB=GPIO19, IN3=GPIO23, IN4=GPIO25
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "esp_err.h"
#include <stdint.h>

/* ── GPIO Pin Definitions ─────────────────── */
#define MOTOR_L_PWM_GPIO   18   /* Left  motors enable / speed */
#define MOTOR_R_PWM_GPIO   19   /* Right motors enable / speed */
#define MOTOR_IN1_GPIO     21
#define MOTOR_IN2_GPIO     22
#define MOTOR_IN3_GPIO     23
#define MOTOR_IN4_GPIO     25

/* Vacuum motor relay GPIO */
#define VACUUM_RELAY_GPIO  15

/* ── LEDC Configuration ───────────────────── */
#define MOTOR_PWM_FREQ_HZ  1000   /* 1 kHz PWM frequency */
#define MOTOR_PWM_RES_BITS 8      /* 8-bit resolution → 0–255 duty */

/* ── API ──────────────────────────────────── */

/**
 * @brief  Initialise LEDC peripheral and direction GPIO pins.
 *         Must be called once before any motor_* function.
 * @return ESP_OK on success.
 */
esp_err_t motor_control_init(void);

/** @brief Move forward at current speed. */
void motor_forward(void);

/** @brief Move in reverse at current speed. */
void motor_reverse(void);

/** @brief Pivot left (right side drives forward, left side reverses). */
void motor_turn_left(void);

/** @brief Pivot right (left side drives forward, right side reverses). */
void motor_turn_right(void);

/** @brief Immediate stop — PWM duty = 0, direction pins cleared. */
void motor_stop(void);

/**
 * @brief Emergency stop — identical to motor_stop() but callable from ISR.
 *        Does NOT use any FreeRTOS blocking APIs.
 */
void motor_emergency_stop(void);

/**
 * @brief Set motor speed (PWM duty cycle).
 * @param duty  PWM duty 0–255 (0 = stopped, 255 = full speed).
 */
void motor_set_speed(uint8_t duty);

/** @brief Return current PWM duty cycle (0–255). */
uint8_t motor_get_speed(void);

/**
 * @brief Enable or disable the vacuum motor relay.
 * @param enable  true = vacuum ON, false = vacuum OFF.
 */
void motor_vacuum_set(bool enable);

/**
 * @brief Dispatch a plain-text command string from the UART parser.
 *        Recognised tokens: FWD, REV, LEFT, RIGHT, STOP,
 *                           SPD:<n>, CLEAN:ON, CLEAN:OFF
 * @param cmd  Null-terminated ASCII command string.
 */
void motor_dispatch_command(const char *cmd);

#endif /* MOTOR_CONTROL_H */
