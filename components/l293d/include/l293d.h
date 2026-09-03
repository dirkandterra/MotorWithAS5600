#pragma once

#include "driver/ledc.h"
#include "esp_err.h"

/**
 * L293D dual H-bridge motor driver.
 *
 * Wiring (one channel):
 *   IN1 → PWM (LEDC) channel A
 *   IN2 → PWM (LEDC) channel B
 *   EN  → hardwired to logic 1
 *
 * Speed is sign-magnitude: the active direction pin carries the PWM and the
 * other is held low. Since EN is tied high the bridge is always enabled, so
 * "coast" is produced by driving both inputs low.
 *
 * Speed range: -100 (full reverse) … 0 (stop) … +100 (full forward)
 */

#define L293D_PWM_FREQ_HZ   5000
#define L293D_PWM_RESOLUTION LEDC_TIMER_10_BIT   /* 0–1023 */

typedef struct {
    int              in1_gpio;
    int              in2_gpio;
    ledc_channel_t   in1_channel;
    ledc_channel_t   in2_channel;
    ledc_timer_t     ledc_timer;
} l293d_config_t;

typedef struct {
    l293d_config_t cfg;
} l293d_t;

/**
 * @brief Initialise the L293D driver (LEDC timer + one channel per input).
 */
esp_err_t l293d_init(const l293d_config_t *cfg, l293d_t *dev);

/**
 * @brief Set motor speed and direction.
 *
 * @param dev    Initialised device struct.
 * @param speed  -100 (full reverse) to +100 (full forward), 0 = coast stop.
 */
esp_err_t l293d_set_speed(l293d_t *dev, int speed);

/**
 * @brief Active brake: both inputs driven at full duty.
 */
esp_err_t l293d_brake(l293d_t *dev);

/**
 * @brief Coast stop: both inputs at zero duty.
 */
esp_err_t l293d_stop(l293d_t *dev);
