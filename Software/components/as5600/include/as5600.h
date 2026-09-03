#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#define AS5600_I2C_ADDR 0x36

typedef struct {
    i2c_master_dev_handle_t dev_handle;
} as5600_t;

typedef struct {
    uint16_t raw_angle;        // 12-bit raw angle (0–4095)
    uint16_t angle;            // 12-bit filtered angle (0–4095)
    float    degrees;          // filtered angle in degrees (0.0–359.91)
    uint8_t  status;           // raw STATUS register byte
    bool     magnet_detected;  // MD bit: magnet in range
    bool     magnet_too_weak;  // ML bit: magnet too far / weak
    bool     magnet_too_strong;// MH bit: magnet too close / strong
} as5600_data_t;

/**
 * @brief Add the AS5600 to an existing I2C master bus and verify it responds.
 *
 * @param bus  Initialised I2C master bus handle.
 * @param dev  Output device struct to populate.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t as5600_init(i2c_master_bus_handle_t bus, as5600_t *dev);

/**
 * @brief Read status, raw angle, and filtered angle from the AS5600.
 *
 * @param dev   Initialised device struct.
 * @param data  Output data struct to populate.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t as5600_read(as5600_t *dev, as5600_data_t *data);
