#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>

#define SSD1306_WIDTH  128
#define SSD1306_HEIGHT  64

typedef struct {
    i2c_master_dev_handle_t dev_handle;
    uint8_t buf[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
} ssd1306_t;

esp_err_t ssd1306_init(i2c_master_bus_handle_t bus, uint8_t addr, ssd1306_t *dev);
void      ssd1306_clear(ssd1306_t *dev);
esp_err_t ssd1306_flush(ssd1306_t *dev);
void      ssd1306_draw_char(ssd1306_t *dev, uint8_t col, uint8_t row, char c);
void      ssd1306_draw_string(ssd1306_t *dev, uint8_t col, uint8_t row, const char *str);
