#include "as5600.h"
#include "esp_log.h"

#define TAG "AS5600"

#define AS5600_REG_STATUS    0x0B
#define AS5600_REG_RAW_ANGLE 0x0C
#define AS5600_REG_ANGLE     0x0E

static esp_err_t as5600_read_reg(as5600_t *dev, uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(dev->dev_handle, &reg, 1, buf, len, 100);
}

esp_err_t as5600_init(i2c_master_bus_handle_t bus, as5600_t *dev)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = AS5600_I2C_ADDR,
        .scl_speed_hz    = 400000,
    };
    esp_err_t ret = i2c_master_bus_add_device(bus, &cfg, &dev->dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = i2c_master_probe(bus, AS5600_I2C_ADDR, 100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AS5600 not found on bus");
    }
    return ret;
}

esp_err_t as5600_read(as5600_t *dev, as5600_data_t *data)
{
    uint8_t buf[2];
    esp_err_t ret;

    ret = as5600_read_reg(dev, AS5600_REG_STATUS, buf, 1);
    if (ret != ESP_OK) return ret;
    data->status           = buf[0];
    data->magnet_detected  = (buf[0] >> 5) & 1;
    data->magnet_too_weak  = (buf[0] >> 4) & 1;
    data->magnet_too_strong= (buf[0] >> 3) & 1;

    ret = as5600_read_reg(dev, AS5600_REG_RAW_ANGLE, buf, 2);
    if (ret != ESP_OK) return ret;
    data->raw_angle = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];

    ret = as5600_read_reg(dev, AS5600_REG_ANGLE, buf, 2);
    if (ret != ESP_OK) return ret;
    data->angle   = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];
    data->degrees = data->angle * 360.0f / 4096.0f;

    return ESP_OK;
}
