#include "l293d.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include <stdlib.h>  /* abs() */

#define DUTY_MAX  ((1 << L293D_PWM_RESOLUTION) - 1)  /* 1023 */

static esp_err_t set_duty(const l293d_t *dev, uint32_t duty)
{
    esp_err_t ret;
    ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, dev->cfg.ledc_channel, duty);
    if (ret != ESP_OK) return ret;
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, dev->cfg.ledc_channel);
}

esp_err_t l293d_init(const l293d_config_t *cfg, l293d_t *dev)
{
    if (!cfg || !dev) return ESP_ERR_INVALID_ARG;
    dev->cfg = *cfg;

    /* Configure IN1 and IN2 as outputs */
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << cfg->in1_gpio) | (1ULL << cfg->in2_gpio),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(cfg->in1_gpio, 0);
    gpio_set_level(cfg->in2_gpio, 0);

    /* Configure LEDC timer */
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = L293D_PWM_RESOLUTION,
        .timer_num       = cfg->ledc_timer,
        .freq_hz         = L293D_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    /* Configure LEDC channel on EN pin */
    ledc_channel_config_t channel = {
        .gpio_num   = cfg->en_gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = cfg->ledc_channel,
        .timer_sel  = cfg->ledc_timer,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel));

    return ESP_OK;
}

esp_err_t l293d_set_speed(l293d_t *dev, int speed)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    /* Clamp to [-100, 100] */
    if (speed >  100) speed =  100;
    if (speed < -100) speed = -100;

    if (speed == 0) {
        return l293d_stop(dev);
    }

    if (speed > 0) {
        /* Forward: IN1=1, IN2=0 */
        gpio_set_level(dev->cfg.in1_gpio, 1);
        gpio_set_level(dev->cfg.in2_gpio, 0);
    } else {
        /* Reverse: IN1=0, IN2=1 */
        gpio_set_level(dev->cfg.in1_gpio, 0);
        gpio_set_level(dev->cfg.in2_gpio, 1);
    }

    uint32_t duty = (uint32_t)(abs(speed) * DUTY_MAX / 100);
    return set_duty(dev, duty);
}

esp_err_t l293d_brake(l293d_t *dev)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    gpio_set_level(dev->cfg.in1_gpio, 1);
    gpio_set_level(dev->cfg.in2_gpio, 1);
    return set_duty(dev, DUTY_MAX);
}

esp_err_t l293d_stop(l293d_t *dev)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    gpio_set_level(dev->cfg.in1_gpio, 0);
    gpio_set_level(dev->cfg.in2_gpio, 0);
    return set_duty(dev, 0);
}
