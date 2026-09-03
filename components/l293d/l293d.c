#include "l293d.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include <stdlib.h>  /* abs() */

#define DUTY_MAX  ((1 << L293D_PWM_RESOLUTION) - 1)  /* 1023 */

static esp_err_t set_duty(ledc_channel_t channel, uint32_t duty)
{
    esp_err_t ret;
    ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    if (ret != ESP_OK) return ret;
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

/* Drive the pair in one shot so both inputs are never momentarily high */
static esp_err_t set_pair(const l293d_t *dev, uint32_t in1_duty, uint32_t in2_duty)
{
    esp_err_t ret = set_duty(dev->cfg.in1_channel, in1_duty);
    if (ret != ESP_OK) return ret;
    return set_duty(dev->cfg.in2_channel, in2_duty);
}

esp_err_t l293d_init(const l293d_config_t *cfg, l293d_t *dev)
{
    if (!cfg || !dev) return ESP_ERR_INVALID_ARG;
    dev->cfg = *cfg;

    /* Configure LEDC timer shared by both inputs */
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = L293D_PWM_RESOLUTION,
        .timer_num       = cfg->ledc_timer,
        .freq_hz         = L293D_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    /* One LEDC channel per direction input; EN is hardwired high */
    ledc_channel_config_t in1 = {
        .gpio_num   = cfg->in1_gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = cfg->in1_channel,
        .timer_sel  = cfg->ledc_timer,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&in1));

    ledc_channel_config_t in2 = in1;
    in2.gpio_num = cfg->in2_gpio;
    in2.channel  = cfg->in2_channel;
    ESP_ERROR_CHECK(ledc_channel_config(&in2));

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

    uint32_t duty = (uint32_t)(abs(speed) * DUTY_MAX / 100);

    /* Forward: PWM on IN1, IN2 low.  Reverse: PWM on IN2, IN1 low. */
    return (speed > 0) ? set_pair(dev, duty, 0)
                       : set_pair(dev, 0, duty);
}

esp_err_t l293d_brake(l293d_t *dev)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    return set_pair(dev, DUTY_MAX, DUTY_MAX);
}

esp_err_t l293d_stop(l293d_t *dev)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    return set_pair(dev, 0, 0);
}
