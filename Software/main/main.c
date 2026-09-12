#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "esp_system.h"
#include "as5600.h"
#include "l293d.h"
#include "can.h"

/* ── Hardware (Waveshare ESP32-C3-Zero) ─────────────────────── */
#define I2C_SDA_GPIO    8
#define I2C_SCL_GPIO    9
#define MOTOR_IN1_GPIO  6
#define MOTOR_IN2_GPIO  7
/* L293D EN is hardwired to logic 1; speed is PWM'd on IN1/IN2. */

/* ── PID tuning ─────────────────────────────────────────────── */
#define PID_PERIOD_MS     10          /* control loop period      */
#define PID_DT            (PID_PERIOD_MS / 1000.0f)
#define KP                0.8f
#define KI                0.3f
#define KD                0.05f
#define INTEGRAL_LIMIT    50.0f       /* anti-windup clamp        */
#define DEADBAND_DEG      1.0f        /* stop inside this window  */
#define MIN_SPEED         40          /* minimum PWM when driving */

/* Status line cadence. Slower than the old 10 Hz display refresh so the
 * console stays readable while a target is being typed. */
#define STATUS_PERIOD_MS  1000
#define STATUS_TICKS      (STATUS_PERIOD_MS / PID_PERIOD_MS)

/* ── Globals ────────────────────────────────────────────────── */
static as5600_t        g_sensor;
static l293d_t         g_motor;
static bool             g_sensor_available;
static bool             g_motor_available;

static SemaphoreHandle_t g_target_mutex;
static float             g_target_deg = 0.0f;

/* ── PID helpers ────────────────────────────────────────────── */
typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_meas;
    float integral_limit;
} pid_t;

/* Wrap error into [-180, 180] to handle the 0/360 boundary */
static float wrap_error(float e, bool over180)
{
    while (e >  180.0f) e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    return e;
}

/* Derivative-on-measurement to avoid kick on setpoint change */
static float pid_update(pid_t *pid, float setpoint, float meas, float dt)
{
    //float error = wrap_error(setpoint - meas);
    float error = (setpoint - meas);

    pid->integral += error * dt;
    if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    float derivative = -(meas - pid->prev_meas) / dt;
    pid->prev_meas = meas;

    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}

/* ── PID / status task ──────────────────────────────────────── */
static void pid_task(void *arg)
{
    pid_t pid = {
        .kp = KP, .ki = KI, .kd = KD,
        .integral_limit = INTEGRAL_LIMIT,
    };
    uint8_t get_Position_Command_Tick =0;
    int status_tick = 0;
    TickType_t last_wake = xTaskGetTickCount();
    uint16_t temp16=0;

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PID_PERIOD_MS));
        if(get_Position_Command_Tick++>10) {
            get_Position_Command_Tick=0;
            temp16=can_get_PositionCommand();
            if(temp16!=CAN_POSITION_INVALID) {
                xSemaphoreTake(g_target_mutex, portMAX_DELAY);
                g_target_deg = (float)temp16 /10.0f;
                xSemaphoreGive(g_target_mutex);
            }
        } 
        as5600_data_t d;
        if (!g_sensor_available || as5600_read(&g_sensor, &d) != ESP_OK) {
            if (g_motor_available) l293d_set_speed(&g_motor, 0);
            continue;
        }
        if(d.degrees>180.0f){
            as5600_set_high_range(true);  
        }else if(d.degrees<140.0f){
            as5600_set_high_range(false);   
        }

        can_set_PositionActual((int16_t)(d.degrees * 10.0f)); /* send the current position of the motor */

        xSemaphoreTake(g_target_mutex, portMAX_DELAY);
        float target = g_target_deg;
        xSemaphoreGive(g_target_mutex);

        //float error = wrap_error(target - d.degrees);
        float error = (target - d.degrees);
        int speed = 0;

        if (fabsf(error) > DEADBAND_DEG) {
            float output = pid_update(&pid, target, d.degrees, PID_DT);
            if (output >  100.0f) output =  100.0f;
            if (output < -100.0f) output = -100.0f;
            /* enforce minimum drive to overcome stiction */
            if (output > 0.0f && output < MIN_SPEED) output = MIN_SPEED;
            if (output < 0.0f && output > -MIN_SPEED) output = -MIN_SPEED;
            speed = (int)output;
        } else {
            pid.integral = 0.0f;   /* reset integrator at rest */
        }

        if (g_motor_available) l293d_set_speed(&g_motor, speed);

        if (++status_tick >= STATUS_TICKS) {
            status_tick = 0;
            printf("Pos: %6.2f deg  Tgt: %6.2f deg  Err: %+7.2f deg  Spd: %+4d%%\n",
                   d.degrees, target, error, speed);
        }
    }
}

/* ── Console task ───────────────────────────────────────────── */
static void console_task(void *arg)
{
    char buf[32];
    int  pos = 0;

    printf("PID position controller ready.\n");
    printf("Enter target angle in degrees (0 - 359.99):\n");

    while (1) {
        int c = getchar();
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* Echo character back */
        putchar(c);
        fflush(stdout);

        if (c == '\n' || c == '\r') {
            buf[pos] = '\0';
            pos = 0;

            if (buf[0] == '\0') continue;

            float val;
            if (sscanf(buf, "%f", &val) != 1) {
                printf("\nInvalid input. Enter a number, e.g. 90.0\n");
                continue;
            }

            if (val < 0.0f) val = 0.0f;
            if (val > 315.0f) val = 315.0f;

            xSemaphoreTake(g_target_mutex, portMAX_DELAY);
            g_target_deg = val;
            xSemaphoreGive(g_target_mutex);

            printf("\nTarget -> %.2f deg\n", val);
        } else if (pos < (int)sizeof(buf) - 1) {
            buf[pos++] = (char)c;
        }
    }
}

/* ── Entry point ────────────────────────────────────────────── */
void app_main(void)
{
    uint8_t dontRunMotor=0;
    printf("Reset reason: %d\n", esp_reset_reason());

    /* I2C bus */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = I2C_SDA_GPIO,
        .scl_io_num        = I2C_SCL_GPIO,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus = NULL;
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &bus);
    if (ret == ESP_OK) {
        ret = as5600_init(bus, &g_sensor);
    }
    g_sensor_available = (ret == ESP_OK);
    if (!g_sensor_available) {
        printf("AS5600 unavailable motor output disabled.\n");
        dontRunMotor=1;
    }

    l293d_config_t motor_cfg = {
        .in1_gpio    = MOTOR_IN1_GPIO,
        .in2_gpio    = MOTOR_IN2_GPIO,
        .in1_channel = LEDC_CHANNEL_0,
        .in2_channel = LEDC_CHANNEL_1,
        .ledc_timer  = LEDC_TIMER_0,
    };
    ret = l293d_init(&motor_cfg, &g_motor);
    g_motor_available = (ret == ESP_OK);
    if (!g_motor_available) {
        printf("Motor driver unavailable motor output disabled.\n");
        dontRunMotor=1;
    }

    g_target_mutex = xSemaphoreCreateMutex();
    if(!dontRunMotor){
        xTaskCreate(pid_task,     "pid",     4096, NULL, 5, NULL);
    }
    xTaskCreate(console_task, "console", 4096, NULL, 3, NULL);
    xTaskCreate(can_rx_task, "can_rx", 3072, NULL, 6, NULL);
    xTaskCreate(can_tx_task, "can_tx", 3072, NULL, 5, NULL);
}
