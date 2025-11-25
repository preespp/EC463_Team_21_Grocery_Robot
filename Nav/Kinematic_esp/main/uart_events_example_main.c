#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"

/**
 * ESP32 mecanum base - I2C-only control
 * - Removes UART + local demo paths; only listens for I2C cmd_vel packets
 */

/* ---------------------------- Geometry & limits ---------------------------- */
#define HALF_WIDTH_M            0.1905f   // 7.5 in
#define HALF_LENGTH_M           0.2159f   // 8.5 in
#define MECHANUM_LEVER_ARM      (HALF_WIDTH_M + HALF_LENGTH_M)
#define WHEEL_RADIUS_M          0.0762f   // 6 in diameter -> 0.1524 m => radius 0.0762 m
#define MAX_WHEEL_RAD_PER_SEC   6.8333f   // derived from measured 41 in in 2s @100% duty -> ~0.5207 m/s / 0.0762 m
#define MIN_DUTY_RATIO          0.20f     // minimum duty to overcome static friction

/* ------------------------------ PWM settings ------------------------------- */
#define LEDC_TIMER_MODE         LEDC_LOW_SPEED_MODE
#define LEDC_TIMER_ID           LEDC_TIMER_0
#define LEDC_TIMER_RES          LEDC_TIMER_10_BIT
#define LEDC_PWM_FREQ_HZ        15000
#define LEDC_MAX_DUTY           ((1U << LEDC_TIMER_RES) - 1U)

/* ------------------------- Motor GPIO assignments -------------------------- */
#define FL_PWM_GPIO    GPIO_NUM_25
#define FL_IN1_GPIO    GPIO_NUM_26
#define FL_IN2_GPIO    GPIO_NUM_27

#define FR_PWM_GPIO    GPIO_NUM_14
#define FR_IN1_GPIO    GPIO_NUM_12
#define FR_IN2_GPIO    GPIO_NUM_13

#define RL_PWM_GPIO    GPIO_NUM_33
#define RL_IN1_GPIO    GPIO_NUM_32
#define RL_IN2_GPIO    GPIO_NUM_23

#define RR_PWM_GPIO    GPIO_NUM_19
#define RR_IN1_GPIO    GPIO_NUM_18
#define RR_IN2_GPIO    GPIO_NUM_5

/* ------------------------------- I2C config -------------------------------- */
#define I2C_PORT               I2C_NUM_0
#define I2C_SDA_GPIO           GPIO_NUM_21
#define I2C_SCL_GPIO           GPIO_NUM_22
#define I2C_SLAVE_RX_BUF_LEN   256
#define I2C_SLAVE_TX_BUF_LEN   256
#define I2C_SLAVE_ADDRESS      0x46

/* --------------------------- Control loop timing --------------------------- */
#define MOTOR_CTRL_PERIOD_MS   20           // 50 Hz
#define CMD_TIMEOUT_MS         500

static const char *TAG = "mecanum_ctrl_i2c";

typedef struct {
    float vx;   // m/s
    float vy;   // m/s
    float w;    // rad/s
} cmd_vel_t;

typedef struct {
    float fl;
    float fr;
    float rl;
    float rr;
} wheel_speeds_t;

typedef struct {
    gpio_num_t pwm;
    gpio_num_t in1;
    gpio_num_t in2;
    ledc_channel_t ledc_channel;
    bool inverted;
} motor_channel_t;

typedef struct __attribute__((packed)) {
    float vx;
    float vy;
    float w;
} cmd_vel_packet_t;

enum {
    WHEEL_FL = 0,
    WHEEL_FR,
    WHEEL_RL,
    WHEEL_RR,
    WHEEL_COUNT
};

static const motor_channel_t s_motor_channels[WHEEL_COUNT] = {
    [WHEEL_FL] = {FL_PWM_GPIO, FL_IN1_GPIO, FL_IN2_GPIO, LEDC_CHANNEL_0, false},
    [WHEEL_FR] = {FR_PWM_GPIO, FR_IN1_GPIO, FR_IN2_GPIO, LEDC_CHANNEL_1, true},
    [WHEEL_RL] = {RL_PWM_GPIO, RL_IN1_GPIO, RL_IN2_GPIO, LEDC_CHANNEL_2, false},
    [WHEEL_RR] = {RR_PWM_GPIO, RR_IN1_GPIO, RR_IN2_GPIO, LEDC_CHANNEL_3, true},
};

static SemaphoreHandle_t g_cmd_mutex = NULL;
static cmd_vel_t g_cmd_vel = {0};
static TickType_t g_last_cmd_tick = 0;

/* ---------------------------- Forward declarations ------------------------- */
static esp_err_t motors_init(void);
static esp_err_t i2c_init(void);
static void mecanum_inverse_kinematics(const cmd_vel_t *cmd, wheel_speeds_t *wheels);
static void apply_wheel_commands(const wheel_speeds_t *wheels);
static float clipf(float value, float limit);
static void set_motor_output(const motor_channel_t *chan, float wheel_speed);
static void write_cmd_vel(const cmd_vel_t *cmd);
static bool read_cmd_vel(cmd_vel_t *out, TickType_t *tick);

/* ------------------------------- Tasks ------------------------------------- */
static void motor_control_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(MOTOR_CTRL_PERIOD_MS);

    while (true) {
        cmd_vel_t cmd_local = {0};
        TickType_t last_tick = 0;
        if (read_cmd_vel(&cmd_local, &last_tick)) {
            TickType_t now = xTaskGetTickCount();
            if ((now - last_tick) > pdMS_TO_TICKS(CMD_TIMEOUT_MS)) {
                cmd_local.vx = 0;
                cmd_local.vy = 0;
                cmd_local.w = 0;
            }

            wheel_speeds_t wheels = {0};
            mecanum_inverse_kinematics(&cmd_local, &wheels);
            apply_wheel_commands(&wheels);
        }
        vTaskDelayUntil(&last_wake, period_ticks);
    }
}

static void i2c_rx_task(void *arg)
{
    const size_t expected = sizeof(cmd_vel_packet_t);      // 12 bytes
    uint8_t buf[sizeof(cmd_vel_packet_t) + 1];             // allow SMBus command byte
    while (true) {
        int len = i2c_slave_read_buffer(I2C_PORT, buf, sizeof(buf), pdMS_TO_TICKS(500));
        if (len <= 0) {
            continue;
        }

        const uint8_t *payload = buf;
        if (len == (int)expected + 1) {
            // SMBus write_block_data adds a 1-byte command/register before our 12-byte payload.
            payload = buf + 1;
        } else if (len != (int)expected) {
            // Ignore malformed lengths.
            continue;
        }

        cmd_vel_packet_t packet = {0};
        memcpy(&packet, payload, expected);
        cmd_vel_t cmd = {.vx = packet.vx, .vy = packet.vy, .w = packet.w};
        write_cmd_vel(&cmd);
    }
}

/* ---------------------------- Initialization ------------------------------- */
void app_main(void)
{
    ESP_ERROR_CHECK(motors_init());
    ESP_ERROR_CHECK(i2c_init());

    g_cmd_mutex = xSemaphoreCreateMutex();
    if (!g_cmd_mutex) {
        ESP_LOGE(TAG, "Failed to create command mutex");
        return;
    }
    g_last_cmd_tick = xTaskGetTickCount();

    xTaskCreate(motor_control_task, "motor_control_task", 4096, NULL, configMAX_PRIORITIES - 3, NULL);
    xTaskCreate(i2c_rx_task, "i2c_rx_task", 4096, NULL, 7, NULL);
}

static esp_err_t motors_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_TIMER_MODE,
        .timer_num = LEDC_TIMER_ID,
        .duty_resolution = LEDC_TIMER_RES,
        .freq_hz = LEDC_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    for (int i = 0; i < WHEEL_COUNT; ++i) {
        const motor_channel_t *chan = &s_motor_channels[i];
        gpio_config_t io_cfg = {
            .pin_bit_mask = (1ULL << chan->in1) | (1ULL << chan->in2),
            .mode = GPIO_MODE_OUTPUT,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&io_cfg));
        ESP_ERROR_CHECK(gpio_set_level(chan->in1, 0));
        ESP_ERROR_CHECK(gpio_set_level(chan->in2, 0));

        ledc_channel_config_t ch_cfg = {
            .gpio_num = chan->pwm,
            .speed_mode = LEDC_TIMER_MODE,
            .channel = chan->ledc_channel,
            .timer_sel = LEDC_TIMER_ID,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));
    }
    return ESP_OK;
}

static esp_err_t i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_SLAVE,
        .sda_io_num = I2C_SDA_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SCL_GPIO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .clk_flags = 0,
    };
    conf.slave.slave_addr = I2C_SLAVE_ADDRESS;
    conf.slave.addr_10bit_en = 0;

    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    return i2c_driver_install(I2C_PORT, conf.mode, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0);
}

/* ------------------------------- Helpers ----------------------------------- */
static void mecanum_inverse_kinematics(const cmd_vel_t *cmd, wheel_speeds_t *wheels)
{
    const float vx = cmd->vx;
    const float vy = cmd->vy;
    const float w = cmd->w;
    const float inv_r = 1.0f / WHEEL_RADIUS_M;

    wheels->fl = inv_r * (vx - vy - MECHANUM_LEVER_ARM * w);
    wheels->fr = inv_r * (vx + vy + MECHANUM_LEVER_ARM * w);
    wheels->rl = inv_r * (vx + vy - MECHANUM_LEVER_ARM * w);
    wheels->rr = inv_r * (vx - vy + MECHANUM_LEVER_ARM * w);
}

static void apply_wheel_commands(const wheel_speeds_t *wheels)
{
    set_motor_output(&s_motor_channels[WHEEL_FL], clipf(wheels->fl, MAX_WHEEL_RAD_PER_SEC));
    set_motor_output(&s_motor_channels[WHEEL_FR], clipf(wheels->fr, MAX_WHEEL_RAD_PER_SEC));
    set_motor_output(&s_motor_channels[WHEEL_RL], clipf(wheels->rl, MAX_WHEEL_RAD_PER_SEC));
    set_motor_output(&s_motor_channels[WHEEL_RR], clipf(wheels->rr, MAX_WHEEL_RAD_PER_SEC));
}

static void set_motor_output(const motor_channel_t *chan, float wheel_speed)
{
    bool forward = wheel_speed >= 0.0f;
    if (chan->inverted) {
        forward = !forward;
    }

    float ratio = fminf(fabsf(wheel_speed) / MAX_WHEEL_RAD_PER_SEC, 1.0f);
    if (ratio > 0.0f && ratio < MIN_DUTY_RATIO) {
        ratio = MIN_DUTY_RATIO;
    }
    uint32_t duty = (uint32_t)(ratio * (float)LEDC_MAX_DUTY);
    gpio_set_level(chan->in1, forward ? 1 : 0);
    gpio_set_level(chan->in2, forward ? 0 : 1);
    (void)ledc_set_duty(LEDC_TIMER_MODE, chan->ledc_channel, duty);
    (void)ledc_update_duty(LEDC_TIMER_MODE, chan->ledc_channel);
}

static float clipf(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static void write_cmd_vel(const cmd_vel_t *cmd)
{
    if (!cmd || !g_cmd_mutex) {
        return;
    }
    if (xSemaphoreTake(g_cmd_mutex, portMAX_DELAY) == pdTRUE) {
        g_cmd_vel = *cmd;
        g_last_cmd_tick = xTaskGetTickCount();
        xSemaphoreGive(g_cmd_mutex);
    }
}

static bool read_cmd_vel(cmd_vel_t *out, TickType_t *tick)
{
    if (!g_cmd_mutex) {
        return false;
    }
    bool ok = false;
    if (xSemaphoreTake(g_cmd_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (out) {
            *out = g_cmd_vel;
        }
        if (tick) {
            *tick = g_last_cmd_tick;
        }
        xSemaphoreGive(g_cmd_mutex);
        ok = true;
    }
    return ok;
}
