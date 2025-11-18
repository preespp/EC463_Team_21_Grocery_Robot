#include <stdio.h>
#include <string.h>
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_SLAVE_ADDR          0x08
#define I2C_SLAVE_SDA           4
#define I2C_SLAVE_SCL           5
#define I2C_SLAVE_PORT          I2C_NUM_0

#define JOINT_COUNT             5

// Servo pins
static const int SERVO_PINS[JOINT_COUNT] = {18, 19, 21, 34, 35};

// Servo limits
#define MIN_US 500
#define MAX_US 2500
#define SERVO_FREQ 50        // 50 Hz
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE

// GLOBAL STATE
static float target_angles[JOINT_COUNT] = {0};
static int current_angle[JOINT_COUNT] = {0};

static int map_deg_to_us(int deg)
{
    if (deg < 0) deg = 0;
    if (deg > 270) deg = 270;
    return MIN_US + ((MAX_US - MIN_US) * deg) / 270;
}

// Convert microseconds to LEDC duty
static uint32_t us_to_duty(int us)
{
    const int resolution_bits = 16; // high resolution
    uint32_t max_duty = (1 << resolution_bits) - 1;

    float period_us = 1000000.0f / SERVO_FREQ;   // 20,000 us
    return (uint32_t)((us / period_us) * max_duty);
}

// Smooth motion identical to your Arduino logic
static void move_smooth_single(int idx, int target)
{
    int cur = current_angle[idx];

    if (target < 0) target = 0;
    if (target > 270) target = 270;

    int dir = (target > cur) ? 1 : -1;
    int v = 1;
    int vMax = 10;
    int vMin = 1;
    int dt = 6; // ms

    while (cur != target)
    {
        int rem = abs(target - cur);
        if (rem > 40) v = (v + 1 < vMax ? v + 1 : vMax);
        else          v = (v - 1 > vMin ? v - 1 : vMin);

        cur += dir * (rem < v ? rem : v);

        int us = map_deg_to_us(cur);
        uint32_t duty = us_to_duty(us);
        ledc_set_duty(LEDC_MODE, idx, duty);
        ledc_update_duty(LEDC_MODE, idx);

        current_angle[idx] = cur;
        vTaskDelay(pdMS_TO_TICKS(dt));
    }
}

static void i2c_slave_task(void *arg)
{
    const int expected_len = JOINT_COUNT * sizeof(float);
    uint8_t data[32];

    while (1)
    {
        int len = i2c_slave_read_buffer(I2C_SLAVE_PORT, data, sizeof(data), 20 / portTICK_PERIOD_MS);
        if (len >= expected_len)
        {
            memcpy(target_angles, data, expected_len);

            printf("I2C RX angles: ");
            for (int i = 0; i < JOINT_COUNT; i++)
                printf("%.1f ", target_angles[i]);
            printf("\n");
        }
    }
}

static void servo_task(void *arg)
{
    while (1)
    {
        for (int i = 0; i < JOINT_COUNT; i++)
        {
            int tgt = (int)target_angles[i];
            if (tgt != current_angle[i])
            {
                move_smooth_single(i, tgt);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static void init_i2c_slave(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_SLAVE,
        .sda_io_num = I2C_SLAVE_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SLAVE_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .slave = {
            .slave_addr = I2C_SLAVE_ADDR
        }
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_SLAVE_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_SLAVE_PORT, I2C_MODE_SLAVE,
                                       1024, 1024, 0));
}

static void init_servos(void)
{
    ledc_timer_config_t timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_TIMER_16_BIT,
        .freq_hz          = SERVO_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    for (int i = 0; i < JOINT_COUNT; i++)
    {
        ledc_channel_config_t ch = {
            .gpio_num = SERVO_PINS[i],
            .speed_mode = LEDC_MODE,
            .channel = i,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER,
            .duty = 0,
            .hpoint = 0
        };
        ledc_channel_config(&ch);

        // Move to 0° at boot
        int us = map_deg_to_us(0);
        uint32_t duty = us_to_duty(us);
        ledc_set_duty(LEDC_MODE, i, duty);
        ledc_update_duty(LEDC_MODE, i);
        current_angle[i] = 0;
    }
}

void app_main(void)
{
    printf("\n=== ESP-IDF ROBOT ARM I2C SERVO CONTROLLER ===\n");

    init_i2c_slave();
    init_servos();

    // Start I2C receive + servo control tasks
    xTaskCreate(i2c_slave_task, "i2c_slave_task", 4096, NULL, 5, NULL);
    xTaskCreate(servo_task, "servo_task", 4096, NULL, 5, NULL);
}
