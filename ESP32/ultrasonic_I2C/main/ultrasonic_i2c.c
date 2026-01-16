#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <esp32/rom/ets_sys.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"

#define TAG "ESP32_ULTRASONIC_I2C"

/* ---------- I2C ---------- */
#define I2C_PORT               I2C_NUM_0
#define I2C_SDA_GPIO           GPIO_NUM_21
#define I2C_SCL_GPIO           GPIO_NUM_22
#define I2C_SLAVE_ADDRESS      0x09 // Change Address for each side
#define I2C_SLAVE_RX_BUF_LEN   128
#define I2C_SLAVE_TX_BUF_LEN   128

/* ---------- Ultrasonic GPIO ---------- */
#define US_LEFT_TRIG   GPIO_NUM_4
#define US_LEFT_ECHO   GPIO_NUM_16

#define US_RIGHT_TRIG  GPIO_NUM_17
#define US_RIGHT_ECHO  GPIO_NUM_34

/* ---------- Timing ---------- */
#define ULTRASONIC_PERIOD_MS   50      // 20 Hz
#define MAX_ECHO_TIMEOUT_US    30000   // ~5m max

/* ======================== DATA STRUCTURES ======================== */
typedef struct __attribute__((packed)) {
    float dist_left_m;
    float dist_right_m;
} ultrasonic_packet_t;

/* ======================== GLOBAL STATE ======================== */
static ultrasonic_packet_t g_ultrasonic_data = {0};
static SemaphoreHandle_t g_ultrasonic_mutex;

/* ======================== I2C INIT ======================== */
static esp_err_t i2c_slave_init(void)
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
    return i2c_driver_install(
        I2C_PORT,
        conf.mode,
        I2C_SLAVE_RX_BUF_LEN,
        I2C_SLAVE_TX_BUF_LEN,
        0
    );
}

/* ======================== GPIO INIT ======================== */
static void ultrasonic_gpio_init(void)
{
    gpio_config_t trig_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask =
            (1ULL << US_LEFT_TRIG) |
            (1ULL << US_RIGHT_TRIG),
    };
    gpio_config(&trig_cfg);

    gpio_config_t echo_cfg = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask =
            (1ULL << US_LEFT_ECHO) |
            (1ULL << US_RIGHT_ECHO),
    };
    gpio_config(&echo_cfg);

    gpio_set_level(US_LEFT_TRIG, 0);
    gpio_set_level(US_RIGHT_TRIG, 0);
}

/* ======================== ULTRASONIC MEASUREMENT ======================== */
static float measure_ultrasonic(gpio_num_t trig, gpio_num_t echo)
{
    gpio_set_level(trig, 0);
    ets_delay_us(2);
    gpio_set_level(trig, 1);
    ets_delay_us(10);
    gpio_set_level(trig, 0);

    int64_t start = esp_timer_get_time();
    while (!gpio_get_level(echo)) {
        if (esp_timer_get_time() - start > MAX_ECHO_TIMEOUT_US)
            return -1.0f;
    }

    int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(echo)) {
        if (esp_timer_get_time() - echo_start > MAX_ECHO_TIMEOUT_US)
            return -1.0f;
    }

    int64_t echo_end = esp_timer_get_time();
    float duration_us = (float)(echo_end - echo_start);

    // Distance = time * speed of sound / 2
    return (duration_us * 0.000001f * 343.0f) / 2.0f;
}

/* ======================== TASKS ======================== */
static void ultrasonic_task(void *arg)
{
    while (true) {
        ultrasonic_packet_t local = {0};

        local.dist_left_m  = measure_ultrasonic(US_LEFT_TRIG, US_LEFT_ECHO);
        local.dist_right_m = measure_ultrasonic(US_RIGHT_TRIG, US_RIGHT_ECHO);

        if (xSemaphoreTake(g_ultrasonic_mutex, portMAX_DELAY)) {
            g_ultrasonic_data = local;
            xSemaphoreGive(g_ultrasonic_mutex);
        }

        ESP_LOGI(TAG, "Left: %.2f m | Right: %.2f m",
                 local.dist_left_m, local.dist_right_m);

        vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_PERIOD_MS));
    }
}

static void i2c_tx_task(void *arg)
{
    ultrasonic_packet_t packet;

    while (true) {
        if (xSemaphoreTake(g_ultrasonic_mutex, portMAX_DELAY)) {
            packet = g_ultrasonic_data;
            xSemaphoreGive(g_ultrasonic_mutex);
        }

        i2c_slave_write_buffer(
            I2C_PORT,
            (uint8_t *)&packet,
            sizeof(packet),
            pdMS_TO_TICKS(100)
        );

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void app_main(void)
{
    ESP_LOGI(TAG, "Starting Ultrasonic I2C Slave");

    ESP_ERROR_CHECK(i2c_slave_init());
    ultrasonic_gpio_init();

    g_ultrasonic_mutex = xSemaphoreCreateMutex();
    if (!g_ultrasonic_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    xTaskCreate(ultrasonic_task, "ultrasonic_task", 4096, NULL, 5, NULL);
    xTaskCreate(i2c_tx_task, "i2c_tx_task", 4096, NULL, 6, NULL);
}
