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

#define TAG "ESP32_LINEAR_ACTUATOR"

/* ================= I2C CONFIG ================= */

#define I2C_PORT              I2C_NUM_0
#define I2C_SDA_GPIO          GPIO_NUM_21
#define I2C_SCL_GPIO          GPIO_NUM_22
#define I2C_SLAVE_ADDRESS     0x13
#define I2C_RX_BUF_LEN        32
#define I2C_TX_BUF_LEN        32

/* ================= STEPPER PINS ================= */
#define STEP_PIN  GPIO_NUM_18
#define DIR_PIN   GPIO_NUM_19
#define EN_PIN    GPIO_NUM_23

/* ================= MOTION PARAMETERS ================= */
// 200 steps/rev * 16 microstep = 3200
// lead = 5 mm
#define STEPS_PER_MM 640.0f
#define STEP_DELAY_US 800   // speed (adjust later)

/* ================= GLOBAL ================= */

static float g_target_height_mm = 0.0f;
static SemaphoreHandle_t g_motion_mutex;
static uint8_t g_motion_state = 0;

/* ================= I2C INIT ================= */
static esp_err_t i2c_slave_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_SLAVE,
        .sda_io_num = I2C_SDA_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SCL_GPIO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    };

    conf.slave.slave_addr = I2C_SLAVE_ADDRESS;
    conf.slave.addr_10bit_en = 0;

    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));

    return i2c_driver_install(
        I2C_PORT,
        conf.mode,
        I2C_RX_BUF_LEN,
        I2C_TX_BUF_LEN,
        0
    );
}

/* ================= GPIO INIT ================= */
static void stepper_gpio_init(void)
{
    gpio_config_t io = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask =
            (1ULL << STEP_PIN) |
            (1ULL << DIR_PIN)  |
            (1ULL << EN_PIN),
    };
    gpio_config(&io);
    gpio_set_level(EN_PIN, 0); // enable driver
}

static void move_steps(int steps, bool dir)
{
    gpio_set_level(DIR_PIN, dir);

    for (int i = 0; i < steps; i++) {

        gpio_set_level(STEP_PIN, 1);
        ets_delay_us(5);

        gpio_set_level(STEP_PIN, 0);
        ets_delay_us(STEP_DELAY_US);
    }
}

static void i2c_rx_task(void *arg)
{
    uint8_t rx_buf[8];

    while (true) {

        int len = i2c_slave_read_buffer(
            I2C_PORT,
            rx_buf,
            sizeof(rx_buf),
            pdMS_TO_TICKS(1000)
        );

        if (len == sizeof(float)) {
            float height;
            memcpy(&height, rx_buf, sizeof(float));
            ESP_LOGI(TAG, "Received height: %.2f mm", height);
            if (xSemaphoreTake(g_motion_mutex, portMAX_DELAY)) {
                g_target_height_mm = height;
                xSemaphoreGive(g_motion_mutex);
            }
        }
    }
}

static void i2c_tx_task(void *arg)
{
    while (true) {
        i2c_slave_write_buffer(
            I2C_PORT,
            &g_motion_state,
            1,
            pdMS_TO_TICKS(100)
        );
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void motion_task(void *arg)
{
    float last_height = 0;
    while (true) {
        float target;
        if (xSemaphoreTake(g_motion_mutex, portMAX_DELAY)) {
            target = g_target_height_mm;
            xSemaphoreGive(g_motion_mutex);
        }
        if (fabs(target - last_height) > 0.5f) {

            g_motion_state = 1;   // MOVING
            float delta = target - last_height;
            bool dir = (delta > 0);
            int steps = fabs(delta) * STEPS_PER_MM;
            ESP_LOGI(TAG, "Move %.2f mm -> %d steps", delta, steps);
            move_steps(steps, dir);
            last_height = target;
            g_motion_state = 2;   // DONE
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Linear Actuator Controller Start");
    ESP_ERROR_CHECK(i2c_slave_init());
    stepper_gpio_init();

    g_motion_mutex = xSemaphoreCreateMutex();

    xTaskCreate(i2c_rx_task, "i2c_rx", 4096, NULL, 6, NULL);
    xTaskCreate(i2c_tx_task, "i2c_tx", 2048, NULL, 6, NULL);
    xTaskCreate(motion_task, "motion", 4096, NULL, 5, NULL);
}
