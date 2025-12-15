#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

/* ------------------------------ Log Tag ----------------------------------- */
static const char *TAG = "servo_i2c_controller";

/* ------------------------------ Servo PWM ---------------------------------- */
#define SERVO_TIMER_MODE       LEDC_LOW_SPEED_MODE
#define SERVO_TIMER_ID         LEDC_TIMER_0
#define SERVO_TIMER_RES        LEDC_TIMER_16_BIT   // Higher resolution for servos
#define SERVO_PWM_FREQ_HZ      50                  // 50Hz servo
#define SERVO_MAX_DUTY         ((1U << SERVO_TIMER_RES) - 1U)

#define SERVO_MIN_US           500
#define SERVO_MAX_US           2500
#define SERVO_PERIOD_US        20000   // 20ms = 50Hz

/* ------------------------- Servo GPIO assignments -------------------------- */
/*  CHANGE THESE TO YOUR REAL SERVO PINS  */
#define SERVO_1_GPIO    GPIO_NUM_25
#define SERVO_2_GPIO    GPIO_NUM_26
#define SERVO_3_GPIO    GPIO_NUM_27
#define SERVO_4_GPIO    GPIO_NUM_32
#define SERVO_5_GPIO    GPIO_NUM_33

/* --------------------------- Servo channel struct -------------------------- */
typedef struct {
    gpio_num_t gpio;
    ledc_channel_t channel;
} servo_channel_t;

enum {
    SERVO_1 = 0,
    SERVO_2,
    SERVO_3,
    SERVO_4,
    SERVO_5,
    SERVO_COUNT
};

static const servo_channel_t s_servo_channels[SERVO_COUNT] = {
    [SERVO_1] = { SERVO_1_GPIO, LEDC_CHANNEL_0 },
    [SERVO_2] = { SERVO_2_GPIO, LEDC_CHANNEL_1 },
    [SERVO_3] = { SERVO_3_GPIO, LEDC_CHANNEL_2 },
    [SERVO_4] = { SERVO_4_GPIO, LEDC_CHANNEL_3 },
    [SERVO_5] = { SERVO_5_GPIO, LEDC_CHANNEL_4 },
};

/* ------------------------------ I2C Slave ---------------------------------- */
#define I2C_SLAVE_NUM           I2C_NUM_0
#define I2C_SLAVE_SDA_IO        GPIO_NUM_8
#define I2C_SLAVE_SCL_IO        GPIO_NUM_9
#define I2C_SLAVE_ADDR          0x08

/* Buffer for 5 float angles received from Jetson */
static float s_servo_targets[SERVO_COUNT] = {0};

/* ---------------------- Utility: angle → microseconds ---------------------- */
static inline int angle_to_us(float angle)
{
    if (angle < 0) angle = 0;
    if (angle > 270) angle = 270;
    return SERVO_MIN_US +
           (int)(angle * (SERVO_MAX_US - SERVO_MIN_US) / 270.0f);
}

/* ---------------------- Utility: microseconds → duty ----------------------- */
static inline uint32_t us_to_duty(int us)
{
    return (uint32_t)(((float)us / SERVO_PERIOD_US) * SERVO_MAX_DUTY);
}

/* ----------------------------- Servo Write --------------------------------- */
static void servo_write_angle(int id, float angle)
{
    int us = angle_to_us(angle);
    uint32_t duty = us_to_duty(us);

    ledc_set_duty(SERVO_TIMER_MODE, s_servo_channels[id].channel, duty);
    ledc_update_duty(SERVO_TIMER_MODE, s_servo_channels[id].channel);
}

/* ------------------------------- Init Servos ------------------------------- */
static esp_err_t servo_init(void)
{
    // Configure LEDC timer
    ledc_timer_config_t timer_cfg = {
        .speed_mode       = SERVO_TIMER_MODE,
        .timer_num        = SERVO_TIMER_ID,
        .duty_resolution  = SERVO_TIMER_RES,
        .freq_hz          = SERVO_PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    // Configure all servo channels
    for (int i = 0; i < SERVO_COUNT; ++i) {
        ledc_channel_config_t ch_cfg = {
            .gpio_num   = s_servo_channels[i].gpio,
            .speed_mode = SERVO_TIMER_MODE,
            .channel    = s_servo_channels[i].channel,
            .timer_sel  = SERVO_TIMER_ID,
            .duty       = 0,
            .hpoint     = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));
    }

    ESP_LOGI(TAG, "Servos initialized.");
    return ESP_OK;
}

/* ------------------------------- Init I2C ---------------------------------- */
static esp_err_t i2c_init(void)
{
    i2c_config_t slave_cfg = {
        .mode = I2C_MODE_SLAVE,
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .slave = {
            .slave_addr = I2C_SLAVE_ADDR,
            .addr_10bit_en = 0,
        },
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_SLAVE_NUM, &slave_cfg));
    ESP_ERROR_CHECK(i2c_driver_install(
        I2C_SLAVE_NUM, I2C_MODE_SLAVE,
        256, 256, 0
    ));

    ESP_LOGI(TAG, "I2C slave initialized on addr 0x%02X.", I2C_SLAVE_ADDR);
    return ESP_OK;
}

/* ------------------------------- I2C Task ---------------------------------- */
static void i2c_task(void *arg)
{
    uint8_t rxbuf[20];  // 5 floats * 4 bytes
    const size_t expected = sizeof(rxbuf);

    while (true)
    {
        int len = i2c_slave_read_buffer(I2C_SLAVE_NUM, rxbuf, expected, portMAX_DELAY);

        if (len == expected) {
            memcpy(s_servo_targets, rxbuf, expected);

            ESP_LOGI(TAG, "Angles: %.1f %.1f %.1f %.1f %.1f",
                     s_servo_targets[0],
                     s_servo_targets[1],
                     s_servo_targets[2],
                     s_servo_targets[3],
                     s_servo_targets[4]);

            // Write servos
            for (int i = 0; i < SERVO_COUNT; i++)
                servo_write_angle(i, s_servo_targets[i]);
        }
        else {
            ESP_LOGW(TAG, "Received unexpected length: %d", len);
        }
    }
}

/* -------------------------------- app_main --------------------------------- */
void app_main(void)
{
    ESP_LOGI(TAG, "Starting 5-servo I2C controller");

    ESP_ERROR_CHECK(servo_init());
    ESP_ERROR_CHECK(i2c_init());

    xTaskCreate(i2c_task, "i2c_task", 4096, NULL, 10, NULL);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
