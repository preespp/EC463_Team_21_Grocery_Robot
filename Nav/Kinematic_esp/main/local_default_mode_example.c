#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"

/**
 * ESP32 mecanum base - ALWAYS HIGH VERSION
 * - Pinout unchanged relative to the main firmware
 * - Sets every motor channel to 100% duty, constant forward (respects inversion flags)
 * - No command handling or movement patterns
 */

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

static const char *TAG = "mecanum_always_high";

/* --------------------------- Motor channel config -------------------------- */

typedef struct {
    gpio_num_t pwm;
    gpio_num_t in1;
    gpio_num_t in2;
    ledc_channel_t ledc_channel;
    bool inverted;
} motor_channel_t;

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

/* --------------------------- Function declarations ------------------------- */
static esp_err_t motors_init(void);
static void drive_full_forward(const motor_channel_t *chan);

/* ------------------------------- app_main ---------------------------------- */

void app_main(void)
{
    ESP_ERROR_CHECK(motors_init());
    ESP_LOGI(TAG, "Driving all motors 100%% duty, forward (always high).");

    for (int i = 0; i < WHEEL_COUNT; ++i) {
        drive_full_forward(&s_motor_channels[i]);
    }

    // Keep the task alive; nothing else to do.
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ---------------------------- Motor init & I/O ----------------------------- */

static esp_err_t motors_init(void)
{
    // Configure LEDC timer for PWM
    ledc_timer_config_t timer_cfg = {
        .speed_mode       = LEDC_TIMER_MODE,
        .timer_num        = LEDC_TIMER_ID,
        .duty_resolution  = LEDC_TIMER_RES,
        .freq_hz          = LEDC_PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    // Configure each motor channel: IN1/IN2 as outputs, PWM as LEDC channel
    for (int i = 0; i < WHEEL_COUNT; ++i) {
        const motor_channel_t *chan = &s_motor_channels[i];

        gpio_config_t io_cfg = {
            .pin_bit_mask = (1ULL << chan->in1) | (1ULL << chan->in2),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&io_cfg));
        ESP_ERROR_CHECK(gpio_set_level(chan->in1, 0));
        ESP_ERROR_CHECK(gpio_set_level(chan->in2, 0));

        ledc_channel_config_t ch_cfg = {
            .gpio_num   = chan->pwm,
            .speed_mode = LEDC_TIMER_MODE,
            .channel    = chan->ledc_channel,
            .timer_sel  = LEDC_TIMER_ID,
            .duty       = 0,
            .hpoint     = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));
    }

    ESP_LOGI(TAG, "Motors initialized with pinout unchanged.");
    return ESP_OK;
}

static void drive_full_forward(const motor_channel_t *chan)
{
    bool forward = true;
    if (chan->inverted) {
        forward = !forward;
    }

    // Direction
    gpio_set_level(chan->in1, forward ? 1 : 0);
    gpio_set_level(chan->in2, forward ? 0 : 1);

    // PWM at 100% duty
    (void)ledc_set_duty(LEDC_TIMER_MODE, chan->ledc_channel, LEDC_MAX_DUTY);
    (void)ledc_update_duty(LEDC_TIMER_MODE, chan->ledc_channel);
}
