#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_timer.h"

#define LED_GPIO GPIO_NUM_2
#define BATTERY_INDICATOR_GPIO GPIO_NUM_4
#define BUTTON_LED_GPIO GPIO_NUM_25
#define BUTTON_PLUS_GPIO GPIO_NUM_13
#define BUTTON_MINUS_GPIO GPIO_NUM_27
#define BUTTON_BATTERY_CHECK_GPIO GPIO_NUM_14
#define FADE_TIME (3000)
#define SHOW_BATTERY_STATUS_TIME (3000)
#define MIN_TAP_INTERVAL_MCS (200000)
#define RESET_TAP_COUNT_INTERVAL_MCS (1000000)

SemaphoreHandle_t countingSem;
nvs_handle_t nvsHandle;

void duty_set(uint32_t duty);

void button_led_level_set(uint32_t buttonLedLevel);

void pwm_set(uint32_t duty);

void pwm_set_with_fade(uint32_t duty);

static bool cb_ledc_fade_end_event(const ledc_cb_param_t *param, void *user_arg);

void app_main(void) {
    // Init plus btn
    gpio_pad_select_gpio(BUTTON_PLUS_GPIO);
    gpio_set_direction(BUTTON_PLUS_GPIO, GPIO_MODE_INPUT);
    gpio_pullup_en(BUTTON_PLUS_GPIO);
    gpio_pulldown_dis(BUTTON_PLUS_GPIO);

    // Init minus btn
    gpio_pad_select_gpio(BUTTON_MINUS_GPIO);
    gpio_set_direction(BUTTON_MINUS_GPIO, GPIO_MODE_INPUT);
    gpio_pullup_en(BUTTON_MINUS_GPIO);
    gpio_pulldown_dis(BUTTON_MINUS_GPIO);

    //Init battery check button
    gpio_pad_select_gpio(BUTTON_BATTERY_CHECK_GPIO);
    gpio_set_direction(BUTTON_BATTERY_CHECK_GPIO, GPIO_MODE_INPUT);
    gpio_pullup_en(BUTTON_BATTERY_CHECK_GPIO);
    gpio_pulldown_dis(BUTTON_BATTERY_CHECK_GPIO);

    gpio_pad_select_gpio(BATTERY_INDICATOR_GPIO);
    gpio_set_direction(BATTERY_INDICATOR_GPIO, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(BUTTON_LED_GPIO);
    gpio_set_direction(BUTTON_LED_GPIO, GPIO_MODE_OUTPUT);

    // Init timer
    ledc_timer_config_t ledc_timer = {0};
    ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_timer.timer_num = LEDC_TIMER_1;
    ledc_timer.freq_hz = 10000;
    ledc_timer.duty_resolution = LEDC_TIMER_8_BIT;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Init pwm
    ledc_channel_config_t ledc_channel = {0};
    ledc_channel.gpio_num = LED_GPIO;
    ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_channel.channel = LEDC_CHANNEL_1;
    ledc_channel.timer_sel = LEDC_TIMER_1;
    ledc_channel.duty = 0;
    ledc_channel.hpoint = 0;
    ledc_channel.flags.output_invert = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // Init fade
    ledc_fade_func_install(0);
    ledc_cbs_t callbacks = {
            .fade_cb = cb_ledc_fade_end_event
    };
    countingSem = xSemaphoreCreateCounting(1, 0);
    ledc_cb_register(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, &callbacks, (void *) countingSem);

    // Init nvs
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = nvs_open("storage", NVS_READWRITE, &nvsHandle);
    ESP_ERROR_CHECK(err);

    // Set saved button led level
    uint8_t buttonLedLevel = 0;
    err = nvs_get_u8(nvsHandle, "buttonLedLevel", &buttonLedLevel);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        buttonLedLevel = 1;
    }
    gpio_set_level(BUTTON_LED_GPIO, buttonLedLevel);

    // Set saved duty
    uint32_t duty = 0;
    err = nvs_get_u32(nvsHandle, "duty", &duty);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        duty = 100;
    }
    pwm_set_with_fade(duty);

    uint8_t secretTapCount = 0;
    uint8_t wasLvlDown = 1;
    uint64_t time = 0;
    while (true) {
        if (gpio_get_level(BUTTON_PLUS_GPIO) != 0) {
            if (duty < 255) {
                duty++;
            }

            duty_set(duty);
        }

        if (gpio_get_level(BUTTON_MINUS_GPIO) != 0) {
            if (duty > 0) {
                duty--;
            }

            duty_set(duty);
        }


        if (gpio_get_level(BUTTON_BATTERY_CHECK_GPIO) != 0) {
            if (esp_timer_get_time() - time > MIN_TAP_INTERVAL_MCS && wasLvlDown == 1 ) {
                secretTapCount += 1;
                wasLvlDown = 0;
                time = esp_timer_get_time();
            }
        } else {
            if (esp_timer_get_time() - time > MIN_TAP_INTERVAL_MCS) {
                wasLvlDown = 1;
            }
        }

        if (esp_timer_get_time() - time > RESET_TAP_COUNT_INTERVAL_MCS ) {
            if (secretTapCount == 1) {
                gpio_set_level(BATTERY_INDICATOR_GPIO, 1);

                vTaskDelay(SHOW_BATTERY_STATUS_TIME / portTICK_PERIOD_MS);

                gpio_set_level(BATTERY_INDICATOR_GPIO, 0);
            } else if (secretTapCount == 2) {
                buttonLedLevel = buttonLedLevel == 1 ? 0 : 1;

                gpio_set_level(BUTTON_LED_GPIO, buttonLedLevel);
                nvs_set_u8(nvsHandle, "buttonLedLevel", buttonLedLevel);
            }

            secretTapCount = 0;
        }
    }
}

static bool cb_ledc_fade_end_event(const ledc_cb_param_t *param, void *user_arg) {
    portBASE_TYPE taskAwoken = pdFALSE;

    if (param->event == LEDC_FADE_END_EVT) {
        SemaphoreHandle_t countingSem = (SemaphoreHandle_t) user_arg;
        xSemaphoreGiveFromISR(countingSem, &taskAwoken);
    }

    return (taskAwoken == pdTRUE);
}

void duty_set(uint32_t duty) {
    nvs_set_u32(nvsHandle, "duty", duty);
    pwm_set(duty);
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void button_led_level_set(uint32_t buttonLedLevel) {
    nvs_set_u32(nvsHandle, "buttonLedLevel", buttonLedLevel);
    gpio_set_level(BUTTON_LED_GPIO, buttonLedLevel);
}

void pwm_set(uint32_t duty) {
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
}


void pwm_set_with_fade(uint32_t duty) {
    ledc_set_fade_with_time(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, duty, FADE_TIME);
    ledc_fade_start(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, LEDC_FADE_NO_WAIT);
    xSemaphoreTake(countingSem, portMAX_DELAY);
}
