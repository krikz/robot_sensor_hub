// main/sensors/fan_controller.c
#include "fan_controller.h"
#include <driver/ledc.h>
#include <esp_log.h>

static const char *TAG = "FAN";

// Конфигурация GPIO (только 0-21!)
#define FAN1_PIN GPIO_NUM_2   // Был GPIO_NUM_12 -> Изменено на GPIO14
#define FAN2_PIN GPIO_NUM_1   // Был GPIO_NUM_13 -> Изменено на GPIO15

#define FAN1_CHANNEL LEDC_CHANNEL_0
#define FAN2_CHANNEL LEDC_CHANNEL_1

static float fan_speeds[2] = {0.0f};

void init_fan_controller(void) {
    ESP_LOGI(TAG, "Initializing fan controller...");

    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 25000
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t ch_conf = {
        .gpio_num = FAN1_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = FAN1_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));

    ch_conf.gpio_num = FAN2_PIN;
    ch_conf.channel = FAN2_CHANNEL;
    ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));

    ESP_LOGI(TAG, "Fan controller initialized");
}

void set_fan_speed(int fan_id, float speed) {
    if (fan_id < 0 || fan_id > 1) return;
    if (speed < 0.0f) speed = 0.0f;
    if (speed > 1.0f) speed = 1.0f;

    uint32_t duty = (uint32_t)(255 * speed);
    ledc_channel_t channel = (fan_id == 0) ? FAN1_CHANNEL : FAN2_CHANNEL;

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));

    fan_speeds[fan_id] = speed;
    ESP_LOGD(TAG, "Fan %d speed set to %.2f", fan_id, speed);
}

float get_fan_speed(int fan_id) {
    if (fan_id < 0 || fan_id > 1) return 0.0f;
    return fan_speeds[fan_id];
}