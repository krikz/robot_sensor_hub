// main/sensors/fan_controller.c
#include "fan_controller.h"
#include <driver/ledc.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <math.h>

static const char *TAG = "FAN";

// Конфигурация GPIO (только 0-21!)
#define FAN1_PIN GPIO_NUM_13  // PWM для первого вентилятора
#define FAN2_PIN GPIO_NUM_14  // PWM для второго вентилятора

// Добавляем пины для тахометров
#define TACHO1_PIN GPIO_NUM_15  // Тахометр первого вентилятора
#define TACHO2_PIN GPIO_NUM_16  // Тахометр второго вентилятора

#define FAN1_CHANNEL LEDC_CHANNEL_0
#define FAN2_CHANNEL LEDC_CHANNEL_1

// Константы для тахометра
#define TACHOMETER_PULSES_PER_REVOLUTION 2  // Два импульса на оборот
#define TACHOMETER_MEASUREMENT_INTERVAL_MS 1000  // Интервал измерения в миллисекундах

// Глобальные переменные для тахометров
static volatile uint32_t pulse_count[2] = {0};
static uint32_t last_rpm[2] = {0};
static int64_t last_measurement_time[2] = {0};

static float fan_speeds[2] = {0.0f};

// Обработчик прерываний для тахометра
static void IRAM_ATTR tachometer_isr_handler(void* arg) {
    int fan_id = (int)arg;
    pulse_count[fan_id]++;
}

void init_fan_controller(void) {
    ESP_LOGI(TAG, "Initializing fan controller...");

    // Настройка PWM
    ledc_timer_config_t timer_conf = {
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 25000,                     // Частота 25 кГц (требование Noctua)
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t ch_conf = {
        .gpio_num = FAN1_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = FAN1_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));

    ch_conf.gpio_num = FAN2_PIN;
    ch_conf.channel = FAN2_CHANNEL;
    ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));

    // Инициализация тахометров
    ESP_LOGI(TAG, "Initializing tachometers...");
    
    // Настройка GPIO для тахометров
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,  // Прерывание по переднему фронту
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << TACHO1_PIN) | (1ULL << TACHO2_PIN),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    // Установка обработчиков прерываний
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(TACHO1_PIN, tachometer_isr_handler, (void*)0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(TACHO2_PIN, tachometer_isr_handler, (void*)1));
    
    // Инициализация времени измерения
    for (int i = 0; i < 2; i++) {
        last_measurement_time[i] = esp_timer_get_time() / 1000;  // В миллисекундах
    }

    ESP_LOGI(TAG, "Fan controller initialized");
}

void set_fan_speed(int fan_id, float speed) {
    if (fan_id < 0 || fan_id > 1) return;
    if (speed < 0.0f) speed = 0.0f;
    if (speed > 1.0f) speed = 1.0f;

    uint32_t duty = (uint32_t)(255 * speed);
    ledc_channel_t channel = (fan_id == 0) ? FAN1_CHANNEL : FAN2_CHANNEL;

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel));

    fan_speeds[fan_id] = speed;
    ESP_LOGD(TAG, "Fan %d speed set to %.2f", fan_id, speed);
}

float get_fan_speed(int fan_id) {
    if (fan_id < 0 || fan_id > 1) return 0.0f;
    return fan_speeds[fan_id];
}

uint32_t get_fan_rpm(int fan_id) {
    if (fan_id < 0 || fan_id > 1) return 0;
    
    int64_t current_time = esp_timer_get_time() / 1000;  // В миллисекундах
    
    // Проверяем, прошел ли достаточный интервал для измерения
    if (current_time - last_measurement_time[fan_id] >= TACHOMETER_MEASUREMENT_INTERVAL_MS) {
        // Расчет RPM: (импульсы / 2) * (60 секунд / интервал в секундах)
        float interval_sec = (current_time - last_measurement_time[fan_id]) / 1000.0f;
        float revolutions = pulse_count[fan_id] / (float)TACHOMETER_PULSES_PER_REVOLUTION;
        last_rpm[fan_id] = (uint32_t)((revolutions / interval_sec) * 60);
        
        // Сброс счетчика импульсов
        pulse_count[fan_id] = 0;
        last_measurement_time[fan_id] = current_time;
    }
    
    return last_rpm[fan_id];
}

bool is_fan_rotating(int fan_id) {
    if (fan_id < 0 || fan_id > 1) return false;
    
    // Вентилятор считается вращающимся, если RPM > 0
    return get_fan_rpm(fan_id) > 0;
}