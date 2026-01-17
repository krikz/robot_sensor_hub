// src/sensors/fan_controller.cpp
#include "fan_controller.h"

// Конфигурация GPIO
#define FAN1_PIN 13  // PWM для первого вентилятора
#define FAN2_PIN 14  // PWM для второго вентилятора
#define TACHO1_PIN 15  // Тахометр первого вентилятора
#define TACHO2_PIN 16  // Тахометр второго вентилятора

// PWM настройки
#define PWM_FREQ 25000  // 25 кГц (стандарт для вентиляторов)
#define PWM_RESOLUTION 8  // 8-бит разрешение (0-255)
#define PWM_CHANNEL_FAN1 0
#define PWM_CHANNEL_FAN2 1

// Тахометр
#define PULSES_PER_REVOLUTION 2
#define RPM_MEASUREMENT_INTERVAL 1000  // мс

// Глобальные переменные
static volatile uint32_t pulse_count[2] = {0, 0};
static uint32_t last_rpm[2] = {0, 0};
static unsigned long last_measurement_time[2] = {0, 0};
static float fan_speeds[2] = {0.0f, 0.0f};

// Обработчики прерываний для тахометров
void IRAM_ATTR tacho1_isr() {
    pulse_count[0]++;
}

void IRAM_ATTR tacho2_isr() {
    pulse_count[1]++;
}

void init_fan_controller(void) {
    Serial.println("[FAN] Initializing fan controller...");
    
    // Настройка PWM для обоих вентиляторов
    ledcSetup(PWM_CHANNEL_FAN1, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(FAN1_PIN, PWM_CHANNEL_FAN1);
    ledcWrite(PWM_CHANNEL_FAN1, 0);
    
    ledcSetup(PWM_CHANNEL_FAN2, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(FAN2_PIN, PWM_CHANNEL_FAN2);
    ledcWrite(PWM_CHANNEL_FAN2, 0);
    
    // Настройка тахометров
    pinMode(TACHO1_PIN, INPUT_PULLUP);
    pinMode(TACHO2_PIN, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(TACHO1_PIN), tacho1_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(TACHO2_PIN), tacho2_isr, RISING);
    
    // Инициализация времени измерения
    last_measurement_time[0] = millis();
    last_measurement_time[1] = millis();
    
    Serial.println("[FAN] Fan controller initialized");
}

void set_fan_speed(int fan_id, float speed) {
    if (fan_id < 0 || fan_id > 1) return;
    
    // Ограничение скорости 0.0-1.0
    if (speed < 0.0f) speed = 0.0f;
    if (speed > 1.0f) speed = 1.0f;
    
    // Преобразование в PWM duty cycle (0-255)
    uint32_t duty = (uint32_t)(255 * speed);
    
    // Установка PWM
    uint8_t channel = (fan_id == 0) ? PWM_CHANNEL_FAN1 : PWM_CHANNEL_FAN2;
    ledcWrite(channel, duty);
    
    fan_speeds[fan_id] = speed;
    Serial.printf("[FAN] Fan %d speed set to %.2f%% (duty=%d)\n", fan_id, speed * 100, duty);
}

float get_fan_speed(int fan_id) {
    if (fan_id < 0 || fan_id > 1) {
        Serial.printf("[FAN] ERROR: Invalid fan_id %d\n", fan_id);
        return NAN;  // Use NAN to indicate invalid input
    }
    return fan_speeds[fan_id];
}

uint32_t get_fan_rpm(int fan_id) {
    if (fan_id < 0 || fan_id > 1) return 0;
    
    unsigned long current_time = millis();
    
    // Проверяем, прошел ли достаточный интервал для измерения
    if (current_time - last_measurement_time[fan_id] >= RPM_MEASUREMENT_INTERVAL) {
        // Расчет RPM
        float interval_sec = (current_time - last_measurement_time[fan_id]) / 1000.0f;
        float revolutions = pulse_count[fan_id] / (float)PULSES_PER_REVOLUTION;
        last_rpm[fan_id] = (uint32_t)((revolutions / interval_sec) * 60);
        
        // Сброс счетчика
        pulse_count[fan_id] = 0;
        last_measurement_time[fan_id] = current_time;
    }
    
    return last_rpm[fan_id];
}

bool is_fan_rotating(int fan_id) {
    if (fan_id < 0 || fan_id > 1) return false;
    return get_fan_rpm(fan_id) > 0;
}
