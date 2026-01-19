// src/sensors/fan_controller.cpp
#include "fan_controller.h"
#include "../target.h"

// Тахометр
#define PULSES_PER_REVOLUTION 2
#define RPM_MEASUREMENT_INTERVAL 1000  // мс

// Глобальные переменные
static volatile uint32_t pulse_count[2] = {0, 0};
static uint32_t last_rpm[2] = {0, 0};
static unsigned long last_measurement_time[2] = {0, 0};
static float fan_speeds[2] = {0.0f, 0.0f};

// Fan pin configuration from target
#if NUM_FANS >= 2
static const uint8_t fan_pwm_pins[2] = {FAN0_PWM_PIN, FAN1_PWM_PIN};
static const uint8_t fan_tacho_pins[2] = {FAN0_TACHO_PIN, FAN1_TACHO_PIN};
#else
static const uint8_t fan_pwm_pins[1] = {FAN0_PWM_PIN};
static const uint8_t fan_tacho_pins[1] = {FAN0_TACHO_PIN};
#endif

// Обработчики прерываний для тахометров
void IRAM_ATTR tacho0_isr() {
    pulse_count[0]++;
}

void IRAM_ATTR tacho1_isr() {
    pulse_count[1]++;
}

void init_fan_controller(void) {
    Serial.printf("[FAN] Initializing %d fan(s) with target: %s\n", NUM_FANS, TARGET_NAME);
    
    // Настройка PWM для вентиляторов
    for (int i = 0; i < NUM_FANS; i++) {
        Serial.printf("[FAN] Fan %d: PWM pin=%d, TACHO pin=%d\n", i, fan_pwm_pins[i], fan_tacho_pins[i]);
        
        ledcSetup(i, FAN_PWM_FREQ, FAN_PWM_RESOLUTION);
        ledcAttachPin(fan_pwm_pins[i], i);
        ledcWrite(i, 0);
        
        // Настройка тахометра
        pinMode(fan_tacho_pins[i], INPUT_PULLUP);
        
        // Инициализация времени измерения
        last_measurement_time[i] = millis();
    }
    
    // Attach interrupts for tachometers
    if (NUM_FANS >= 1) {
        attachInterrupt(digitalPinToInterrupt(fan_tacho_pins[0]), tacho0_isr, RISING);
    }
    if (NUM_FANS >= 2) {
        attachInterrupt(digitalPinToInterrupt(fan_tacho_pins[1]), tacho1_isr, RISING);
    }
    
    Serial.println("[FAN] Fan controller initialized");
}

void set_fan_speed(int fan_id, float speed) {
    if (fan_id < 0 || fan_id >= NUM_FANS) {
        Serial.printf("[FAN] ERROR: Invalid fan_id %d (max %d)\n", fan_id, NUM_FANS-1);
        return;
    }
    
    // Ограничение скорости 0.0-1.0
    if (speed < 0.0f) speed = 0.0f;
    if (speed > 1.0f) speed = 1.0f;
    
    // Преобразование в PWM duty cycle (0-255)
    uint32_t duty = (uint32_t)((1 << FAN_PWM_RESOLUTION) - 1) * speed;
    
    // Установка PWM
    ledcWrite(fan_id, duty);
    
    fan_speeds[fan_id] = speed;
    Serial.printf("[FAN] Fan %d speed set to %.2f%% (duty=%d)\n", fan_id, speed * 100, duty);
}

float get_fan_speed(int fan_id) {
    if (fan_id < 0 || fan_id >= NUM_FANS) {
        Serial.printf("[FAN] ERROR: Invalid fan_id %d\n", fan_id);
        return NAN;  // Use NAN to indicate invalid input
    }
    return fan_speeds[fan_id];
}

uint32_t get_fan_rpm(int fan_id) {
    if (fan_id < 0 || fan_id >= NUM_FANS) return 0;
    
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
    if (fan_id < 0 || fan_id >= NUM_FANS) return false;
    return get_fan_rpm(fan_id) > 0;
}
