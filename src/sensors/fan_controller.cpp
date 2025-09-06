// fan_controller.cpp
#include <Arduino.h>
#include "config.h"

// PWM каналы
const int FAN_CHANNELS[] = { FAN_PWM_CHANNEL_1, FAN_PWM_CHANNEL_2 };
const int FAN_PINS[] = { FAN1_PWM_PIN, FAN2_PWM_PIN };
const int FAN_RESOLUTION = FAN_PWM_RESOLUTION;
const int FAN_FREQ = FAN_PWM_FREQ;

// Скорость от 0.0 до 1.0
float fan_speeds[2] = {0.0, 0.0};

void init_fan_controller() {
  for (int i = 0; i < 2; i++) {
    ledcSetup(FAN_CHANNELS[i], FAN_FREQ, FAN_RESOLUTION);
    ledcAttachPin(FAN_PINS[i], FAN_CHANNELS[i]);
    ledcWrite(FAN_CHANNELS[i], 0);  // Выключить
    Serial.printf("✅ Fan %d: PWM on GPIO%d\n", i+1, FAN_PINS[i]);
  }
}

void set_fan_speed(int fan_id, float speed) {
  if (fan_id < 0 || fan_id >= 2) return;
  speed = constrain(speed, 0.0, 1.0);
  fan_speeds[fan_id] = speed;

  // Переводим 0.0–1.0 → 0–255 (для 8-битного ШИМ)
  int duty = (int)(speed * ((1 << FAN_RESOLUTION) - 1));
  ledcWrite(FAN_CHANNELS[fan_id], duty);

  Serial.printf("💨 Fan %d: %.1f%% (%d/%d)\n", fan_id+1, speed*100, duty, (1 << FAN_RESOLUTION)-1);
}

float get_fan_speed(int fan_id) {
  if (fan_id < 0 || fan_id >= 2) return 0.0;
  return fan_speeds[fan_id];
}