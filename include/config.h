// config.h
#pragma once

#include <Arduino.h>

// 📡 ROS2 Topics
#define TOPIC_WEIGHT_CALIBRATE "weight/calibration_factor"
#define TOPIC_WEIGHT_TARE      "weight/tare"
#define TOPIC_WEIGHT           "sensor/weight"

// 🧩 Аппаратные настройки
#define NUM_AHT30_SENSORS      4
#define TCA9548A_ADDR          0x70

// I2C
#define I2C_SDA_PIN            7
#define I2C_SCL_PIN            6

// HX711 (весы)
#define HX711_DOUT_PIN         1
#define HX711_CLK_PIN          2

// Кулера (PWM)
#define FAN1_PWM_PIN           5
#define FAN2_PWM_PIN           4
#define FAN_PWM_FREQ           25000
#define FAN_PWM_RESOLUTION     8
#define FAN_PWM_CHANNEL_1      0
#define FAN_PWM_CHANNEL_2      1

// ⏱️ Период опроса (мс)
#define SENSOR_READ_INTERVAL   500   // Каждые 500 мс