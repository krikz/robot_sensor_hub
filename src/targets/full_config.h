#pragma once

// Custom target configuration example
// Copy this file and modify for your hardware setup

// ========================================
// I2C Configuration
// ========================================
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_FREQ 100000

// ========================================
// TCA9548A I2C Multiplexer
// ========================================
#define USE_TCA9548A 1
#define TCA9548A_ADDRESS 0x70

// AHT30 sensor channels - Enable all 8 channels
#define AHT30_CHANNEL_0 1
#define AHT30_CHANNEL_1 1
#define AHT30_CHANNEL_2 1
#define AHT30_CHANNEL_3 1
#define AHT30_CHANNEL_4 1
#define AHT30_CHANNEL_5 1
#define AHT30_CHANNEL_6 1
#define AHT30_CHANNEL_7 1

// ========================================
// HX711 Weight Sensor Configuration
// ========================================
#define USE_HX711 1
#define HX711_DOUT_PIN 18
#define HX711_SCK_PIN 19

// ========================================
// Fan Controller Configuration
// ========================================
#define NUM_FANS 2

// Fan 0
#define FAN0_PWM_PIN 13
#define FAN0_TACHO_PIN 15

// Fan 1
#define FAN1_PWM_PIN 14
#define FAN1_TACHO_PIN 16

// PWM Configuration
#define FAN_PWM_FREQ 25000
#define FAN_PWM_RESOLUTION 8

// ========================================
// Serial Configuration
// ========================================
#define SERIAL_BAUD 115200

// ========================================
// Target Name
// ========================================
#define TARGET_NAME "ESP32_SENSOR_HUB_FULL"
