// src/sensors/aht30_reader.cpp
#include "aht30_reader.h"
#include <Wire.h>
#include <Adafruit_AHTX0.h>

// Конфигурация I2C
#define I2C_SDA 21
#define I2C_SCL 22
#define TCA9548_ADDR 0x70

// Массив датчиков AHT30 (по одному на канал мультиплексора)
static Adafruit_AHTX0 aht30_sensors[8];
static bool sensor_present[8] = {false};

// Функция для выбора канала на TCA9548A
void tca_select(uint8_t channel) {
    if (channel > 7) return;
    Wire.beginTransmission(TCA9548_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

void init_aht30_sensors(void) {
    Serial.println("[AHT30] Initializing TCA9548A and AHT30 sensors...");
    
    // Инициализация I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    delay(100);
    
    // Проверка наличия TCA9548A
    Wire.beginTransmission(TCA9548_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("[AHT30] ERROR: TCA9548A not found!");
        return;
    }
    Serial.println("[AHT30] TCA9548A found");
    
    // Инициализация датчиков на каждом канале
    for (int channel = 0; channel < 8; channel++) {
        tca_select(channel);
        delay(10);
        
        if (aht30_sensors[channel].begin()) {
            sensor_present[channel] = true;
            Serial.printf("[AHT30] Sensor found on channel %d\n", channel);
        } else {
            sensor_present[channel] = false;
            Serial.printf("[AHT30] No sensor on channel %d\n", channel);
        }
    }
    
    // Сброс мультиплексора
    Wire.beginTransmission(TCA9548_ADDR);
    Wire.write(0);
    Wire.endTransmission();
    
    Serial.println("[AHT30] Initialization complete");
}

void read_all_aht30(float *temps, float *hums) {
    for (int channel = 0; channel < 8; channel++) {
        temps[channel] = NAN;
        hums[channel] = NAN;
        
        if (!sensor_present[channel]) continue;
        
        tca_select(channel);
        delay(5);
        
        sensors_event_t humidity, temp;
        if (aht30_sensors[channel].getEvent(&humidity, &temp)) {
            temps[channel] = temp.temperature;
            hums[channel] = humidity.relative_humidity;
        } else {
            Serial.printf("[AHT30] Failed to read channel %d\n", channel);
        }
    }
    
    // Сброс мультиплексора
    Wire.beginTransmission(TCA9548_ADDR);
    Wire.write(0);
    Wire.endTransmission();
}
