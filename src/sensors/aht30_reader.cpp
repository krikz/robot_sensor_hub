// src/sensors/aht30_reader.cpp
#include "aht30_reader.h"
#include "../target.h"
#include <Wire.h>
#include <Adafruit_AHTX0.h>

// Array to check which channels should be scanned
static const bool channel_enabled[8] = {
    AHT30_CHANNEL_0,
    AHT30_CHANNEL_1,
    AHT30_CHANNEL_2,
    AHT30_CHANNEL_3,
    AHT30_CHANNEL_4,
    AHT30_CHANNEL_5,
    AHT30_CHANNEL_6,
    AHT30_CHANNEL_7
};

// Массив датчиков AHT30 (по одному на канал мультиплексора)
static Adafruit_AHTX0 aht30_sensors[8];
static bool sensor_present[8] = {false};

// Функция для выбора канала на TCA9548A
void tca_select(uint8_t channel) {
    if (channel > 7) return;
    Wire.beginTransmission(TCA9548A_ADDRESS);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

void init_aht30_sensors(void) {
    Serial.printf("[AHT30] Initializing with target: %s\n", TARGET_NAME);
    
#if USE_TCA9548A
    // Инициализация I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_FREQ);
    delay(100);
    
    // Проверка наличия TCA9548A
    Wire.beginTransmission(TCA9548A_ADDRESS);
    if (Wire.endTransmission() != 0) {
        Serial.println("[AHT30] ERROR: TCA9548A not found!");
        return;
    }
    Serial.printf("[AHT30] TCA9548A found at 0x%02X\n", TCA9548A_ADDRESS);
    
    // Инициализация датчиков только на разрешенных каналах
    for (int channel = 0; channel < 8; channel++) {
        if (!channel_enabled[channel]) {
            Serial.printf("[AHT30] Channel %d disabled in target config\n", channel);
            continue;
        }
        
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
    Wire.beginTransmission(TCA9548A_ADDRESS);
    Wire.write(0);
    Wire.endTransmission();
    
    Serial.println("[AHT30] Initialization complete");
#else
    Serial.println("[AHT30] TCA9548A disabled in target config");
#endif
}

void read_all_aht30(float *temps, float *hums) {
#if USE_TCA9548A
    for (int channel = 0; channel < 8; channel++) {
        temps[channel] = NAN;
        hums[channel] = NAN;
        
        // Skip if channel not enabled in config or sensor not present
        if (!channel_enabled[channel] || !sensor_present[channel]) continue;
        
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
    Wire.beginTransmission(TCA9548A_ADDRESS);
    Wire.write(0);
    Wire.endTransmission();
#else
    // No multiplexer configured
    for (int channel = 0; channel < 8; channel++) {
        temps[channel] = NAN;
        hums[channel] = NAN;
    }
#endif
}
