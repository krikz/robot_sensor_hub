// aht30_reader.cpp
// Драйвер для работы с несколькими датчиками AHT30 через I2C-мультиплексор TCA9548A
// Автоматическое обнаружение подключённых датчиков по каналам
// Использует библиотеку RobTillaart/TCA9548 и K0I05/esp_ahtxx

#include <Arduino.h>
#include <TCA9548.h>
#include <ahtxx.h>
#include "config.h"

// Внешние переменные (объявлены глобально в main.cpp)
extern TCA9548 tca;
extern ahtxx_handle_t aht_handles[TCA9548A_CHANNEL_COUNT];
extern bool sensor_status[TCA9548A_CHANNEL_COUNT];
extern uint8_t active_sensor_count;
extern int active_sensor_channels[TCA9548A_CHANNEL_COUNT];

// Локальные массивы (определены здесь, но extern в других файлах)
ahtxx_handle_t aht_handles[TCA9548A_CHANNEL_COUNT] = {nullptr};
bool sensor_status[TCA9548A_CHANNEL_COUNT] = {false};
uint8_t active_sensor_count = 0;
int active_sensor_channels[TCA9548A_CHANNEL_COUNT] = { -1 };

/**
 * @brief Инициализация всех датчиков AHT30 на каналах TCA9548A
 * 
 * Проходит по всем 8 каналам мультиплексора, пытается инициализировать AHT30.
 * Сохраняет дескрипторы только для успешно подключённых датчиков.
 * Номер канала = номер датчика в системе (для топиков и диагностики).
 */
void init_aht30_sensors() {
  Serial.println("🔍 Scanning TCA9548A channels for AHT30 sensors...");

  active_sensor_count = 0;

  for (int channel = 0; channel < TCA9548A_CHANNEL_COUNT; channel++) {
    // Переключаемся на канал
    tca.selectChannel(channel);
    delay(10); // Даём время на стабилизацию

    // Конфигурация датчика (AHT30)
    ahtxx_config_t config = I2C_AHT30_CONFIG_DEFAULT;
    esp_err_t err = ahtxx_init(NULL, &config, &aht_handles[channel]);

    if (err == ESP_OK && aht_handles[channel] != nullptr) {
      sensor_status[channel] = true;
      active_sensor_channels[active_sensor_count] = channel;
      active_sensor_count++;
      Serial.printf("✅ AHT30 Sensor on channel %d: Found and initialized\n", channel);
    } else {
      aht_handles[channel] = nullptr;
      sensor_status[channel] = false;
      Serial.printf("❌ AHT30 Sensor on channel %d: Not found (err=%d)\n", channel, err);
    }
  }

  Serial.printf("✅ Initialization complete. %d AHT30 sensor(s) active.\n", active_sensor_count);
}

/**
 * @brief Чтение данных со всех подключённых AHT30
 * 
 * @param temps Массив float-значений температуры (индекс = номер канала)
 * @param hums  Массив float-значений влажности (индекс = номер канала)
 * 
 * Заполняет массивы по индексу канала. Если датчик не отвечает — NaN.
 */
void read_all_aht30(float *temps, float *hums) {
  // Сброс всех значений
  for (int i = 0; i < TCA9548A_CHANNEL_COUNT; i++) {
    temps[i] = NAN;
    hums[i] = NAN;
  }

  for (uint8_t i = 0; i < active_sensor_count; i++) {
    int channel = active_sensor_channels[i];

    tca.selectChannel(channel);
    delay(10);

    float temperature, humidity;
    esp_err_t result = ahtxx_get_measurement(aht_handles[channel], &temperature, &humidity);

    if (result == ESP_OK) {
      temps[channel] = temperature;
      hums[channel] = humidity;
      sensor_status[channel] = true;
    } else {
      temps[channel] = NAN;
      hums[channel] = NAN;
      sensor_status[channel] = false;
      Serial.printf("⚠️  AHT30 Sensor %d: Read failed (%s)\n", channel, esp_err_to_name(result));
    }
  }
}