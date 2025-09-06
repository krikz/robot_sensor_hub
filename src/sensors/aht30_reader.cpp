// aht30_reader.cpp
#include <Arduino.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_TCA9548A.h>
#include "config.h"

extern Adafruit_AHTX0 aht[4];     // Максимум 4 датчика (можно расширить)
extern Adafruit_TCA9548A tca;
extern bool sensor_status[4];     // Статус каждого AHT30
extern uint8_t num_aht_sensors;

void init_aht30_sensors() {
  tca.begin(TCA9548A_ADDR);
  for (int i = 0; i < num_aht_sensors; i++) {
    tca.select(i);
    delay(10);
    if (aht[i].begin()) {
      sensor_status[i] = true;
      Serial.printf("✅ AHT30 Sensor %d: OK\n", i);
    } else {
      sensor_status[i] = false;
      Serial.printf("❌ AHT30 Sensor %d: Not found\n", i);
    }
  }
}

void read_all_aht30(float *temps, float *hums) {
  for (int i = 0; i < num_aht_sensors; i++) {
    tca.select(i);
    delay(10);
    sensors_event_t humidity, temp;
    if (aht[i].getEvent(&humidity, &temp)) {
      temps[i] = temp.temperature;
      hums[i] = humidity.relative_humidity;
      sensor_status[i] = true;
    } else {
      temps[i] = NAN;
      hums[i] = NAN;
      sensor_status[i] = false;
    }
  }
}