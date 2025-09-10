// main/sensors/aht30_reader.c
#include "aht30_reader.h"
#include <aht.h>
#include <tca9548.h>
#include <esp_log.h>
#include <math.h>

static const char *TAG = "AHT30";

// Конфигурация I2C
#define I2C_PORT 0
#define I2C_SDA GPIO_NUM_6   // Был GPIO_NUM_0 -> Изменено на безопасный GPIO6
#define I2C_SCL GPIO_NUM_7   // Был GPIO_NUM_1 -> Изменено на безопасный GPIO7

// Адрес TCA9548A
#define TCA9548_ADDR TCA9548_ADDR_0

// Адрес AHT30
#define AHT30_ADDR AHT_I2C_ADDRESS_GND

// Один экземпляр TCA9548A
static i2c_dev_t tca9548;

// Массив датчиков AHT30 (по одному на канал)
static aht_t aht30_sensors[8];

// Флаги инициализации
static bool sensor_present[8] = {false};

void init_aht30_sensors(void) {
    ESP_LOGI(TAG, "Initializing TCA9548A and AHT30 sensors...");

    memset(&tca9548, 0, sizeof(tca9548));

    ESP_LOGI(TAG, "memset(&tca9548, 0, sizeof(tca9548));");
    
    esp_err_t err = tca9548_init_desc(&tca9548, TCA9548_ADDR, I2C_PORT, I2C_SDA, I2C_SCL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize TCA9548 descriptor: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "TCA9548 descriptor initialized.");

    // КРИТИЧЕСКИЙ ШАГ: Дайте системе время на создание мьютекса
    vTaskDelay(pdMS_TO_TICKS(1000));

    for (int channel = 0; channel < 8; channel++) {        
        err = tca9548_set_channels(&tca9548, 1 << channel);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set TCA9548 channel %d: %s", channel, esp_err_to_name(err));
            continue;
        }

        ESP_LOGI(TAG, "Selected channel %d on TCA9548.", channel);

        aht_t *dev = &aht30_sensors[channel];
        memset(dev, 0, sizeof(aht_t));
        dev->mode = AHT_MODE_NORMAL;
        dev->type = AHT_TYPE_AHT20;

        err = aht_init_desc(dev, AHT30_ADDR, I2C_PORT, I2C_SDA, I2C_SCL);
        ESP_LOGI(TAG, "Attempting to init AHT30 on channel %d...", channel);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "AHT30 not found on channel %d: %s", channel, esp_err_to_name(err));
            continue;
        }

        err = aht_init(dev);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to initialize AHT30 on channel %d: %s", channel, esp_err_to_name(err));
            continue;
        }

        bool calibrated;
        aht_get_status(dev, NULL, &calibrated);
        if (!calibrated) {
            ESP_LOGW(TAG, "AHT30 on channel %d is not calibrated", channel);
        }

        sensor_present[channel] = true;
        ESP_LOGI(TAG, "AHT30 detected on channel %d", channel);
    }

    // Вернём мультиплексор в нейтральное состояние
    tca9548_set_channels(&tca9548, 0);
    ESP_LOGI(TAG, "TCA9548A and AHT30 sensors initialized.");
}

void read_all_aht30(float *temps, float *hums) {
    for (int channel = 0; channel < 8; channel++) {
        temps[channel] = NAN;
        hums[channel] = NAN;

        if (!sensor_present[channel]) continue;

        tca9548_set_channels(&tca9548, 1 << channel);

        float temperature, humidity;
        esp_err_t res = aht_get_data(&aht30_sensors[channel], &temperature, &humidity);
        if (res == ESP_OK) {
            temps[channel] = temperature;
            hums[channel] = humidity;
        } else {
            ESP_LOGE(TAG, "Failed to read AHT30 on channel %d: %s", channel, esp_err_to_name(res));
        }
    }

    tca9548_set_channels(&tca9548, 0);
}