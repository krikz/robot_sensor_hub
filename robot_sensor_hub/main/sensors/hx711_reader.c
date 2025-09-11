// main/sensors/hx711_reader.c
#include "hx711_reader.h"
#include <hx711.h>
#include <esp_log.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "HX711";

// Конфигурация GPIO (только 0-21!)
#define HX711_DOUT GPIO_NUM_10  // Был GPIO_NUM_2 -> Изменено на безопасный GPIO10
#define HX711_SCK  GPIO_NUM_7  // Был GPIO_NUM_3 -> Изменено на безопасный GPIO11

#define HX711_GAIN HX711_GAIN_A_128
#define AVG_TIMES 10

static float calibration_factor = 1.0f;

static hx711_t hx711_dev = {
    .dout = HX711_DOUT,
    .pd_sck = HX711_SCK,
    .gain = HX711_GAIN
};

void init_hx711(void) {
    ESP_LOGI(TAG, "Initializing HX711...");
    ESP_ERROR_CHECK(hx711_init(&hx711_dev));
    ESP_LOGI(TAG, "HX711 initialized");
}

float read_weight(void) {
    int32_t raw;
    esp_err_t res = hx711_wait(&hx711_dev, 500);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "HX711 not ready: %s", esp_err_to_name(res));
        return NAN;
    }

    res = hx711_read_average(&hx711_dev, AVG_TIMES, &raw);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read HX711: %s", esp_err_to_name(res));
        return NAN;
    }

    return (float)raw / calibration_factor;
}

void set_calibration_factor(float factor) {
    if (factor != 0.0f) {
        calibration_factor = factor;
        ESP_LOGI(TAG, "Calibration factor set to %.2f", factor);
    }
}

void tare_scale(void) {
    int32_t sum = 0;
    int32_t raw;
    bool ready;

    for (int i = 0; i < 10; i++) {
        while (hx711_is_ready(&hx711_dev, &ready) != ESP_OK || !ready) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        hx711_read_data(&hx711_dev, &raw);
        sum += raw;
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    int32_t offset = sum / 10;
    calibration_factor = -offset;
    ESP_LOGI(TAG, "Tare complete. Offset: %" PRId32, offset);
}