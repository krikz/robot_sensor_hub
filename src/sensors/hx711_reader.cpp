// src/sensors/hx711_reader.cpp
#include "hx711_reader.h"
#include "../target.h"
#include <HX711.h>

// Глобальный объект HX711
static HX711 scale;
static float calibration_factor = 1.0f;
static bool initialized = false;

void init_hx711(void) {
#if USE_HX711
    Serial.printf("[HX711] Initializing with pins DOUT=%d, SCK=%d...\n", HX711_DOUT_PIN, HX711_SCK_PIN);
    
    scale.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
    
    if (scale.wait_ready_timeout(1000)) {
        Serial.println("[HX711] Sensor ready");
        scale.set_scale(calibration_factor);
        scale.tare();
        initialized = true;
    } else {
        Serial.println("[HX711] ERROR: Sensor not ready!");
        initialized = false;
    }
#else
    Serial.println("[HX711] Disabled in target config");
    initialized = false;
#endif
}

float read_weight(void) {
#if USE_HX711
    if (!initialized) return NAN;
    
    if (scale.wait_ready_timeout(1000)) {
        float weight = scale.get_units(10);  // Среднее из 10 измерений
        return weight;
    } else {
        Serial.println("[HX711] ERROR: Timeout reading weight");
        return NAN;
    }
#else
    return NAN;
#endif
}

void set_calibration_factor(float factor) {
#if USE_HX711
    if (factor == 0.0f) {
        Serial.println("[HX711] WARNING: Cannot set calibration factor to 0");
        return;
    }
    calibration_factor = factor;
    scale.set_scale(factor);
    Serial.printf("[HX711] Calibration factor set to %.2f\n", factor);
#endif
}

void tare_scale(void) {
#if USE_HX711
    if (!initialized) return;
    
    Serial.println("[HX711] Taring scale...");
    scale.tare();
    Serial.println("[HX711] Tare complete");
#endif
}
