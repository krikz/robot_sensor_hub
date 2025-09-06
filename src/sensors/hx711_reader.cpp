// hx711_reader.cpp
#include <Arduino.h>
#include <HX711.h>
#include "config.h"

HX711 loadcell;
float calibration_factor = 2280.0;  // Будет обновляться из ROS
bool hx711_ok = false;

void init_hx711() {
  loadcell.begin(HX711_DOUT_PIN, HX711_CLK_PIN);
  loadcell.set_scale(calibration_factor);
  delay(100);

  // Проверка датчика
  float test = loadcell.get_units(5);
  hx711_ok = !isnan(test) && abs(test) < 10000;

  if (hx711_ok) {
    Serial.println("✅ HX711: Initialized");
    Serial.printf("📊 Initial calibration factor: %.2f\n", calibration_factor);
  } else {
    Serial.println("❌ HX711: Not responding");
  }
}

float read_weight() {
  if (!hx711_ok) return NAN;
  float value = loadcell.get_units(10);
  if (isnan(value)) hx711_ok = false;
  return value;
}

// --- ROS Callbacks (будут вызываться из subscribers.cpp) ---

void set_calibration_factor(float factor) {
  if (factor <= 0) {
    Serial.println("⚠️  Calibration factor must be > 0");
    return;
  }
  calibration_factor = factor;
  loadcell.set_scale(calibration_factor);
  Serial.printf("🔧 Calibration factor updated: %.2f\n", calibration_factor);
}

void tare_scale() {
  loadcell.tare(10);
  Serial.println("⚖️  HX711: Tared (zeroed)");
}