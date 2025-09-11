// main.cpp
// Узел мониторинга состояния робота: температура, влажность, вес, управление кулерами
// Платформа: ESP32-C3 + micro-ROS (UDP over WiFi)
// Проект: robot_sensor_hub
// Автор: Ваше имя

#include <Arduino.h>
#include <Wire.h>

// micro-ROS headers
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Внешние библиотеки
#include <TCA9548.h>
#include <ahtxx.h>
#include <HX711.h>

#include "config.h"

// === Глобальные переменные ===

// TCA9548A и AHT30
TCA9548 tca(TCA9548A_ADDR);
ahtxx_handle_t aht_handles[TCA9548A_CHANNEL_COUNT] = {nullptr};
bool sensor_status[TCA9548A_CHANNEL_COUNT] = {false};
uint8_t active_sensor_count = 0;
int active_sensor_channels[TCA9548A_CHANNEL_COUNT] = { -1 };

// HX711 (весы)
HX711 loadcell;
float calibration_factor = 2280.0; // Будет обновляться через ROS
bool hx711_ok = false;

// Кулера
const int FAN_CHANNELS[] = { FAN_PWM_CHANNEL_1, FAN_PWM_CHANNEL_2 };
const int FAN_PINS[] = { FAN1_PWM_PIN, FAN2_PWM_PIN };
float fan_speeds[2] = {0.0, 0.0};

// Данные с датчиков
float temperatures[TCA9548A_CHANNEL_COUNT] = {NAN};
float humidities[TCA9548A_CHANNEL_COUNT] = {NAN};
float weight_value = NAN;

// micro-ROS объекты
rcl_node_t node;
rcl_timer_t timer;
rclc_support_t support;
rclc_executor_t executor;

// === Прототипы функций (из других файлов) ===
void init_aht30_sensors();
void read_all_aht30(float *temps, float *hums);

void init_hx711();
float read_weight();
void set_calibration_factor(float factor);
void tare_scale();

void init_fan_controller();
void set_fan_speed(int fan_id, float speed);
float get_fan_speed(int fan_id);

void init_publishers(rcl_node_t *node);
void publish_sensor_data(rcl_node_t *node, rcl_timer_t *timer, int64_t last_call_time);
void init_subscribers(rcl_node_t *node);

// === setup() ===
void setup() {
  // Инициализация Serial
  Serial.begin(115200);
  while (!Serial && millis() < 2000);
  Serial.println("\n🚀 robot_sensor_hub: Starting up...");

  // Инициализация I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Serial.println("✅ I2C bus initialized (SDA: GPIO7, SCL: GPIO6)");

  // === Инициализация датчиков ===

  // AHT30 через TCA9548A
  init_aht30_sensors();

  // HX711 (весы)
  init_hx711();

  // Кулера (PWM)
  init_fan_controller();

  // === Инициализация micro-ROS ===
  rcl_ret_t ret;
  rclc_support_init(&support, 0, NULL, &allocator);

  // Создание узла
  rclc_node_init_default(&node, "sensor_hub", "", &support);
  Serial.println("✅ micro-ROS node created: sensor_hub");

  // Инициализация паблишеров и подписчиков
  init_publishers(&node);
  init_subscribers(&node);

  // Таймер опроса датчиков
  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(SENSOR_READ_INTERVAL),
    publish_sensor_data,
    &node
  );

  // Executor (1 таймер + 4 подписчика)
  rclc_executor_init(&executor, &support.context, 5, &allocator);
  rclc_executor_add_timer(&executor, &timer);

  Serial.println("✅ robot_sensor_hub: Ready and running!");
  Serial.printf("📡 Connecting to micro-ROS agent at %s:%d...\n", STRINGIFY(AGENT_IP), AGENT_PORT);
}

// === loop() ===
void loop() {
  // Основной цикл обработки micro-ROS
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

  // Чтение данных с датчиков
  read_all_aht30(temperatures, humidities);
  weight_value = read_weight();
}