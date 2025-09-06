// main.cpp
// Узел мониторинга состояния робота: температура, влажность, вес, управление кулерами
// Платформа: ESP32-C3 + micro-ROS
// Автор: Ваше имя
// Проект: robot_sensor_hub

#include <Arduino.h>
#include <Wire.h>

// micro-ROS headers
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "config.h"

// Внешние модули
extern "C" {
  void init_aht30_sensors();
  void read_all_aht30(float *temps, float *hums);

  void init_hx711();
  float read_weight();

  void init_fan_controller();
  void set_fan_speed(int fan_id, float speed);
  float get_fan_speed(int fan_id);

  void init_publishers(rcl_node_t *node);
  void publish_sensor_data(rcl_node_t *node, rcl_timer_t *timer, int64_t last_call_time);

  void init_subscribers(rcl_node_t *node);
}

// Глобальные данные
float temperatures[NUM_AHT30_SENSORS] = {NAN};
float humidities[NUM_AHT30_SENSORS] = {NAN};
float weight_value = NAN;

// Статус датчиков
bool sensor_status[NUM_AHT30_SENSORS] = {false};
bool hx711_ok = false;
uint8_t num_aht_sensors = NUM_AHT30_SENSORS;

// micro-ROS объекты
rcl_node_t node;
rcl_timer_t timer;
rclc_support_t support;
rclc_executor_t executor;

void setup() {
  // Инициализация Serial
  Serial.begin(115200);
  while (!Serial && millis() < 2000); // Ждём отладочный терминал
  Serial.println("\n🚀 robot_sensor_hub: Starting up...");

  // Инициализация I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Serial.println("✅ I2C bus initialized");

  // Инициализация датчиков
  init_aht30_sensors();
  init_hx711();
  init_fan_controller();

  // Инициализация micro-ROS
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
  Serial.println("📡 Waiting for micro-ROS agent...");
}

void loop() {
  // Основной цикл обработки micro-ROS
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

  // Опционально: ручной опрос (если не через таймер)
  // read_all_aht30(temperatures, humidities);
  // weight_value = read_weight();
}