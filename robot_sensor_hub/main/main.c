// main/main.c
#include <stdio.h>
#include <string.h>
#include <math.h>     // Для NAN, isnan
#include <stdlib.h>   // Для malloc, free
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <uros_network_interfaces.h>
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/temperature.h>
#include <sensor_msgs/msg/relative_humidity.h>
#include <diagnostic_msgs/msg/diagnostic_array.h>

#include "sensors/aht30_reader.h"
#include "sensors/hx711_reader.h"
#include "sensors/fan_controller.h"

static const char *TAG = "sensor_hub";

// === Глобальные переменные ===
rcl_node_t node;
rcl_timer_t timer;
rclc_support_t support;
rclc_executor_t executor;

// Паблишеры
rcl_publisher_t temp_pubs[8];
rcl_publisher_t humidity_pubs[8];
rcl_publisher_t weight_pub;
rcl_publisher_t fan_telem_pubs[2];
rcl_publisher_t diagnostics_pub;

// Сообщения
sensor_msgs__msg__Temperature temp_msgs[8];
sensor_msgs__msg__RelativeHumidity humidity_msgs[8];
std_msgs__msg__Float32 weight_msg;
std_msgs__msg__Float32 fan_telem_msgs[2];
diagnostic_msgs__msg__DiagnosticArray diagnostics_msg;
diagnostic_msgs__msg__DiagnosticStatus diag_status[10];

// Данные с датчиков
float temperatures[8] = {NAN};
float humidities[8] = {NAN};
float weight_value = NAN;
bool sensor_status[8] = {false};
bool hx711_ok = false;

// === Прототипы функций ===
void publish_diagnostics(void);

// === Таймер: вызывается каждые 500 мс ===
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)timer; (void)last_call_time;

  // Чтение данных
  read_all_aht30(temperatures, humidities);
  weight_value = read_weight();

  // Получаем текущее время
  int64_t now_ns;  // Важно: int64_t, а не uint64_t
  rcl_ret_t ret = rcutils_system_time_now(&now_ns);
  if (ret != RCL_RET_OK) return;

  // Публикация температуры и влажности
  for (int i = 0; i < 8; i++) {
    if (!isnan(temperatures[i])) {
      temp_msgs[i].header.stamp.nanosec = now_ns % 1000000000LL;
      temp_msgs[i].header.stamp.sec = now_ns / 1000000000LL;
      temp_msgs[i].temperature = temperatures[i];
      rcl_publish(&temp_pubs[i], &temp_msgs[i], NULL);
    }
    if (!isnan(humidities[i])) {
      humidity_msgs[i].header.stamp.nanosec = now_ns % 1000000000LL;
      humidity_msgs[i].header.stamp.sec = now_ns / 1000000000LL;
      humidity_msgs[i].relative_humidity = humidities[i] / 100.0f;
      rcl_publish(&humidity_pubs[i], &humidity_msgs[i], NULL);
    }
  }

  // Публикация веса
  if (!isnan(weight_value)) {
    weight_msg.data = weight_value;
    rcl_publish(&weight_pub, &weight_msg, NULL);
  }

  // Публикация телеметрии кулеров
  for (int i = 0; i < 2; i++) {
    fan_telem_msgs[i].data = get_fan_speed(i);
    rcl_publish(&fan_telem_pubs[i], &fan_telem_msgs[i], NULL);
  }

  // Публикация диагностики
  publish_diagnostics();
}

// === Публикация диагностики ===
void publish_diagnostics(void) {
  int64_t now_ns;
  rcutils_system_time_now(&now_ns);

  diagnostics_msg.header.stamp.nanosec = now_ns % 1000000000LL;
  diagnostics_msg.header.stamp.sec = now_ns / 1000000000LL;

  // Очистка предыдущих сообщений
  for (int i = 0; i < 10; i++) {
    if (diag_status[i].name.data) {
      free(diag_status[i].name.data);
      diag_status[i].name.data = NULL;
    }
    if (diag_status[i].message.data) {
      free(diag_status[i].message.data);
      diag_status[i].message.data = NULL;
    }
  }

  int count = 0;
  // AHT30 датчики
  for (int i = 0; i < 8; i++) {
    diagnostic_msgs__msg__DiagnosticStatus *s = &diag_status[count];
    s->name.data = malloc(64);
    snprintf(s->name.data, 64, "AHT30 Sensor %d", i);
    s->name.size = strlen(s->name.data);
    s->name.capacity = 64;

    if (sensor_status[i]) {
      s->level = 0; // OK
      s->message.data = malloc(16);
      strcpy(s->message.data, "OK");
    } else {
      s->level = 2; // ERROR
      s->message.data = malloc(16);
      strcpy(s->message.data, "No Response");
    }
    s->message.size = strlen(s->message.data);
    s->message.capacity = 16;
    count++;
  }

  // HX711
  diagnostic_msgs__msg__DiagnosticStatus *s = &diag_status[count];
  s->name.data = malloc(64);
  strcpy(s->name.data, "Weight Sensor");
  s->name.size = strlen(s->name.data);
  s->name.capacity = 64;

  if (hx711_ok) {
    s->level = 0;
    s->message.data = malloc(16);
    strcpy(s->message.data, "OK");
  } else {
    s->level = 2;
    s->message.data = malloc(16);
    strcpy(s->message.data, "Not Responding");
  }
  s->message.size = strlen(s->message.data);
  s->message.capacity = 16;
  count++;

  diagnostics_msg.status.data = diag_status;
  diagnostics_msg.status.size = count;
  diagnostics_msg.status.capacity = 10;

  rcl_publish(&diagnostics_pub, &diagnostics_msg, NULL);
}

// === Инициализация и основной цикл ===
void app_main(void) {
  ESP_LOGI(TAG, "Starting robot_sensor_hub...");

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN)
  ESP_ERROR_CHECK(uros_network_interface_initialize());
#elif defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
  ESP_ERROR_CHECK(uros_network_interface_initialize());
#else
  ESP_LOGE(TAG, "Network interface not configured in menuconfig");
  return;
#endif

  // Инициализация датчиков
  init_aht30_sensors();
  init_hx711();
  init_fan_controller();

  // Инициализация micro-ROS
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "sensor_hub", "", &support);
  ESP_LOGI(TAG, "micro-ROS node created");

  // Создание паблишеров
  for (int i = 0; i < 8; i++) {
    char topic[64];
    snprintf(topic, sizeof(topic), "sensor/temperature_%d", i);
    rclc_publisher_init_default(&temp_pubs[i], &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature), topic);
    snprintf(topic, sizeof(topic), "sensor/humidity_%d", i);
    rclc_publisher_init_default(&humidity_pubs[i], &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, RelativeHumidity), topic);
  }

  rclc_publisher_init_default(&weight_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "sensor/weight");
  rclc_publisher_init_default(&fan_telem_pubs[0], &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "fan/fan1/telemetry");
  rclc_publisher_init_default(&fan_telem_pubs[1], &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "fan/fan2/telemetry");
  rclc_publisher_init_default(&diagnostics_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticArray), "diagnostics");

  // Таймер (500 мс)
  const uint64_t timer_timeout = 500;
  rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback, true);

  // Executor
  rclc_executor_init(&executor, &support.context, 5, &allocator);
  rclc_executor_add_timer(&executor, &timer);

  ESP_LOGI(TAG, "Ready. Spinning executor...");
  while (1) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    usleep(10000);
  }
}