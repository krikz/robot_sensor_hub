// publishers.cpp
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/temperature.h>
#include <sensor_msgs/msg/relative_humidity.h>
#include <diagnostic_msgs/msg/diagnostic_array.h>

#include "config.h"

// Внешние данные
extern float temperatures[4];
extern float humidities[4];
extern float weight_value;
extern bool sensor_status[4];
extern bool hx711_ok;
extern float get_fan_speed(int fan_id);

// Паблишеры
rcl_publisher_t temp_pubs[4];
rcl_publisher_t humidity_pubs[4];
rcl_publisher_t weight_pub;
rcl_publisher_t fan_telem_pubs[2];
rcl_publisher_t diagnostics_pub;

// Сообщения
sensor_msgs__msg__Temperature temp_msgs[4];
sensor_msgs__msg__RelativeHumidity humidity_msgs[4];
std_msgs__msg__Float32 weight_msg;
std_msgs__msg__Float32 fan_telem_msgs[2];
diagnostic_msgs__msg__DiagnosticArray diagnostics_msg;
diagnostic_msgs__msg__DiagnosticStatus diag_status[6]; // AHT30 x4 + HX711 + System

void init_publishers(rcl_node_t *node) {
  for (int i = 0; i < NUM_AHT30_SENSORS; i++) {
    rcl_publisher_init(
      &temp_pubs[i],
      node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
      ("sensor/temperature_" + String(i)).c_str(),
      NULL
    );
    rcl_publisher_init(
      &humidity_pubs[i],
      node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, RelativeHumidity),
      ("sensor/humidity_" + String(i)).c_str(),
      NULL
    );

    // Инициализация сообщений
    temp_msgs[i].header.frame_id = ("aht30_" + String(i)).c_str();
    humidity_msgs[i].header.frame_id = ("aht30_" + String(i)).c_str();
  }

  rcl_publisher_init(
    &weight_pub,
    node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "sensor/weight",
    NULL
  );

  for (int i = 0; i < 2; i++) {
    rcl_publisher_init(
      &fan_telem_pubs[i],
      node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      ("fan/fan" + String(i+1) + "/telemetry").c_str(),
      NULL
    );
  }

  rcl_publisher_init(
    &diagnostics_pub,
    node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticArray),
    "diagnostics",
    NULL
  );

  // Инициализация diagnostics
  diagnostics_msg.status.data = diag_status;
  diagnostics_msg.status.capacity = 6;
  diagnostics_msg.status.size = 0;
}

void publish_sensor_data(rcl_node_t *node, rcl_timer_t *timer, int64_t last_call_time) {
  (void)timer; (void)last_call_time;

  // Публикация температуры и влажности
  for (int i = 0; i < NUM_AHT30_SENSORS; i++) {
    if (!isnan(temperatures[i])) {
      temp_msgs[i].header.stamp = rclcpp::Clock().now();
      temp_msgs[i].temperature = temperatures[i];
      rcl_publish(&temp_pubs[i], &temp_msgs[i], NULL);
    }

    if (!isnan(humidities[i])) {
      humidity_msgs[i].header.stamp = rclcpp::Clock().now();
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
  publish_diagnostics(node);
}

void publish_diagnostics(rcl_node_t *node) {
  diagnostics_msg.status.size = 0;

  // AHT30 датчики
  for (int i = 0; i < NUM_AHT30_SENSORS; i++) {
    auto &s = diag_status[diagnostics_msg.status.size++];
    s.name = ("AHT30 Sensor " + String(i)).c_str();
    if (sensor_status[i]) {
      s.level = diagnostic_msgs__msg__DiagnosticStatus__OK;
      s.message = "OK";
    } else {
      s.level = diagnostic_msgs__msg__DiagnosticStatus__ERROR;
      s.message = "No Response";
    }
  }

  // HX711
  auto &hx = diag_status[diagnostics_msg.status.size++];
  hx.name = "Weight Sensor";
  if (hx711_ok) {
    hx.level = diagnostic_msgs__msg__DiagnosticStatus__OK;
    hx.message = "OK";
  } else {
    hx.level = diagnostic_msgs__msg__DiagnosticStatus__ERROR;
    hx.message = "Not Responding";
  }

  // System
  auto &sys = diag_status[diagnostics_msg.status.size++];
  sys.name = "System";
  sys.level = diagnostic_msgs__msg__DiagnosticStatus__OK;
  sys.message = "Nominal";
  for (int i = 0; i < diagnostics_msg.status.size - 1; i++) {
    if (diag_status[i].level != diagnostic_msgs__msg__DiagnosticStatus__OK) {
      sys.level = diagnostic_msgs__msg__DiagnosticStatus__WARN;
      sys.message = "Some sensors offline";
      break;
    }
  }

  diagnostics_msg.header.stamp = rclcpp::Clock().now();
  rcl_publish(&diagnostics_pub, &diagnostics_msg, NULL);
}