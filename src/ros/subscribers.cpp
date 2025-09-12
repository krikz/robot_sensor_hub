// subscribers.cpp
#include <rcl/rcl.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/empty.h>

#include "config.h"
#include "hx711_reader.h"  // set_calibration_factor, tare_scale
#include "fan_controller.h" // set_fan_speed

// Подписчики
rcl_subscription_t fan1_cmd_sub;
rcl_subscription_t fan2_cmd_sub;
rcl_subscription_t calib_factor_sub;
rcl_subscription_t tare_sub;

// Сообщения
std_msgs__msg__Float32 fan1_cmd_msg;
std_msgs__msg__Float32 fan2_cmd_msg;
std_msgs__msg__Float32 calib_msg;
std_msgs__msg__Empty tare_msg;

// Callbacks
void fan1_speed_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  float speed = constrain(msg->data, 0.0f, 1.0f);
  set_fan_speed(0, speed);
  Serial.printf("🎛️  Fan 1 speed set to %.1f%%\n", speed * 100);
}

void fan2_speed_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  float speed = constrain(msg->data, 0.0f, 1.0f);
  set_fan_speed(1, speed);
  Serial.printf("🎛️  Fan 2 speed set to %.1f%%\n", speed * 100);
}

void calibration_factor_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  if (msg->data > 0) {
    set_calibration_factor(msg->data);
    Serial.printf("🔧 Calibration factor set to: %.2f\n", msg->data);
  } else {
    Serial.println("❌ Invalid calibration factor (must be > 0)");
  }
}

void tare_callback(const void *msgin) {
  (void)msgin;
  tare_scale();
  Serial.println("⚖️  Weight sensor tared (zeroed)");
}

void init_subscribers(rcl_node_t *node) {
  // Fan 1
  rcl_subscription_init(
    &fan1_cmd_sub,
    node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "fan/fan1/speed_cmd",
    NULL
  );

  // Fan 2
  rcl_subscription_init(
    &fan2_cmd_sub,
    node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "fan/fan2/speed_cmd",
    NULL
  );

  // Calibration factor
  rcl_subscription_init(
    &calib_factor_sub,
    node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    TOPIC_WEIGHT_CALIBRATE,
    NULL
  );

  // Tare
  rcl_subscription_init(
    &tare_sub,
    node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
    TOPIC_WEIGHT_TARE,
    NULL
  );
}