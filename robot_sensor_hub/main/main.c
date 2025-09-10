// main/main.c

#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <sensor_msgs/msg/temperature.h>
#include <sensor_msgs/msg/relative_humidity.h>
#include <std_msgs/msg/float32.h>
#include <diagnostic_msgs/msg/diagnostic_array.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include <i2cdev.h> // Для i2cdev_init()
#include "sensors/aht30_reader.h"
#include "sensors/hx711_reader.h"
#include "sensors/fan_controller.h"

static const char *TAG = "sensor_hub";

// === Глобальные переменные ===
sensor_msgs__msg__Temperature temp_msgs[8];
sensor_msgs__msg__RelativeHumidity humidity_msgs[8];
std_msgs__msg__Float32 weight_msg;
std_msgs__msg__Float32 fan_telem_msgs[2];
diagnostic_msgs__msg__DiagnosticArray diagnostics_msg;

rcl_publisher_t temp_pubs[8];
rcl_publisher_t humidity_pubs[8];
rcl_publisher_t weight_pub;
rcl_publisher_t fan_telem_pubs[2];
rcl_publisher_t diagnostics_pub;

rclc_executor_t executor;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;

// Объявляем publish_diagnostics ДО первого использования
void publish_diagnostics();

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// === Callback таймера ===
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) last_call_time;
    if (timer != NULL) {
        float temps[8], hums[8];
        read_all_aht30(temps, hums);
        float weight = read_weight();

        for (int i = 0; i < 8; i++) {
            temp_msgs[i].header.stamp.nanosec = 0;
            temp_msgs[i].header.stamp.sec = 0;
            // Исправление 1: Простое присваивание строки, так как __assign недоступна
            temp_msgs[i].header.frame_id.data = "";
            temp_msgs[i].header.frame_id.size = 0;
            temp_msgs[i].header.frame_id.capacity = 0;

            humidity_msgs[i].header.stamp.nanosec = 0;
            humidity_msgs[i].header.stamp.sec = 0;
            humidity_msgs[i].header.frame_id.data = "";
            humidity_msgs[i].header.frame_id.size = 0;
            humidity_msgs[i].header.frame_id.capacity = 0;

            temp_msgs[i].temperature = temps[i];
            humidity_msgs[i].relative_humidity = hums[i];

            RCSOFTCHECK(rcl_publish(&temp_pubs[i], &temp_msgs[i], NULL));
            RCSOFTCHECK(rcl_publish(&humidity_pubs[i], &humidity_msgs[i], NULL));
        }

        weight_msg.data = weight;
        RCSOFTCHECK(rcl_publish(&weight_pub, &weight_msg, NULL));

        for (int i = 0; i < 2; i++) {
            fan_telem_msgs[i].data = get_fan_speed(i);
            RCSOFTCHECK(rcl_publish(&fan_telem_pubs[i], &fan_telem_msgs[i], NULL));
        }

        publish_diagnostics();
    }
}

// === Функция публикации диагностики ===
void publish_diagnostics() {
    // Исправление 2: Получаем время как int64_t
    rcutils_time_point_value_t now_ns = 0;
    // Игнорируем предупреждение, так как это стандартная практика
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-result"
    rcutils_system_time_now(&now_ns);
    #pragma GCC diagnostic pop

    diagnostics_msg.header.stamp.nanosec = now_ns % 1000000000ULL;
    diagnostics_msg.header.stamp.sec = now_ns / 1000000000ULL;

    // Создаем статус диагностики
    diagnostic_msgs__msg__DiagnosticStatus diag_status;
    // Исправление 3: Заполняем поля вручную, без __init и __assign
    diag_status.name.data = "robot_sensor_hub";
    diag_status.name.size = strlen("robot_sensor_hub");
    diag_status.name.capacity = diag_status.name.size + 1;

    diag_status.message.data = "OK";
    diag_status.message.size = strlen("OK");
    diag_status.message.capacity = diag_status.message.size + 1;

    diag_status.level = 0; // DiagnosticStatus::OK

    // Заполняем KeyValue
    diagnostic_msgs__msg__KeyValue kv;
    kv.key.data = "node_state";
    kv.key.size = strlen("node_state");
    kv.key.capacity = kv.key.size + 1;

    kv.value.data = "running";
    kv.value.size = strlen("running");
    kv.value.capacity = kv.value.size + 1;

    // Устанавливаем массив значений
    diag_status.values.data = &kv;
    diag_status.values.size = 1;
    diag_status.values.capacity = 1;

    // Устанавливаем массив статусов
    diagnostics_msg.status.data = &diag_status;
    diagnostics_msg.status.size = 1;
    diagnostics_msg.status.capacity = 1;

    RCSOFTCHECK(rcl_publish(&diagnostics_pub, &diagnostics_msg, NULL));
}

// === Основная задача micro-ROS ===
void micro_ros_task(void * arg)
{
    ESP_LOGI(TAG, "Starting micro-ROS task...");

    ESP_LOGI(TAG, "Initializing i2cdev driver...");
    if (i2cdev_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize i2cdev");
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "i2cdev driver initialized.");

    init_aht30_sensors();
    init_hx711();
    init_fan_controller();

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "sensor_hub", "", &support));
    ESP_LOGI(TAG, "micro-ROS node created");
    vTaskDelay(pdMS_TO_TICKS(1000));
    // 6. Создание паблишеров
    for (int i = 0; i < 8; i++) {
        char topic[64];

        // === Логируем начало инициализации паблишера температуры ===
        ESP_LOGI(TAG, "[PUB_INIT] Creating temperature publisher for channel %d...", i);
        snprintf(topic, sizeof(topic), "sensor/temperature_%d", i);
        ESP_LOGD(TAG, "[PUB_INIT] Topic: '%s'", topic);

        rcl_ret_t rc_temp = rclc_publisher_init_default(
            &temp_pubs[i],
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
            topic
        );

        if (rc_temp != RCL_RET_OK) {
            ESP_LOGE(TAG, "[PUB_INIT] FAILED to create temp_pubs[%d]! Error code: %d", i, (int)rc_temp);
            ESP_LOGE(TAG, "[PUB_INIT] Error message: %s", rcl_get_error_string().str);
            RCCHECK(rc_temp); // Это вызовет vTaskDelete(NULL)
        } else {
            ESP_LOGI(TAG, "[PUB_INIT] SUCCESS: temp_pubs[%d] created.", i);
        }

        // Добавим небольшую задержку между инициализациями
        vTaskDelay(pdMS_TO_TICKS(1000));

        // === Логируем начало инициализации паблишера влажности ===
        ESP_LOGI(TAG, "[PUB_INIT] Creating humidity publisher for channel %d...", i);
        snprintf(topic, sizeof(topic), "sensor/humidity_%d", i);
        ESP_LOGD(TAG, "[PUB_INIT] Topic: '%s'", topic);

        rcl_ret_t rc_hum = rclc_publisher_init_default(
            &humidity_pubs[i],
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, RelativeHumidity),
            topic
        );

        if (rc_hum != RCL_RET_OK) {
            ESP_LOGE(TAG, "[PUB_INIT] FAILED to create humidity_pubs[%d]! Error code: %d", i, (int)rc_hum);
            ESP_LOGE(TAG, "[PUB_INIT] Error message: %s", rcl_get_error_string().str);
            RCCHECK(rc_hum);
        } else {
            ESP_LOGI(TAG, "[PUB_INIT] SUCCESS: humidity_pubs[%d] created.", i);
        }

        // Ещё задержка
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    RCCHECK(rclc_publisher_init_default(&weight_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "sensor/weight"));
    RCCHECK(rclc_publisher_init_default(&fan_telem_pubs[0], &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "fan/fan1/telemetry"));
    RCCHECK(rclc_publisher_init_default(&fan_telem_pubs[1], &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "fan/fan2/telemetry"));
    RCCHECK(rclc_publisher_init_default(&diagnostics_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticArray), "diagnostics"));

    timer = rcl_get_zero_initialized_timer();
    const uint64_t timer_period_ms = 500;
    // Исправление 4: Используем устаревшую, но рабочую функцию
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_period_ms),
        timer_callback));

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
    unsigned int rcl_wait_timeout = 100;
    RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    ESP_LOGI(TAG, "Ready. Spinning executor...");
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(100000);
    }

    RCCHECK(rcl_node_fini(&node));
    RCCHECK(rclc_support_fini(&support));
    vTaskDelete(NULL);
}

// === Входная точка приложения ===
void app_main(void) {
    ESP_LOGI(TAG, "Starting robot_sensor_hub...");

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#else
    ESP_LOGE(TAG, "Network interface not configured in menuconfig");
    return;
#endif

    xTaskCreate(micro_ros_task,
                "micro_ros_task",
                16384,
                NULL,
                5,
                NULL);
}