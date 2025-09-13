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
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include <i2cdev.h> // Для i2cdev_init()
#include "sensors/aht30_reader.h"
#include "sensors/hx711_reader.h"
#include "sensors/fan_controller.h"
#include <math.h>
#include <robot_sensor_hub_msg/msg/device_data.h>
#include <robot_sensor_hub_msg/msg/device_command.h>
#include <robot_sensor_hub_msg/msg/device_snapshot.h>  // Новое сообщение

static const char *TAG = "sensor_hub";

// === Глобальные переменные ===
// Паблишер для снапшота данных
rcl_publisher_t snapshot_pub;

// Подписчик на команды
rcl_subscription_t command_sub;

// Сообщения
robot_sensor_hub_msg__msg__DeviceSnapshot snapshot_msg;
robot_sensor_hub_msg__msg__DeviceCommand command_msg;

// Массив для временного хранения данных
robot_sensor_hub_msg__msg__DeviceData device_data_buffer[11]; // 8 AHT30 + 1 HX711 + 2 FAN

// Объекты ROS2
rclc_executor_t executor;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;

// Макросы из официальных примеров micro-ROS
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// === Константы типов устройств и команд ===
#define DEVICE_TYPE_AHT30 0
#define DEVICE_TYPE_HX711 1
#define DEVICE_TYPE_FAN   2

#define DATA_TYPE_TEMPERATURE 1
#define DATA_TYPE_HUMIDITY    2
#define DATA_TYPE_WEIGHT      3
#define DATA_TYPE_SPEED       4

#define COMMAND_SET_SPEED     0
#define COMMAND_TARE_SCALE    1


// === Callback таймера (публикация данных) ===
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) last_call_time;
    if (timer != NULL) {
        float temps[8], hums[8];
        read_all_aht30(temps, hums);
        float weight = read_weight();
        int device_count = 0;
        
        // === Сбор данных с AHT30 ===
        for (int i = 0; i < 8; i++) {
            if (!isnan(temps[i])) {
                robot_sensor_hub_msg__msg__DeviceData__init(&device_data_buffer[device_count]);
                device_data_buffer[device_count].device_type = DEVICE_TYPE_AHT30;
                device_data_buffer[device_count].device_id = i;
                device_data_buffer[device_count].data_type = DATA_TYPE_TEMPERATURE;
                device_data_buffer[device_count].value = temps[i];
                device_data_buffer[device_count].error_code = 0;
                device_count++;
            }
            
            if (!isnan(hums[i])) {
                robot_sensor_hub_msg__msg__DeviceData__init(&device_data_buffer[device_count]);
                device_data_buffer[device_count].device_type = DEVICE_TYPE_AHT30;
                device_data_buffer[device_count].device_id = i;
                device_data_buffer[device_count].data_type = DATA_TYPE_HUMIDITY;
                device_data_buffer[device_count].value = hums[i];
                device_data_buffer[device_count].error_code = 0;
                device_count++;
            }
        }

        // === Сбор данных с HX711 ===
        if (!isnan(weight)) {
            robot_sensor_hub_msg__msg__DeviceData__init(&device_data_buffer[device_count]);
            device_data_buffer[device_count].device_type = DEVICE_TYPE_HX711;
            device_data_buffer[device_count].device_id = 0;
            device_data_buffer[device_count].data_type = DATA_TYPE_WEIGHT;
            device_data_buffer[device_count].value = weight;
            device_data_buffer[device_count].error_code = 0;
            device_count++;
        }

        // === Сбор данных с кулеров ===
        for (int i = 0; i < 2; i++) {
            robot_sensor_hub_msg__msg__DeviceData__init(&device_data_buffer[device_count]);
            device_data_buffer[device_count].device_type = DEVICE_TYPE_FAN;
            device_data_buffer[device_count].device_id = i;
            device_data_buffer[device_count].data_type = DATA_TYPE_SPEED;
            device_data_buffer[device_count].value = get_fan_speed(i);
            device_data_buffer[device_count].error_code = 0;
            device_count++;
        }
        
        // === Публикация снапшота ===
        // Инициализируем снапшот
        robot_sensor_hub_msg__msg__DeviceSnapshot__init(&snapshot_msg);
        
        // Устанавливаем количество устройств
        snapshot_msg.devices.size = device_count;
        snapshot_msg.devices.capacity = 11; // Максимальное количество
        snapshot_msg.devices.data = device_data_buffer;
        
        // Устанавливаем временну́ю метку
        rcl_time_point_value_t now_ns;
        rcl_ret_t ret = rcl_clock_get_now(&(support.clock), &now_ns);
        if (ret != RCL_RET_OK) {
            ESP_LOGE(TAG, "Failed to get current time");
        }
        
        // Публикуем снапшот
        RCSOFTCHECK(rcl_publish(&snapshot_pub, &snapshot_msg, NULL));
        ESP_LOGI(TAG, "Published snapshot with %d devices", device_count);
    }
}


// === Callback для команд (приём команд) ===
void command_callback(const void * msgin)
{
    const robot_sensor_hub_msg__msg__DeviceCommand * cmd = (const robot_sensor_hub_msg__msg__DeviceCommand *)msgin;

    ESP_LOGI(TAG, "Received command: type=%d, id=%d, cmd=%d", 
             cmd->device_type, cmd->device_id, cmd->command_code);

    switch (cmd->device_type) {
        case DEVICE_TYPE_FAN:
            if (cmd->device_id < 2 && cmd->command_code == COMMAND_SET_SPEED) {
                set_fan_speed(cmd->device_id, cmd->param_1);
                ESP_LOGI(TAG, "Fan %d speed set to %.2f", cmd->device_id, cmd->param_1);
            } else {
                ESP_LOGW(TAG, "Unknown fan command or ID");
            }
            break;

        case DEVICE_TYPE_HX711:
            if (cmd->device_id == 0 && cmd->command_code == COMMAND_TARE_SCALE) {
                tare_scale();
                ESP_LOGI(TAG, "HX711 tared");
            } else {
                ESP_LOGW(TAG, "Unknown HX711 command or ID");
            }
            break;

        default:
            ESP_LOGW(TAG, "Unknown device type: %d", cmd->device_type);
            break;
    }
}

// === Основная задача micro-ROS ===
void micro_ros_task(void * arg)
{
    ESP_LOGI(TAG, "Starting micro-ROS task...");

    // 1. Инициализация драйвера i2cdev
    ESP_LOGI(TAG, "Initializing i2cdev driver...");
    if (i2cdev_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize i2cdev");
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "i2cdev driver initialized.");

    // 2. Инициализация датчиков
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

    // 4. Инициализация support
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // 5. Создание ноды
    node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "sensor_hub", "", &support));
    ESP_LOGI(TAG, "micro-ROS node created");

    // 6. Создание паблишера (снапшот)
    RCCHECK(rclc_publisher_init_default(
        &snapshot_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(robot_sensor_hub_msg, msg, DeviceSnapshot),
        "device/snapshot"));

    // 7. Создание подписчика (команды)
    RCCHECK(rclc_subscription_init_default(
        &command_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(robot_sensor_hub_msg, msg, DeviceCommand),
        "device/command"));

    // 8. Таймер (1000 мс)
    timer = rcl_get_zero_initialized_timer();
    const uint64_t timer_period_ms = 1000;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_period_ms),
        timer_callback));


    ESP_LOGI(TAG, "Waiting for micro-ROS agent...");
    int connected = 0;
    while (!connected) {
        if (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) {
            ESP_LOGI(TAG, "Connected to micro-ROS agent!");
            connected = 1;
        } else {
            ESP_LOGI(TAG, "Waiting for micro-ROS agent...");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }


    // 9. Executor
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator)); // 2 - хватит для таймера и подписчика
    unsigned int rcl_wait_timeout = 1000;
    RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

    // Добавление таймера
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    // Добавление подписчика
    RCCHECK(rclc_executor_add_subscription(&executor, &command_sub, &command_msg, command_callback, ON_NEW_DATA));

    ESP_LOGI(TAG, "Ready. Spinning executor...");
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(100000); // 100ms
    }

    // Очистка (не достигается)
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
            "uros_task",
            32768,
            NULL,
            5,
            NULL);
    // app_main больше ничего не делает
}