// src/main.cpp - Robot Sensor Hub firmware for PlatformIO
// Прошивка для сбора данных с датчиков и управления устройствами

#include <Arduino.h>
#include "sensors/aht30_reader.h"
#include "sensors/hx711_reader.h"
#include "sensors/fan_controller.h"

// Константы типов устройств
#define DEVICE_TYPE_AHT30 0
#define DEVICE_TYPE_HX711 1
#define DEVICE_TYPE_FAN   2

// Константы типов данных
#define DATA_TYPE_TEMPERATURE 1
#define DATA_TYPE_HUMIDITY    2
#define DATA_TYPE_WEIGHT      3
#define DATA_TYPE_SPEED       4
#define DATA_TYPE_RPM         5

// Константы команд
#define COMMAND_SET_SPEED     0
#define COMMAND_TARE_SCALE    1

// Интервал публикации данных (мс)
#define PUBLISH_INTERVAL 1000

// Глобальные переменные
unsigned long last_publish_time = 0;

// Структура данных устройства
struct DeviceData {
    uint8_t device_type;
    uint8_t device_id;
    uint8_t data_type;
    float value;
    uint8_t error_code;
};

// Функция для отправки данных в JSON формате
void publish_sensor_data() {
    float temps[8], hums[8];
    read_all_aht30(temps, hums);
    float weight = read_weight();
    
    // Начало JSON объекта
    Serial.println("{");
    Serial.println("  \"devices\": [");
    
    bool first = true;
    
    // Данные с AHT30
    for (int i = 0; i < 8; i++) {
        if (!isnan(temps[i])) {
            if (!first) Serial.println(",");
            Serial.printf("    {\"type\":%d,\"id\":%d,\"data_type\":%d,\"value\":%.2f,\"error\":0}", 
                         DEVICE_TYPE_AHT30, i, DATA_TYPE_TEMPERATURE, temps[i]);
            first = false;
            
            Serial.println(",");
            Serial.printf("    {\"type\":%d,\"id\":%d,\"data_type\":%d,\"value\":%.2f,\"error\":0}", 
                         DEVICE_TYPE_AHT30, i, DATA_TYPE_HUMIDITY, hums[i]);
        }
    }
    
    // Данные с HX711
    if (!isnan(weight)) {
        if (!first) Serial.println(",");
        Serial.printf("    {\"type\":%d,\"id\":0,\"data_type\":%d,\"value\":%.2f,\"error\":0}", 
                     DEVICE_TYPE_HX711, DATA_TYPE_WEIGHT, weight);
        first = false;
    }
    
    // Данные с вентиляторов
    for (int i = 0; i < 2; i++) {
        if (!first) Serial.println(",");
        Serial.printf("    {\"type\":%d,\"id\":%d,\"data_type\":%d,\"value\":%.2f,\"error\":0}", 
                     DEVICE_TYPE_FAN, i, DATA_TYPE_SPEED, get_fan_speed(i));
        
        Serial.println(",");
        Serial.printf("    {\"type\":%d,\"id\":%d,\"data_type\":%d,\"value\":%u,\"error\":0}", 
                     DEVICE_TYPE_FAN, i, DATA_TYPE_RPM, get_fan_rpm(i));
        first = false;
    }
    
    Serial.println();
    Serial.println("  ]");
    Serial.println("}");
}

// Функция обработки команд через Serial
void process_serial_command() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        // Формат команды: TYPE,ID,COMMAND,PARAM
        // Например: 2,0,0,0.75 - установить скорость вентилятора 0 на 75%
        
        int firstComma = command.indexOf(',');
        int secondComma = command.indexOf(',', firstComma + 1);
        int thirdComma = command.indexOf(',', secondComma + 1);
        
        if (firstComma > 0 && secondComma > 0 && thirdComma > 0) {
            uint8_t device_type = command.substring(0, firstComma).toInt();
            uint8_t device_id = command.substring(firstComma + 1, secondComma).toInt();
            uint8_t cmd = command.substring(secondComma + 1, thirdComma).toInt();
            float param = command.substring(thirdComma + 1).toFloat();
            
            Serial.printf("[CMD] Received: type=%d, id=%d, cmd=%d, param=%.2f\n", 
                         device_type, device_id, cmd, param);
            
            // Обработка команд
            if (device_type == DEVICE_TYPE_FAN && cmd == COMMAND_SET_SPEED) {
                if (device_id < 2) {
                    set_fan_speed(device_id, param);
                    Serial.printf("[CMD] Fan %d speed set to %.2f\n", device_id, param);
                }
            } else if (device_type == DEVICE_TYPE_HX711 && cmd == COMMAND_TARE_SCALE) {
                tare_scale();
                Serial.println("[CMD] Scale tared");
            } else {
                Serial.println("[CMD] Unknown command");
            }
        }
    }
}

void setup() {
    // Инициализация Serial
    Serial.begin(115200);
    delay(1000);
    
    Serial.println();
    Serial.println("=================================");
    Serial.println("  Robot Sensor Hub v2.0");
    Serial.println("  PlatformIO / Arduino Framework");
    Serial.println("=================================");
    Serial.println();
    
    // Инициализация датчиков
    Serial.println("[INIT] Initializing sensors...");
    init_aht30_sensors();
    init_hx711();
    init_fan_controller();
    
    Serial.println("[INIT] Initialization complete!");
    Serial.println();
    Serial.println("Command format: TYPE,ID,CMD,PARAM");
    Serial.println("  Set fan speed: 2,0,0,0.75 (fan 0, 75%)");
    Serial.println("  Tare scale: 1,0,1,0");
    Serial.println();
    
    last_publish_time = millis();
}

void loop() {
    unsigned long current_time = millis();
    
    // Публикация данных с датчиков
    if (current_time - last_publish_time >= PUBLISH_INTERVAL) {
        publish_sensor_data();
        last_publish_time = current_time;
    }
    
    // Обработка команд
    process_serial_command();
    
    delay(1);  // Minimal delay to prevent CPU hogging
}
