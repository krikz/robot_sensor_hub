// src/main.cpp - Robot Sensor Hub firmware for PlatformIO
// Request-response protocol for Raspberry Pi communication

#include <Arduino.h>
#include "sensors/aht30_reader.h"
#include "sensors/hx711_reader.h"
#include "sensors/fan_controller.h"

// Device types
#define DEVICE_TYPE_AHT30 0
#define DEVICE_TYPE_HX711 1
#define DEVICE_TYPE_FAN   2

// Data types
#define DATA_TYPE_TEMPERATURE 1
#define DATA_TYPE_HUMIDITY    2
#define DATA_TYPE_WEIGHT      3
#define DATA_TYPE_SPEED       4
#define DATA_TYPE_RPM         5

// Request commands
#define CMD_GET_SENSORS     0  // Get list of available sensors
#define CMD_READ_SENSOR     1  // Read specific sensor data
#define CMD_SET_FAN_SPEED   2  // Set fan speed
#define CMD_TARE_SCALE      3  // Tare scale
#define CMD_GET_ALL_DATA    4  // Get all sensor data

// Response codes
#define RESP_OK             0
#define RESP_ERROR          1
#define RESP_INVALID_CMD    2
#define RESP_INVALID_PARAM  3

// Send response in JSON format
void send_response(uint8_t status, const char* message = NULL) {
    Serial.print("{\"status\":");
    Serial.print(status);
    if (message) {
        Serial.print(",\"message\":\"");
        Serial.print(message);
        Serial.print("\"");
    }
    Serial.println("}");
}

// CMD 0: Get list of available sensors and their state
void cmd_get_sensors() {
    Serial.println("{\"status\":0,\"sensors\":[");
    
    // Check AHT30 sensors
    float temps[8], hums[8];
    read_all_aht30(temps, hums);
    bool first = true;
    
    for (int i = 0; i < 8; i++) {
        if (!isnan(temps[i])) {
            if (!first) Serial.println(",");
            Serial.printf("  {\"type\":%d,\"id\":%d,\"name\":\"AHT30\",\"available\":true}", 
                         DEVICE_TYPE_AHT30, i);
            first = false;
        }
    }
    
    // Check HX711
    float weight = read_weight();
    if (!isnan(weight)) {
        if (!first) Serial.println(",");
        Serial.printf("  {\"type\":%d,\"id\":0,\"name\":\"HX711\",\"available\":true}", 
                     DEVICE_TYPE_HX711);
        first = false;
    }
    
    // Fans are always available
    for (int i = 0; i < 2; i++) {
        if (!first) Serial.println(",");
        Serial.printf("  {\"type\":%d,\"id\":%d,\"name\":\"FAN\",\"available\":true}", 
                     DEVICE_TYPE_FAN, i);
        first = false;
    }
    
    Serial.println("\n]}");
}

// CMD 1: Read specific sensor data
void cmd_read_sensor(uint8_t device_type, uint8_t device_id) {
    if (device_type == DEVICE_TYPE_AHT30) {
        if (device_id < 8) {
            float temps[8], hums[8];
            read_all_aht30(temps, hums);
            
            if (!isnan(temps[device_id])) {
                Serial.printf("{\"status\":0,\"type\":%d,\"id\":%d,\"data\":[", 
                             device_type, device_id);
                Serial.printf("{\"data_type\":%d,\"value\":%.2f},", 
                             DATA_TYPE_TEMPERATURE, temps[device_id]);
                Serial.printf("{\"data_type\":%d,\"value\":%.2f}", 
                             DATA_TYPE_HUMIDITY, hums[device_id]);
                Serial.println("]}");
            } else {
                send_response(RESP_ERROR, "Sensor not available");
            }
        } else {
            send_response(RESP_INVALID_PARAM, "Invalid device_id");
        }
    } else if (device_type == DEVICE_TYPE_HX711) {
        float weight = read_weight();
        if (!isnan(weight)) {
            Serial.printf("{\"status\":0,\"type\":%d,\"id\":0,\"data\":[", device_type);
            Serial.printf("{\"data_type\":%d,\"value\":%.2f}", DATA_TYPE_WEIGHT, weight);
            Serial.println("]}");
        } else {
            send_response(RESP_ERROR, "Sensor not available");
        }
    } else if (device_type == DEVICE_TYPE_FAN) {
        if (device_id < 2) {
            float speed = get_fan_speed(device_id);
            uint32_t rpm = get_fan_rpm(device_id);
            
            Serial.printf("{\"status\":0,\"type\":%d,\"id\":%d,\"data\":[", 
                         device_type, device_id);
            Serial.printf("{\"data_type\":%d,\"value\":%.2f},", DATA_TYPE_SPEED, speed);
            Serial.printf("{\"data_type\":%d,\"value\":%u}", DATA_TYPE_RPM, rpm);
            Serial.println("]}");
        } else {
            send_response(RESP_INVALID_PARAM, "Invalid device_id");
        }
    } else {
        send_response(RESP_INVALID_PARAM, "Invalid device_type");
    }
}

// CMD 2: Set fan speed
void cmd_set_fan_speed(uint8_t fan_id, float speed) {
    if (fan_id < 2) {
        if (speed >= 0.0f && speed <= 1.0f) {
            set_fan_speed(fan_id, speed);
            send_response(RESP_OK, "Fan speed set");
        } else {
            send_response(RESP_INVALID_PARAM, "Speed must be 0.0-1.0");
        }
    } else {
        send_response(RESP_INVALID_PARAM, "Invalid fan_id");
    }
}

// CMD 3: Tare scale
void cmd_tare_scale() {
    tare_scale();
    send_response(RESP_OK, "Scale tared");
}

// CMD 4: Get all sensor data
void cmd_get_all_data() {
    float temps[8], hums[8];
    read_all_aht30(temps, hums);
    float weight = read_weight();
    
    Serial.println("{\"status\":0,\"data\":[");
    bool first = true;
    
    // AHT30 sensors
    for (int i = 0; i < 8; i++) {
        if (!isnan(temps[i])) {
            if (!first) Serial.println(",");
            Serial.printf("  {\"type\":%d,\"id\":%d,\"values\":[", DEVICE_TYPE_AHT30, i);
            Serial.printf("{\"data_type\":%d,\"value\":%.2f},", DATA_TYPE_TEMPERATURE, temps[i]);
            Serial.printf("{\"data_type\":%d,\"value\":%.2f}", DATA_TYPE_HUMIDITY, hums[i]);
            Serial.print("]}");
            first = false;
        }
    }
    
    // HX711
    if (!isnan(weight)) {
        if (!first) Serial.println(",");
        Serial.printf("  {\"type\":%d,\"id\":0,\"values\":[", DEVICE_TYPE_HX711);
        Serial.printf("{\"data_type\":%d,\"value\":%.2f}", DATA_TYPE_WEIGHT, weight);
        Serial.print("]}");
        first = false;
    }
    
    // Fans
    for (int i = 0; i < 2; i++) {
        if (!first) Serial.println(",");
        Serial.printf("  {\"type\":%d,\"id\":%d,\"values\":[", DEVICE_TYPE_FAN, i);
        Serial.printf("{\"data_type\":%d,\"value\":%.2f},", DATA_TYPE_SPEED, get_fan_speed(i));
        Serial.printf("{\"data_type\":%d,\"value\":%u}", DATA_TYPE_RPM, get_fan_rpm(i));
        Serial.print("]}");
        first = false;
    }
    
    Serial.println("\n]}");
}

// Process incoming requests
void process_request() {
    if (Serial.available()) {
        String request = Serial.readStringUntil('\n');
        request.trim();
        
        // Request format: CMD,PARAM1,PARAM2,...
        int firstComma = request.indexOf(',');
        
        if (firstComma < 0) {
            // Single command without parameters
            uint8_t cmd = request.toInt();
            
            switch (cmd) {
                case CMD_GET_SENSORS:
                    cmd_get_sensors();
                    break;
                case CMD_TARE_SCALE:
                    cmd_tare_scale();
                    break;
                case CMD_GET_ALL_DATA:
                    cmd_get_all_data();
                    break;
                default:
                    send_response(RESP_INVALID_CMD, "Unknown command");
                    break;
            }
        } else {
            // Command with parameters
            uint8_t cmd = request.substring(0, firstComma).toInt();
            String params = request.substring(firstComma + 1);
            
            int secondComma = params.indexOf(',');
            
            switch (cmd) {
                case CMD_READ_SENSOR: {
                    if (secondComma > 0) {
                        uint8_t device_type = params.substring(0, secondComma).toInt();
                        uint8_t device_id = params.substring(secondComma + 1).toInt();
                        cmd_read_sensor(device_type, device_id);
                    } else {
                        send_response(RESP_INVALID_PARAM, "Missing parameters");
                    }
                    break;
                }
                case CMD_SET_FAN_SPEED: {
                    if (secondComma > 0) {
                        uint8_t fan_id = params.substring(0, secondComma).toInt();
                        float speed = params.substring(secondComma + 1).toFloat();
                        cmd_set_fan_speed(fan_id, speed);
                    } else {
                        send_response(RESP_INVALID_PARAM, "Missing parameters");
                    }
                    break;
                }
                default:
                    send_response(RESP_INVALID_CMD, "Unknown command");
                    break;
            }
        }
    }
}

void setup() {
    // Initialize Serial
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("{\"status\":0,\"message\":\"Robot Sensor Hub v2.1 - Request-Response Protocol\"}");
    
    // Initialize sensors
    init_aht30_sensors();
    init_hx711();
    init_fan_controller();
    
    Serial.println("{\"status\":0,\"message\":\"Initialization complete, ready for requests\"}");
}

void loop() {
    // Process incoming requests
    process_request();
    
    delay(1);  // Minimal delay to prevent CPU hogging
}
