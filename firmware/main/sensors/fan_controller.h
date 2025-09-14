// main/sensors/fan_controller.h
#pragma once

#include <stdint.h>
#include <stdbool.h>

void init_fan_controller(void);
void set_fan_speed(int fan_id, float speed);
float get_fan_speed(int fan_id);
uint32_t get_fan_rpm(int fan_id);
bool is_fan_rotating(int fan_id);