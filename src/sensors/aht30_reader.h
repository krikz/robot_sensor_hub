#pragma once

#include <Arduino.h>

void init_aht30_sensors(void);
void read_all_aht30(float *temps, float *hums);
