#pragma once
void init_hx711(void);
float read_weight(void);
void set_calibration_factor(float factor);
void tare_scale(void);