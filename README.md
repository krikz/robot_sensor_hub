# robot_sensor_hub

Edge sensor node for robot health monitoring: temperature, humidity, weight, and fan control via micro-ROS.
Built on ESP32-C3 with AHT30, TCA9548A, HX711, and PWM fans.

## Features
- ✅ Multi-point temperature & humidity monitoring (AHT30)
- ✅ Load cell weight sensing (HX711)
- ✅ PWM fan speed control
- ✅ ROS2 integration via micro-ROS

## Hardware
- ESP32-C3
- TCA9548A I2C multiplexer
- AHT30 × N
- HX711 + Load cell
- 2× PWM fans
