# Example: Creating a Custom Target

This example shows how to create a custom hardware configuration.

## Scenario

You have:
- 4 AHT30 sensors (on channels 0, 1, 4, 5)
- No HX711 weight sensor
- 1 fan instead of 2
- Custom GPIO pins

## Steps

### 1. Create target file

Create `src/targets/custom_4sensors.h`:

```c
#pragma once

// ========================================
// I2C Configuration
// ========================================
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_FREQ 100000

// ========================================
// TCA9548A I2C Multiplexer
// ========================================
#define USE_TCA9548A 1
#define TCA9548A_ADDRESS 0x70

// Only 4 AHT30 sensors on specific channels
#define AHT30_CHANNEL_0 1
#define AHT30_CHANNEL_1 1
#define AHT30_CHANNEL_2 0
#define AHT30_CHANNEL_3 0
#define AHT30_CHANNEL_4 1
#define AHT30_CHANNEL_5 1
#define AHT30_CHANNEL_6 0
#define AHT30_CHANNEL_7 0

// ========================================
// HX711 Weight Sensor - DISABLED
// ========================================
#define USE_HX711 0
#define HX711_DOUT_PIN 18  // Not used but must be defined
#define HX711_SCK_PIN 19

// ========================================
// Fan Controller - Only 1 fan
// ========================================
#define NUM_FANS 1

// Fan 0 on custom pins
#define FAN0_PWM_PIN 25
#define FAN0_TACHO_PIN 26

// Fan 1 not used but must be defined
#define FAN1_PWM_PIN 14
#define FAN1_TACHO_PIN 16

// PWM Configuration
#define FAN_PWM_FREQ 25000
#define FAN_PWM_RESOLUTION 8

// ========================================
// Serial Configuration
// ========================================
#define SERIAL_BAUD 115200

// ========================================
// Target Name
// ========================================
#define TARGET_NAME "CUSTOM_4SENSORS_1FAN"
```

### 2. Add to platformio.ini

Add this section to `platformio.ini`:

```ini
; ==============================================
; Target: Custom 4-sensor configuration
; ==============================================
[env:custom_4sensors]
platform = ${common.platform}
board = ${common.board}
framework = ${common.framework}
monitor_speed = ${common.monitor_speed}
monitor_filters = ${common.monitor_filters}
lib_deps = ${common.lib_deps}
upload_speed = ${common.upload_speed}
build_flags = 
    -DUSE_TARGET_custom_4sensors
    -DCORE_DEBUG_LEVEL=3
; upload_port = /dev/ttyUSB0
```

### 3. Build and upload

```bash
pio run -e custom_4sensors -t upload
```

### 4. Verify

When ESP32 boots, you'll see:
```json
{"status":0,"message":"Robot Sensor Hub v2.1 - Target: CUSTOM_4SENSORS_1FAN"}
[AHT30] Initializing with target: CUSTOM_4SENSORS_1FAN
[AHT30] Channel 2 disabled in target config
[AHT30] Channel 3 disabled in target config
...
[HX711] Disabled in target config
[FAN] Initializing 1 fan(s) with target: CUSTOM_4SENSORS_1FAN
[FAN] Fan 0: PWM pin=25, TACHO pin=26
```

## Common Customizations

### Change I2C pins
```c
#define I2C_SDA_PIN 19
#define I2C_SCL_PIN 23
```

### Use different TCA9548A address
```c
#define TCA9548A_ADDRESS 0x71
```

### Disable all temperature sensors
```c
#define USE_TCA9548A 0
#define AHT30_CHANNEL_0 0
// ... all channels 0
```

### Use higher PWM frequency for fans
```c
#define FAN_PWM_FREQ 40000  // 40kHz
```

### Different serial speed
```c
#define SERIAL_BAUD 921600  // Faster
```

## Tips

1. **Start with `default.h`**: Copy it and modify incrementally
2. **Test each change**: Build and upload after each modification
3. **Keep target name unique**: Makes debugging easier
4. **Document your config**: Add comments explaining your hardware
5. **Share configs**: If you have a common board, share your target file

## Multiple Hardware Versions

You can maintain multiple configs for different hardware revisions:

```
src/targets/
├── default.h           # Standard config
├── full_config.h       # All sensors
├── v1_hardware.h       # Your first hardware revision
├── v2_hardware.h       # Second revision with changes
└── production.h        # Final production config
```

Then build with: `pio run -e v2_hardware -t upload`
