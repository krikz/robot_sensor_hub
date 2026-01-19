# Hardware Target Configurations

This directory contains hardware target configurations for different sensor hub setups.

## What is a Target?

A target defines the hardware configuration for your sensor hub, including:
- GPIO pin assignments for sensors and fans
- Which I2C channels have AHT30 sensors connected
- Enable/disable specific hardware modules
- Serial communication settings

## Available Targets

### `default.h` (Default Configuration)
- 2 AHT30 sensors on channels 0-1
- HX711 weight sensor enabled
- 2 fans with PWM control
- Standard GPIO pins

### `full_config.h` (Full Configuration)
- 8 AHT30 sensors on all channels 0-7
- HX711 weight sensor enabled
- 2 fans with PWM control
- Standard GPIO pins

## Creating Your Own Target

1. Copy `default.h` to a new file (e.g., `my_custom.h`)
2. Modify the configuration values:
   ```c
   // Enable/disable specific AHT30 channels
   #define AHT30_CHANNEL_0 1  // 1 = enabled, 0 = disabled
   #define AHT30_CHANNEL_1 1
   #define AHT30_CHANNEL_2 0
   // ... etc
   
   // Change GPIO pins if needed
   #define FAN0_PWM_PIN 13
   #define FAN0_TACHO_PIN 15
   
   // Enable/disable modules
   #define USE_HX711 1  // 1 = enabled, 0 = disabled
   ```

3. Update `platformio.ini` to use your target:
   ```ini
   [env:my_custom]
   platform = espressif32
   board = esp32dev
   framework = arduino
   build_flags = 
       -DUSE_TARGET_my_custom
       -DCORE_DEBUG_LEVEL=3
   ```

4. Build and upload:
   ```bash
   pio run -e my_custom -t upload
   ```

## Configuration Options

### I2C Settings
- `I2C_SDA_PIN` - GPIO pin for I2C data line
- `I2C_SCL_PIN` - GPIO pin for I2C clock line
- `I2C_FREQ` - I2C bus frequency in Hz

### TCA9548A Multiplexer
- `USE_TCA9548A` - Enable/disable multiplexer (1/0)
- `TCA9548A_ADDRESS` - I2C address (default: 0x70)
- `AHT30_CHANNEL_X` - Enable AHT30 on channel X (0-7)

### HX711 Weight Sensor
- `USE_HX711` - Enable/disable sensor (1/0)
- `HX711_DOUT_PIN` - Data output GPIO pin
- `HX711_SCK_PIN` - Clock GPIO pin

### Fan Controller
- `NUM_FANS` - Number of fans (1-2)
- `FANX_PWM_PIN` - PWM control GPIO pin
- `FANX_TACHO_PIN` - Tachometer input GPIO pin
- `FAN_PWM_FREQ` - PWM frequency in Hz
- `FAN_PWM_RESOLUTION` - PWM resolution in bits

### Serial Communication
- `SERIAL_BAUD` - Baud rate for serial communication

## Examples

### Minimal Setup (1 sensor + 1 fan)
```c
#define AHT30_CHANNEL_0 1
#define AHT30_CHANNEL_1 0
// ... all other channels 0

#define USE_HX711 0  // No weight sensor

#define NUM_FANS 1  // Only 1 fan
```

### Custom GPIO Pins
```c
// Move fans to different pins
#define FAN0_PWM_PIN 25
#define FAN0_TACHO_PIN 26

#define FAN1_PWM_PIN 32
#define FAN1_TACHO_PIN 33

// Move HX711 to different pins
#define HX711_DOUT_PIN 4
#define HX711_SCK_PIN 5
```

## Target Selection in PlatformIO

Add your target as a build environment in `platformio.ini`:

```ini
[env:default]
platform = espressif32
board = esp32dev
framework = arduino
build_flags = 
    -DUSE_TARGET_default
    -DCORE_DEBUG_LEVEL=3

[env:full]
platform = espressif32
board = esp32dev
framework = arduino
build_flags = 
    -DUSE_TARGET_full_config
    -DCORE_DEBUG_LEVEL=3

[env:my_custom]
platform = espressif32
board = esp32dev
framework = arduino
build_flags = 
    -DUSE_TARGET_my_custom
    -DCORE_DEBUG_LEVEL=3
```

Then build with: `pio run -e my_custom`
