# Firmware Update Guide

This guide explains how to update the firmware on your ESP32 Robot Sensor Hub using serial port.

## Method: Serial Bootloader (Recommended)

This method uses ESP32's built-in ROM bootloader, which is always available and cannot be overwritten. This is the safest method for firmware updates.

## Prerequisites

1. **Python 3** installed
2. **esptool.py** - ESP32 flashing tool
   ```bash
   pip install esptool
   ```
3. **USB cable** connected to ESP32
4. **Firmware binary file** (`.bin`)

## Quick Start

### 1. Build Firmware

First, build the firmware for your target:

```bash
# Build default configuration
pio run -e default

# Or build full configuration
pio run -e full
```

The firmware binary will be created at:
- `.pio/build/default/firmware.bin`
- `.pio/build/full/firmware.bin`

### 2. Update Firmware

Use the provided update script:

```bash
python3 firmware_update.py /dev/ttyUSB0 .pio/build/default/firmware.bin
```

**Windows:**
```bash
python3 firmware_update.py COM3 .pio/build/default/firmware.bin
```

The script will:
1. Check esptool.py is installed
2. Verify firmware file exists
3. Show current firmware version (if readable)
4. Ask for confirmation
5. Optionally erase flash
6. Flash new firmware
7. Reset ESP32

## Manual Method (Advanced)

If you prefer to use esptool.py directly:

### Check Current Connection

```bash
esptool.py --port /dev/ttyUSB0 chip_id
```

### Erase Flash (Optional, Recommended for Clean Install)

```bash
esptool.py --port /dev/ttyUSB0 erase_flash
```

### Flash Firmware

```bash
esptool.py --port /dev/ttyUSB0 \
  --baud 921600 \
  --before default_reset \
  --after hard_reset \
  write_flash \
  --flash_mode dio \
  --flash_freq 40m \
  --flash_size detect \
  0x10000 .pio/build/default/firmware.bin
```

### Verify Version

After flashing, connect and check version:

```bash
python3 sensor_client.py /dev/ttyUSB0
```

The client will automatically display the firmware version.

## Understanding Flash Addresses

- `0x1000` - Bootloader (DO NOT OVERWRITE)
- `0x8000` - Partition table
- `0x10000` - **Application firmware** (what we update)
- `0xe000` - Boot app

We only write to `0x10000` to update the application firmware.

## Troubleshooting

### esptool.py Not Found

Install it:
```bash
pip install esptool
# or
pip3 install esptool
```

### Permission Denied (Linux/Mac)

Add your user to the dialout group:
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

Or use sudo:
```bash
sudo python3 firmware_update.py /dev/ttyUSB0 firmware.bin
```

### Cannot Connect to ESP32

1. **Check USB cable** - Use a data cable, not just charging cable
2. **Close serial monitor** - Can't flash while monitor is open
3. **Check port name**:
   - Linux: `ls /dev/ttyUSB*` or `ls /dev/ttyACM*`
   - Mac: `ls /dev/cu.*`
   - Windows: Check Device Manager
4. **Try lower baud rate**: Use `--baud 115200` instead of 921600
5. **Enter bootloader manually**:
   - Hold BOOT button
   - Press RESET button
   - Release RESET
   - Release BOOT
   - Try flashing again

### Flash Failed / Verification Failed

1. **Erase flash first**: Run with erase option
2. **Check firmware file**: Make sure it's the correct `.bin` file
3. **Try different USB port**
4. **Check power supply**: USB hub may not provide enough power

### Device Not Responding After Update

1. **Wait 5 seconds** - ESP32 needs time to boot
2. **Press RESET button** on ESP32
3. **Power cycle** - Disconnect and reconnect USB
4. **Reflash firmware** - Try flashing again

## Firmware Update Workflow

### Development Cycle

```bash
# 1. Make code changes
nano src/main.cpp

# 2. Build
pio run -e default

# 3. Flash
python3 firmware_update.py /dev/ttyUSB0 .pio/build/default/firmware.bin

# 4. Test
python3 sensor_client.py /dev/ttyUSB0
```

### Production Deployment

1. **Build release firmware**:
   ```bash
   pio run -e default
   ```

2. **Test on development board**:
   ```bash
   python3 firmware_update.py /dev/ttyUSB0 .pio/build/default/firmware.bin
   ```

3. **Verify version and functionality**:
   ```bash
   python3 sensor_client.py /dev/ttyUSB0
   # Send command: 5
   # Verify version is correct
   ```

4. **Copy firmware to Raspberry Pi**:
   ```bash
   scp .pio/build/default/firmware.bin pi@raspberrypi.local:~/
   ```

5. **Flash on production device** (from Raspberry Pi):
   ```bash
   python3 firmware_update.py /dev/ttyUSB0 firmware.bin
   ```

## Backup and Recovery

### Backup Current Firmware

```bash
esptool.py --port /dev/ttyUSB0 \
  read_flash 0x10000 0x200000 firmware_backup.bin
```

### Restore Backup

```bash
esptool.py --port /dev/ttyUSB0 \
  write_flash 0x10000 firmware_backup.bin
```

## Version Management

Always check version before and after update:

**Before update:**
```bash
python3 sensor_client.py /dev/ttyUSB0
# Command: 6 (get version)
```

**After update:**
```bash
# Wait 5 seconds for ESP32 to boot
python3 sensor_client.py /dev/ttyUSB0
# Version displayed automatically
```

## Safety Notes

1. ✅ **Safe**: This method uses ROM bootloader, cannot brick device
2. ✅ **Recoverable**: Can always reflash using esptool.py
3. ⚠️ **Don't disconnect**: during flashing process
4. ⚠️ **Close monitors**: before flashing
5. 💡 **Use erase**: for clean installs or after major changes

## Automated Updates (Optional)

For automated deployment, create a script:

```bash
#!/bin/bash
# auto_update.sh

PORT=/dev/ttyUSB0
FIRMWARE=.pio/build/default/firmware.bin

echo "Building firmware..."
pio run -e default

echo "Flashing firmware..."
esptool.py --port $PORT --baud 921600 \
  write_flash 0x10000 $FIRMWARE

echo "Done! Rebooting..."
sleep 3

echo "Checking version..."
python3 sensor_client.py $PORT
```

Make executable and run:
```bash
chmod +x auto_update.sh
./auto_update.sh
```

## FAQ

**Q: Can I update via WiFi instead?**  
A: Not currently implemented. Serial update is safer and works everywhere.

**Q: Will I lose my target configuration?**  
A: No, target configuration is compiled into the firmware.

**Q: Can I update multiple ESP32 boards?**  
A: Yes, one at a time. Or write a script to loop through ports.

**Q: What if power is lost during update?**  
A: ESP32 won't boot, but you can reflash. ROM bootloader always works.

**Q: How long does update take?**  
A: Usually 30-60 seconds at 921600 baud, 1-2 minutes at 115200 baud.

**Q: Can I update via Raspberry Pi?**  
A: Yes! Install esptool on RPi and use the same commands.

## Further Reading

- [esptool.py documentation](https://docs.espressif.com/projects/esptool/en/latest/)
- [ESP32 Flash Download Tool](https://www.espressif.com/en/support/download/other-tools) (GUI, Windows only)
- PlatformIO upload options: `pio run -t upload --help`
