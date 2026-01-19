#!/usr/bin/env python3
"""
Firmware Updater for Robot Sensor Hub via Serial

This script uses esptool.py to flash new firmware to ESP32 via serial port.
It uses the ESP32's built-in ROM bootloader, which is always available.

Usage:
    python3 firmware_update.py <port> <firmware.bin>
    
Examples:
    python3 firmware_update.py /dev/ttyUSB0 firmware.bin
    python3 firmware_update.py COM3 firmware.bin
"""

import sys
import os
import subprocess
import time

def check_esptool():
    """Check if esptool.py is installed"""
    try:
        result = subprocess.run(['esptool.py', 'version'], 
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print(f"✓ esptool.py found: {result.stdout.strip()}")
            return True
    except FileNotFoundError:
        pass
    except subprocess.TimeoutExpired:
        pass
    
    print("✗ esptool.py not found!")
    print("\nInstall with: pip install esptool")
    return False

def check_firmware_file(firmware_path):
    """Check if firmware file exists"""
    if not os.path.exists(firmware_path):
        print(f"✗ Firmware file not found: {firmware_path}")
        return False
    
    file_size = os.path.getsize(firmware_path)
    print(f"✓ Firmware file found: {firmware_path} ({file_size} bytes)")
    return True

def flash_firmware(port, firmware_path, baudrate=921600):
    """Flash firmware to ESP32"""
    print("\n" + "="*60)
    print("FLASHING FIRMWARE TO ESP32")
    print("="*60)
    print(f"Port: {port}")
    print(f"Firmware: {firmware_path}")
    print(f"Baudrate: {baudrate}")
    print("\nThis will:")
    print("1. Erase flash (optional, can be skipped)")
    print("2. Write new firmware")
    print("3. Reset ESP32")
    print("\n⚠️  Do NOT disconnect during flashing!")
    print("="*60 + "\n")
    
    # Ask for confirmation
    response = input("Continue? [y/N]: ").strip().lower()
    if response not in ['y', 'yes']:
        print("Cancelled by user")
        return False
    
    print("\n--- Step 1: Erasing flash (optional) ---")
    erase_response = input("Erase flash before flashing? [y/N]: ").strip().lower()
    
    if erase_response in ['y', 'yes']:
        print("Erasing flash...")
        cmd = [
            'esptool.py',
            '--port', port,
            '--baud', str(baudrate),
            'erase_flash'
        ]
        
        try:
            result = subprocess.run(cmd, timeout=60)
            if result.returncode != 0:
                print("✗ Flash erase failed!")
                return False
            print("✓ Flash erased successfully\n")
        except subprocess.TimeoutExpired:
            print("✗ Flash erase timeout!")
            return False
        except KeyboardInterrupt:
            print("\n✗ Cancelled by user")
            return False
    else:
        print("Skipping flash erase\n")
    
    print("--- Step 2: Writing firmware ---")
    print("Flashing... (this may take 1-2 minutes)\n")
    
    # Flash command
    # 0x10000 is the standard app partition offset for ESP32
    cmd = [
        'esptool.py',
        '--port', port,
        '--baud', str(baudrate),
        '--before', 'default_reset',
        '--after', 'hard_reset',
        'write_flash',
        '--flash_mode', 'dio',
        '--flash_freq', '40m',
        '--flash_size', 'detect',
        '0x10000', firmware_path
    ]
    
    try:
        result = subprocess.run(cmd, timeout=180)
        if result.returncode != 0:
            print("\n✗ Firmware flash failed!")
            return False
        
        print("\n" + "="*60)
        print("✓ FIRMWARE UPDATED SUCCESSFULLY!")
        print("="*60)
        print("\nESP32 is rebooting...")
        print("You can now connect and verify the version.")
        return True
        
    except subprocess.TimeoutExpired:
        print("\n✗ Firmware flash timeout!")
        return False
    except KeyboardInterrupt:
        print("\n✗ Cancelled by user")
        return False

def get_firmware_info(port, baudrate=115200):
    """Try to get current firmware version via serial"""
    try:
        import serial
        import json
        
        print(f"\n--- Checking current firmware version ---")
        ser = serial.Serial(port, baudrate, timeout=2)
        time.sleep(2)
        
        # Clear buffer
        ser.reset_input_buffer()
        
        # Send version command
        ser.write(b"5\n")
        time.sleep(0.5)
        
        # Read response
        if ser.in_waiting:
            response = ser.readline().decode('utf-8', errors='ignore').strip()
            try:
                data = json.loads(response)
                if data.get('status') == 0 and 'version' in data:
                    version = data['version']
                    print(f"Current firmware: v{version.get('firmware', 'unknown')}")
                    print(f"Target: {version.get('target', 'unknown')}")
                    print(f"Built: {version.get('build_date', '')} {version.get('build_time', '')}")
                    ser.close()
                    return True
            except json.JSONDecodeError:
                pass
        
        ser.close()
        print("Could not read version (device may not be running sensor hub firmware)")
        return False
        
    except ImportError:
        print("pyserial not installed, skipping version check")
        return False
    except Exception as e:
        print(f"Could not read version: {e}")
        return False

def main():
    print("="*60)
    print("Robot Sensor Hub - Firmware Updater")
    print("="*60)
    
    if len(sys.argv) < 3:
        print("\nUsage: python3 firmware_update.py <port> <firmware.bin>")
        print("\nExamples:")
        print("  python3 firmware_update.py /dev/ttyUSB0 firmware.bin")
        print("  python3 firmware_update.py COM3 firmware.bin")
        print("\nTo build firmware first:")
        print("  pio run -e default")
        print("  Firmware will be at: .pio/build/default/firmware.bin")
        sys.exit(1)
    
    port = sys.argv[1]
    firmware_path = sys.argv[2]
    
    print()
    
    # Check prerequisites
    if not check_esptool():
        sys.exit(1)
    
    if not check_firmware_file(firmware_path):
        sys.exit(1)
    
    # Try to get current version
    get_firmware_info(port)
    
    # Flash firmware
    if flash_firmware(port, firmware_path):
        print("\n💡 Tip: Wait 3-5 seconds, then connect with:")
        print(f"   python3 sensor_client.py {port}")
        sys.exit(0)
    else:
        print("\n✗ Firmware update failed!")
        print("\n🔧 Troubleshooting:")
        print("  1. Check USB cable connection")
        print("  2. Check port name is correct")
        print("  3. Close any programs using the serial port")
        print("  4. Try holding BOOT button while connecting")
        sys.exit(1)

if __name__ == '__main__':
    main()
