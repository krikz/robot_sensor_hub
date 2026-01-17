#!/usr/bin/env python3
"""
Robot Sensor Hub - Python Reader Example

Reads sensor data from ESP32 via Serial port and displays it in a user-friendly format.

Usage:
    python3 read_sensors.py [port] [baudrate]
    
Examples:
    python3 read_sensors.py /dev/ttyUSB0 115200
    python3 read_sensors.py COM3 115200
"""

import serial
import json
import time
import sys
from datetime import datetime

# Device type constants
DEVICE_TYPE_AHT30 = 0
DEVICE_TYPE_HX711 = 1
DEVICE_TYPE_FAN = 2

# Data type constants
DATA_TYPE_TEMPERATURE = 1
DATA_TYPE_HUMIDITY = 2
DATA_TYPE_WEIGHT = 3
DATA_TYPE_SPEED = 4
DATA_TYPE_RPM = 5

def get_device_type_name(device_type):
    """Convert device type to human-readable name"""
    types = {
        DEVICE_TYPE_AHT30: "AHT30",
        DEVICE_TYPE_HX711: "HX711",
        DEVICE_TYPE_FAN: "FAN"
    }
    return types.get(device_type, f"Unknown({device_type})")

def get_data_type_name(data_type):
    """Convert data type to human-readable name"""
    types = {
        DATA_TYPE_TEMPERATURE: "Temperature",
        DATA_TYPE_HUMIDITY: "Humidity",
        DATA_TYPE_WEIGHT: "Weight",
        DATA_TYPE_SPEED: "Speed",
        DATA_TYPE_RPM: "RPM"
    }
    return types.get(data_type, f"Unknown({data_type})")

def get_unit(data_type):
    """Get unit for data type"""
    units = {
        DATA_TYPE_TEMPERATURE: "°C",
        DATA_TYPE_HUMIDITY: "%",
        DATA_TYPE_WEIGHT: "g",
        DATA_TYPE_SPEED: "%",
        DATA_TYPE_RPM: "RPM"
    }
    return units.get(data_type, "")

def format_device_data(device):
    """Format device data for display"""
    dev_type = get_device_type_name(device['type'])
    dev_id = device['id']
    data_type = get_data_type_name(device['data_type'])
    value = device['value']
    unit = get_unit(device['data_type'])
    
    return f"  {dev_type}[{dev_id}] - {data_type}: {value:.2f} {unit}"

def main():
    # Parse command line arguments
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    
    print(f"Robot Sensor Hub - Python Reader")
    print(f"=" * 50)
    print(f"Port: {port}")
    print(f"Baudrate: {baudrate}")
    print(f"=" * 50)
    print()
    
    try:
        # Open serial connection
        print(f"Connecting to {port}...")
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Wait for connection to stabilize
        print("Connected!")
        print()
        
        buffer = ""
        
        while True:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                buffer += data
                
                # Look for complete JSON object
                if '{' in buffer and '}' in buffer:
                    start = buffer.index('{')
                    end = buffer.index('}', start) + 1
                    json_str = buffer[start:end]
                    buffer = buffer[end:]
                    
                    try:
                        data = json.loads(json_str)
                        
                        # Display snapshot
                        timestamp = datetime.now().strftime('%H:%M:%S')
                        print(f"\n[{timestamp}] Sensor Snapshot:")
                        print("-" * 50)
                        
                        for device in data['devices']:
                            print(format_device_data(device))
                        
                        print("-" * 50)
                        
                    except json.JSONDecodeError as e:
                        pass  # Ignore incomplete JSON
            
            time.sleep(0.01)  # Small delay to prevent CPU hogging
            
    except serial.SerialException as e:
        print(f"Error: {e}")
        print("\nTips:")
        print("  - Check if the port name is correct")
        print("  - Make sure ESP32 is connected")
        print("  - Check if you have permissions (Linux: sudo usermod -a -G dialout $USER)")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\nExiting...")
        ser.close()
        sys.exit(0)

if __name__ == '__main__':
    main()
