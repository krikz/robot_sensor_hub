#!/usr/bin/env python3
"""
Robot Sensor Hub - Request-Response Client for Raspberry Pi

This script demonstrates the request-response protocol for communicating
with the ESP32 sensor hub. It can be used as a standalone tool or as a
reference for building a ROS node.

Protocol Commands:
    0 - Get list of available sensors
    1,TYPE,ID - Read specific sensor data
    2,FAN_ID,SPEED - Set fan speed (0.0-1.0)
    3 - Tare scale
    4 - Get all sensor data
    5 - Get firmware version information

Usage:
    python3 sensor_client.py [port] [baudrate]
    
Example:
    python3 sensor_client.py /dev/ttyUSB0 115200
"""

import serial
import json
import time
import sys
from typing import Dict, List, Optional

# Command codes
CMD_GET_SENSORS = 0
CMD_READ_SENSOR = 1
CMD_SET_FAN_SPEED = 2
CMD_TARE_SCALE = 3
CMD_GET_ALL_DATA = 4
CMD_GET_VERSION = 5

# Device types
DEVICE_TYPE_AHT30 = 0
DEVICE_TYPE_HX711 = 1
DEVICE_TYPE_FAN = 2

# Data types
DATA_TYPE_TEMPERATURE = 1
DATA_TYPE_HUMIDITY = 2
DATA_TYPE_WEIGHT = 3
DATA_TYPE_SPEED = 4
DATA_TYPE_RPM = 5


class SensorHubClient:
    """Client for communicating with Robot Sensor Hub"""
    
    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 115200):
        """Initialize serial connection"""
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Wait for connection to stabilize
        # Read any startup messages
        while self.ser.in_waiting:
            self.ser.readline()
    
    def _send_command(self, command: str) -> Optional[Dict]:
        """Send command and wait for JSON response"""
        self.ser.write(f"{command}\n".encode('utf-8'))
        time.sleep(0.1)  # Small delay for processing
        
        if self.ser.in_waiting:
            response = self.ser.readline().decode('utf-8', errors='ignore').strip()
            try:
                return json.loads(response)
            except json.JSONDecodeError:
                print(f"Failed to parse response: {response}")
                return None
        return None
    
    def get_sensors(self) -> Optional[List[Dict]]:
        """Get list of available sensors"""
        response = self._send_command(str(CMD_GET_SENSORS))
        if response and response.get('status') == 0:
            return response.get('sensors', [])
        return None
    
    def read_sensor(self, device_type: int, device_id: int) -> Optional[Dict]:
        """Read specific sensor data"""
        command = f"{CMD_READ_SENSOR},{device_type},{device_id}"
        response = self._send_command(command)
        if response and response.get('status') == 0:
            return response
        return None
    
    def set_fan_speed(self, fan_id: int, speed: float) -> bool:
        """Set fan speed (0.0-1.0)"""
        command = f"{CMD_SET_FAN_SPEED},{fan_id},{speed:.2f}"
        response = self._send_command(command)
        return response and response.get('status') == 0
    
    def tare_scale(self) -> bool:
        """Tare the weight scale"""
        response = self._send_command(str(CMD_TARE_SCALE))
        return response and response.get('status') == 0
    
    def get_all_data(self) -> Optional[List[Dict]]:
        """Get all sensor data at once"""
        response = self._send_command(str(CMD_GET_ALL_DATA))
        if response and response.get('status') == 0:
            return response.get('data', [])
        return None
    
    def get_version(self) -> Optional[Dict]:
        """Get firmware version information"""
        response = self._send_command(str(CMD_GET_VERSION))
        if response and response.get('status') == 0:
            return response.get('version', {})
        return None
    
    def close(self):
        """Close serial connection"""
        self.ser.close()


def print_sensors(sensors: List[Dict]):
    """Pretty print sensor list"""
    print("\n=== Available Sensors ===")
    for sensor in sensors:
        print(f"  {sensor['name']} (Type: {sensor['type']}, ID: {sensor['id']}) - Available: {sensor['available']}")
    print()


def print_sensor_data(data: Dict):
    """Pretty print sensor data"""
    device_names = {0: "AHT30", 1: "HX711", 2: "FAN"}
    data_names = {
        1: ("Temperature", "°C"),
        2: ("Humidity", "%"),
        3: ("Weight", "g"),
        4: ("Speed", "%"),
        5: ("RPM", "")
    }
    
    device_name = device_names.get(data.get('type', -1), "Unknown")
    print(f"\n{device_name} [ID: {data.get('id')}]:")
    
    for item in data.get('data', []):
        data_type = item.get('data_type')
        value = item.get('value')
        name, unit = data_names.get(data_type, ("Unknown", ""))
        print(f"  {name}: {value:.2f} {unit}")


def print_all_data(all_data: List[Dict]):
    """Pretty print all sensor data"""
    print("\n=== All Sensor Data ===")
    for data in all_data:
        device_names = {0: "AHT30", 1: "HX711", 2: "FAN"}
        data_names = {
            1: ("Temperature", "°C"),
            2: ("Humidity", "%"),
            3: ("Weight", "g"),
            4: ("Speed", "%"),
            5: ("RPM", "")
        }
        
        device_name = device_names.get(data.get('type', -1), "Unknown")
        print(f"\n{device_name} [ID: {data.get('id')}]:")
        
        for item in data.get('values', []):
            data_type = item.get('data_type')
            value = item.get('value')
            name, unit = data_names.get(data_type, ("Unknown", ""))
            print(f"  {name}: {value:.2f} {unit}")
    print()


def print_version(version: Dict):
    """Pretty print version information"""
    print("\n=== Firmware Version ===")
    print(f"  Project: {version.get('project', 'Unknown')}")
    print(f"  Version: {version.get('firmware', 'Unknown')}")
    print(f"  Build Date: {version.get('build_date', 'Unknown')} {version.get('build_time', 'Unknown')}")
    print(f"  Protocol: {version.get('protocol', 'Unknown')}")
    print(f"  Target: {version.get('target', 'Unknown')}")
    print()


def interactive_mode(client: SensorHubClient):
    """Interactive command-line interface"""
    print("\n=== Robot Sensor Hub Client ===")
    print("Commands:")
    print("  1 - Get available sensors")
    print("  2 - Read sensor data")
    print("  3 - Set fan speed")
    print("  4 - Tare scale")
    print("  5 - Get all data")
    print("  6 - Get firmware version")
    print("  q - Quit")
    print()
    
    while True:
        try:
            cmd = input("Enter command: ").strip().lower()
            
            if cmd == 'q':
                break
            elif cmd == '1':
                sensors = client.get_sensors()
                if sensors:
                    print_sensors(sensors)
                else:
                    print("Failed to get sensors")
            elif cmd == '2':
                device_type = int(input("Device type (0=AHT30, 1=HX711, 2=FAN): "))
                device_id = int(input("Device ID: "))
                data = client.read_sensor(device_type, device_id)
                if data:
                    print_sensor_data(data)
                else:
                    print("Failed to read sensor")
            elif cmd == '3':
                fan_id = int(input("Fan ID (0 or 1): "))
                speed = float(input("Speed (0.0-1.0): "))
                if client.set_fan_speed(fan_id, speed):
                    print(f"Fan {fan_id} speed set to {speed:.2f}")
                else:
                    print("Failed to set fan speed")
            elif cmd == '4':
                if client.tare_scale():
                    print("Scale tared successfully")
                else:
                    print("Failed to tare scale")
            elif cmd == '5':
                all_data = client.get_all_data()
                if all_data:
                    print_all_data(all_data)
                else:
                    print("Failed to get all data")
            elif cmd == '6':
                version = client.get_version()
                if version:
                    print_version(version)
                else:
                    print("Failed to get version")
            else:
                print("Invalid command")
        except ValueError as e:
            print(f"Invalid input: {e}")
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {e}")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    
    print(f"Connecting to {port} at {baudrate} baud...")
    
    try:
        client = SensorHubClient(port, baudrate)
        print("Connected!")
        
        # Get and display version
        print("\n--- Firmware Version ---")
        version = client.get_version()
        if version:
            print_version(version)
        
        # Example usage
        print("\n--- Example: Getting available sensors ---")
        sensors = client.get_sensors()
        if sensors:
            print_sensors(sensors)
        
        # Start interactive mode
        interactive_mode(client)
        
        client.close()
        print("\nDisconnected.")
        
    except serial.SerialException as e:
        print(f"Error: {e}")
        print("\nTips:")
        print("  - Check if the port name is correct")
        print("  - Make sure ESP32 is connected")
        print("  - Check permissions (Linux: sudo usermod -a -G dialout $USER)")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\nExiting...")
        sys.exit(0)


if __name__ == '__main__':
    main()
