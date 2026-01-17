#!/usr/bin/env python3
"""
Robot Sensor Hub - Command Sender

Send commands to ESP32 via Serial port.

Usage:
    python3 send_command.py [port] [command]
    
Commands format: TYPE,ID,CMD,PARAM

Examples:
    # Set fan 0 speed to 75%
    python3 send_command.py /dev/ttyUSB0 "2,0,0,0.75"
    
    # Set fan 1 speed to 50%
    python3 send_command.py /dev/ttyUSB0 "2,1,0,0.50"
    
    # Tare scale
    python3 send_command.py /dev/ttyUSB0 "1,0,1,0"
"""

import serial
import time
import sys

def send_command(port, baudrate, command):
    """Send command to ESP32"""
    try:
        print(f"Connecting to {port}...")
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Wait for connection
        
        print(f"Sending command: {command}")
        ser.write(f"{command}\n".encode('utf-8'))
        
        # Wait for response
        time.sleep(0.5)
        if ser.in_waiting:
            response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            print("Response:")
            print(response)
        
        ser.close()
        print("Command sent successfully!")
        
    except serial.SerialException as e:
        print(f"Error: {e}")
        sys.exit(1)

def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    
    port = sys.argv[1]
    command = sys.argv[2] if len(sys.argv) > 2 else None
    baudrate = 115200
    
    if not command:
        print("Error: Command required")
        print(__doc__)
        sys.exit(1)
    
    send_command(port, baudrate, command)

if __name__ == '__main__':
    main()
