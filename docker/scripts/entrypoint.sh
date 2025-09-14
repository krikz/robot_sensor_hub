#!/bin/bash
# entrypoint.sh

# Активация среды ESP-IDF
. /opt/esp/idf/export.sh

echo "=== MicroROS ESP32 Builder ==="
echo "Project directory: /project"

# Проверяем и копируем кастомные сообщения в extra_packages
if [ -d "/project/robot_sensor_hub_msg" ]; then
    echo "Copying custom messages to MicroROS component..."
    mkdir -p /opt/micro_ros_espidf/extra_packages/robot_sensor_hub_msg
    cp -r /project/robot_sensor_hub_msg/* /opt/micro_ros_espidf/extra_packages/robot_sensor_hub_msg/
    
    # Проверяем, есть ли файлы сообщений
    if [ -f "/opt/micro_ros_espidf/extra_packages/robot_sensor_hub_msg/msg/DeviceCommand.msg" ]; then
        echo "✓ Custom messages copied successfully:"
        echo "  - DeviceCommand.msg"
        echo "  - DeviceData.msg"
    else
        echo "⚠ Warning: Custom messages not found in expected location"
    fi
else
    echo "⚠ Warning: robot_sensor_hub_msg directory not found"
fi

# Проверяем наличие firmware директории
if [ ! -d "/project/firmware" ]; then
    echo "❌ Error: firmware directory not found!"
    echo "Please mount your project volume correctly"
    exit 1
fi

echo "Environment setup complete!"
echo "Available commands:"
echo "  idf.py build      - Build the project"
echo "  idf.py flash      - Flash to ESP32 (specify port with -p)"
echo "  idf.py monitor    - Monitor serial output"
echo ""

# Переходим в директорию проекта
cd /project/firmware

# Запускаем переданную команду или bash по умолчанию
exec "$@"