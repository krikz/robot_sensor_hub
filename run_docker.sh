#!/bin/bash
# run_docker.sh - Универсальный скрипт запуска для Linux/Windows

echo "Starting MicroROS ESP32 Builder..."

# Проверяем, запущен ли Docker
if ! docker info > /dev/null 2>&1; then
    echo "❌ Docker is not running. Please start Docker Desktop first."
    exit 1
fi

# Определяем ОС и настраиваем параметры
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    echo "🐧 Linux detected"
    # Для Linux: пробрасываем все доступные USB устройства
    USB_DEVICES=""
    for dev in /dev/ttyUSB* /dev/ttyACM*; do
        if [ -e "$dev" ]; then
            USB_DEVICES="$USB_DEVICES --device=$dev:$dev"
            echo "  Found device: $dev"
        fi
    done
    
    # Запускаем с пробросом USB устройств
    docker-compose -f docker/docker-compose.yml run --rm $USB_DEVICES microros-esp32-builder

elif [[ "$OSTYPE" == "msys" || "$OSTYPE" == "win32" ]]; then
    echo "🪟 Windows detected"
    # Для Windows: используем стандартный compose
    docker-compose -f docker/docker-compose.yml run --rm microros-esp32-builder
else
    echo "❌ Unsupported OS: $OSTYPE"
    exit 1
fi