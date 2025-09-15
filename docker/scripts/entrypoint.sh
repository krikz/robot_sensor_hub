#!/bin/bash
# entrypoint.sh

# Активация среды ESP-IDF
. $IDF_PATH/export.sh

echo "=== MicroROS ESP32 Builder ==="
echo "Project directory: /project"

# Проверяем наличие проекта
if [ ! -d "/project" ]; then
    echo "❌ Error: project directory not found!"
    exit 1
fi

# Переходим в директорию проекта
cd /project

echo "Environment setup complete!"
echo "Available commands:"
echo "  idf.py build             - Build project"
echo "  idf.py flash             - Flash to ESP32"
echo "  idf.py monitor           - Monitor serial output"
echo "  idf.py menuconfig        - Configure project"
echo ""

# Запускаем переданную команду или bash по умолчанию
exec "$@"