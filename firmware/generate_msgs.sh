#!/bin/bash
cd /home/ros2/robot_sensor_hub/robot_sensor_hub

# Удаляем старые директории
rm -rf build_msgs install_msgs

# Создаем новые
mkdir -p build_msgs install_msgs

# Переходим в директорию сборки
cd build_msgs

echo "=== Generating messages with colcon ==="

# Запускаем colcon напрямую, без участия ESP-IDF
colcon build \
  --base-paths .. \
  --build-base build_colcon \
  --install-base install_msgs \
  --merge-install \
  --packages-select robot_sensor_hub

# Абсолютный путь к целевой директории
TARGET_DIR="$PWD/../components/micro_ros_espidf_component/include/robot_sensor_hub"

# Создаем целевую директорию
mkdir -p "$TARGET_DIR"

# Копируем заголовки
cp -r install_msgs/include/robot_sensor_hub/robot_sensor_hub/* "$TARGET_DIR/"

# Проверяем результат
if [ ! -d "$TARGET_DIR/msg" ]; then
    echo "❌ ERROR: Failed to copy message headers!"
    exit 1
fi

echo "✅ Messages generated and copied successfully."
echo "Headers are located in:"
ls -la "$TARGET_DIR/msg/"