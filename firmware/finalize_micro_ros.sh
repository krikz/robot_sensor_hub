#!/bin/bash
PROJECT_ROOT="/home/ros2/robot_sensor_hub/robot_sensor_hub"
echo "=== Finalizing micro-ROS setup with custom messages ==="

# --- Шаг 1: Убедимся, что скрипты исполняемые ---
chmod +x generate_msgs.sh || true
chmod +x update_libmicroros.sh || true

# --- Шаг 2: Генерируем сообщения ---
echo "✅ Step 1: Generating message headers and sources..."
./generate_msgs.sh

if [ ! -d "$PROJECT_ROOT/build_msgs/build_colcon" ]; then
    echo "❌ ERROR: colcon build failed. Check generate_msgs.sh."
    exit 1
fi

# --- Шаг 3: Находим .c файлы ---
DEVICE_DATA_C="$PROJECT_ROOT/build_msgs/build_colcon/robot_sensor_hub/rosidl_generator_c/robot_sensor_hub/msg/detail/device_data__functions.c"
DEVICE_COMMAND_C="$PROJECT_ROOT/build_msgs/build_colcon/robot_sensor_hub/rosidl_generator_c/robot_sensor_hub/msg/detail/device_command__functions.c"

if [ ! -f "$DEVICE_DATA_C" ] || [ ! -f "$DEVICE_COMMAND_C" ]; then
    echo "❌ ERROR: Generated C files not found!"
    echo "Looking for:"
    echo "  $DEVICE_DATA_C"
    echo "  $DEVICE_COMMAND_C"
    exit 1
fi

# --- Шаг 4: Обновляем libmicroros.a ---
TMP_DIR="/tmp/microros_update_final"
MICRO_ROS_LIB="$PROJECT_ROOT/components/micro_ros_espidf_component/libmicroros.a"

rm -rf "$TMP_DIR"
mkdir -p "$TMP_DIR"
cd "$TMP_DIR"

echo "✅ Step 2: Updating libmicroros.a..."

# Извлекаем существующие объекты
if [ ! -f "$MICRO_ROS_LIB" ]; then
    echo "❌ ERROR: $MICRO_ROS_LIB not found!"
    exit 1
fi
/home/ros2/.espressif/tools/riscv32-esp-elf/esp-14.2.0_20241119/riscv32-esp-elf/bin/riscv32-esp-elf-ar x "$MICRO_ROS_LIB"

# Компилятор и флаги
CC="/home/ros2/.espressif/tools/riscv32-esp-elf/esp-14.2.0_20241119/riscv32-esp-elf/bin/riscv32-esp-elf-gcc"
INCLUDE_FLAGS=(
    "-I$PROJECT_ROOT/components/micro_ros_espidf_component/include"
    "-I$PROJECT_ROOT/components/micro_ros_espidf_component/include/rosidl_generator_c"
    "-I$PROJECT_ROOT/components/micro_ros_espidf_component/include/rosidl_typesupport_c"
    "-I$PROJECT_ROOT/components/micro_ros_espidf_component/include/rosidl_runtime_c"
    "-I$PROJECT_ROOT/components/micro_ros_espidf_component/include/rcutils"
)
CPPFLAGS="-DROSIDL_TYPESUPPORT_INTERFACE__EXPORT_HWMAPPING_C -std=c99"

# Функция для компиляции файла
compile_file() {
    local c_file=$1
    local obj_name=$(basename ${c_file%.c}).obj
    
    echo "Compiling $c_file -> $obj_name"
    $CC ${INCLUDE_FLAGS[@]} $CPPFLAGS -c "$c_file" -o "$obj_name"
    
    if [ $? -ne 0 ]; then
        echo "❌ Compilation of $c_file failed!"
        exit 1
    fi
}

# Компилируем файлы
compile_file "$DEVICE_DATA_C"
compile_file "$DEVICE_COMMAND_C"

# Пересобираем библиотеку
echo "Rebuilding libmicroros.a..."
/home/ros2/.espressif/tools/riscv32-esp-elf/esp-14.2.0_20241119/riscv32-esp-elf/bin/riscv32-esp-elf-ar rc -s libmicroros.a *.obj

# Копируем обратно
cp libmicroros.a "$PROJECT_ROOT/components/micro_ros_espidf_component/"
echo "✅ libmicroros.a successfully updated!"

# --- Шаг 5: Проверяем результат ---
echo "🔍 Verifying symbols in the library..."
/home/ros2/.espressif/tools/riscv32-esp-elf/esp-14.0.2_20240528/riscv32-esp-elf/bin/riscv32-esp-elf-nm "$MICRO_ROS_LIB" | grep -E "robot_sensor_hub__msg__Device(Data|Command)"

echo "✅ Setup complete. You can now run 'idf.py build'."