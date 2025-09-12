#!/bin/bash
PROJECT_ROOT="/home/ros2/robot_sensor_hub/robot_sensor_hub"

cd "$PROJECT_ROOT"

# Пути
MICRO_ROS_LIB="$PROJECT_ROOT/components/micro_ros_espidf_component/libmicroros.a"
TMP_DIR="/tmp/microros_update"

# Удаляем и создаем временную директорию
rm -rf "$TMP_DIR"
mkdir -p "$TMP_DIR"
cd "$TMP_DIR"

echo "=== Updating libmicroros.a ==="

# Проверяем наличие библиотеки
if [ ! -f "$MICRO_ROS_LIB" ]; then
    echo "❌ ERROR: $MICRO_ROS_LIB not found!"
    exit 1
fi

# Извлекаем существующие объектные файлы
/home/ros2/.espressif/tools/riscv32-esp-elf/esp-14.2.0_20241119/riscv32-esp-elf/bin/riscv32-esp-elf-ar x "$MICRO_ROS_LIB"

# Список .c файлов для компиляции
C_FILES=(
    "$PROJECT_ROOT/build_msgs/build_colcon/robot_sensor_hub/rosidl_generator_c/robot_sensor_hub/msg/detail/device_data__functions.c"
    "$PROJECT_ROOT/build_msgs/build_colcon/robot_sensor_hub/rosidl_generator_c/robot_sensor_hub/msg/detail/device_command__functions.c"
)

# Список путей к include
INCLUDE_DIRS=(
    "$PROJECT_ROOT/components/micro_ros_espidf_component/include"
    "$PROJECT_ROOT/components/micro_ros_espidf_component/include/rosidl_generator_c"
    "$PROJECT_ROOT/components/micro_ros_espidf_component/include/rosidl_typesupport_c"
    "$PROJECT_ROOT/components/micro_ros_espidf_component/include/rosidl_runtime_c"
    "$PROJECT_ROOT/components/micro_ros_espidf_component/include/rcutils"
)

# Флаги для компилятора
CPPFLAGS="-DROSIDL_TYPESUPPORT_INTERFACE__EXPORT_HWMAPPING_C -std=c99"

# Компилируем каждый .c файл
for c_file in "${C_FILES[@]}"; do
    if [ ! -f "$c_file" ]; then
        echo "❌ ERROR: Source file $c_file not found!"
        exit 1
    fi
    
    obj_name="$(basename ${c_file%.c}).obj"
    echo "Compiling $c_file -> $obj_name"
    
    # Собираем флаги -I
    INCLUDE_FLAGS=""
    for inc_dir in "${INCLUDE_DIRS[@]}"; do
        INCLUDE_FLAGS="$INCLUDE_FLAGS -I$inc_dir"
    done
    
    /home/ros2/.espressif/tools/riscv32-esp-elf/esp-14.2.0_20241119/riscv32-esp-elf/bin/riscv32-esp-elf-gcc \
        $INCLUDE_FLAGS \
        $CPPFLAGS \
        -c "$c_file" -o "$obj_name"
done

# Пересобираем библиотеку
echo "Rebuilding libmicroros.a..."
/home/ros2/.espressif/tools/riscv32-esp-elf/esp-14.2.0_20241119/riscv32-esp-elf/bin/riscv32-esp-elf-ar rc -s libmicroros.a *.obj

# Копируем обратно
cp libmicroros.a "$PROJECT_ROOT/components/micro_ros_espidf_component/"

echo "✅ libmicroros.a successfully updated with custom messages!"