cmake_minimum_required(VERSION 3.5)
project(robot_sensor_hub_msg)

# По умолчанию используется C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Поиск зависимостей
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Генерация интерфейсов сообщений
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorReading.msg"
  "msg/DeviceData.msg"
  "msg/SystemSnapshot.msg"
  "msg/SystemMeta.msg"
  "msg/ControlCommand.msg"
  DEPENDENCIES std_msgs
)

# Установка файлов
install(
  DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

ament_package()