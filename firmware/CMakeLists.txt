# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# Root CMakeLists.txt for colcon
# Root CMakeLists.txt for colcon
#cmake_minimum_required(VERSION 3.5)
project(robot_sensor_hub)

#find_package(ament_cmake REQUIRED)
#find_package(rosidl_generator_c REQUIRED)
#find_package(builtin_interfaces REQUIRED)
#find_package(std_msgs REQUIRED)

#rosidl_generate_interfaces(${PROJECT_NAME}
#  "msg/DeviceData.msg"
#  "msg/DeviceCommand.msg"
#  DEPENDENCIES builtin_interfaces std_msgs
#)

#ament_package()