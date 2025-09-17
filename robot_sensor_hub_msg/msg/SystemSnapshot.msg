# robot_sensor_hub/SystemSnapshot.msg
std_msgs/Header header          # стандартный ROS header
DeviceData[] devices   # массив данных всех устройств
SystemMeta metadata    # метаданные системы