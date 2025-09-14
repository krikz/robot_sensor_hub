# robot_sensor_hub/DeviceData.msg
uint8 device_type     # тип устройства (AHT30, HX711, FAN)
uint8 device_id       # ID устройства
SensorReading[] readings  # массив показаний этого устройства
uint8 device_status   # статус устройства (0=active, 1=error, 2=calibrating, 3=not_available)