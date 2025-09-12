# DeviceData.msg - Универсальное сообщение о состоянии устройства

# Тип устройства (0=AHT30, 1=HX711, 2=FAN)
uint8 device_type

# Идентификатор экземпляра устройства
# Для AHT30: номер канала (0-7)
# Для HX711: 0
# Для FAN: 0 или 1
uint8 device_id

# Тип данных в поле 'value'
# 1 = temperature (°C)
# 2 = humidity (%)
# 3 = weight (kg)
# 4 = speed (0.0-1.0)
uint8 data_type

# Значение данных
float32 value

# Код ошибки (0=OK, 1=NO_RESPONSE, 2=SENSOR_ERROR, 3=DEVICE_NOT_FOUND и т.д.)
uint8 error_code

# Временная метка (наносекунды с эпохи Unix)
int64 timestamp