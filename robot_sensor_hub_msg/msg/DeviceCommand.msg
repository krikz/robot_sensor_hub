# DeviceCommand.msg - Команда для управления устройством

# Тип устройства, которому предназначена команда
uint8 device_type

# Идентификатор целевого устройства
uint8 device_id

# Код команды (0=SET_SPEED, 1=TARE_SCALE)
uint8 command_code

# Параметры команды
float32 param_1
float32 param_2

# Временная метка
int64 timestamp