# Быстрый старт / Quick Start Guide

## 🚀 За 5 минут

### 1. Установка PlatformIO
```bash
pip install platformio
```

### 2. Прошивка ESP32
```bash
git clone https://github.com/krikz/robot_sensor_hub.git
cd robot_sensor_hub
pio run -t upload
```

### 3. Использование

**Интерактивный режим:**
```bash
python3 sensor_client.py /dev/ttyUSB0
```

**Из Python кода:**
```python
from sensor_client import SensorHubClient

client = SensorHubClient('/dev/ttyUSB0', 115200)

# Список датчиков
sensors = client.get_sensors()

# Чтение датчика
data = client.read_sensor(0, 0)  # AHT30 ID 0

# Управление вентилятором
client.set_fan_speed(0, 0.75)  # Fan 0 -> 75%

client.close()
```

## 📡 Протокол

**Запросы (отправить в Serial):**
```
0           # Список датчиков
1,0,0       # Читать AHT30[0]
2,0,0.75    # Вентилятор 0 -> 75%
3           # Тарировать весы
4           # Все данные
```

**Ответы (JSON):**
```json
{"status":0,"sensors":[...]}
{"status":0,"type":0,"id":0,"data":[...]}
```

## 🔌 Подключение

```
I2C: GPIO21 (SDA), GPIO22 (SCL)
HX711: GPIO18 (DAT), GPIO19 (CLK)
FAN0: GPIO13 (PWM), GPIO15 (TACHO)
FAN1: GPIO14 (PWM), GPIO16 (TACHO)
```

## 🤖 ROS интеграция

```python
import rclpy
from rclpy.node import Node
from sensor_client import SensorHubClient

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_hub')
        self.client = SensorHubClient('/dev/ttyUSB0')
        self.timer = self.create_timer(1.0, self.read_data)
    
    def read_data(self):
        data = self.client.get_all_data()
        self.get_logger().info(f'{data}')
```

**Готово! 🎉**
