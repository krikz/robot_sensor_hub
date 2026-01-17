# Robot Sensor Hub - Документация v2.1

## 📋 Описание проекта

**Robot Sensor Hub** - система на базе ESP32 для сбора данных с датчиков и управления устройствами через **request-response протокол** для интеграции с Raspberry Pi и ROS.

## 🎯 Возможности

- 8× AHT30 датчиков (температура/влажность через TCA9548A)
- 1× HX711 (вес)
- 2× вентилятора (PWM + тахометр)
- Request-response протокол через Serial
- Python клиент для Raspberry Pi
- Готово для интеграции с ROS

## 📡 Протокол

### Запросы (RPi → ESP32)
```
0                # Список датчиков
1,TYPE,ID        # Читать датчик
2,FAN_ID,SPEED   # Управление вентилятором (0.0-1.0)
3                # Тарировать весы
4                # Все данные
```

### Ответы (ESP32 → RPi)
JSON формат:
```json
{"status":0,"sensors":[...]}
{"status":0,"type":0,"id":0,"data":[...]}
```

Коды: 0=OK, 1=ERROR, 2=INVALID_CMD, 3=INVALID_PARAM

Типы устройств: 0=AHT30, 1=HX711, 2=FAN

Типы данных: 1=Temp(°C), 2=Humidity(%), 3=Weight(g), 4=Speed, 5=RPM

## 🔌 Подключение

```
I2C: SDA→GPIO21, SCL→GPIO22
HX711: DAT→GPIO18, CLK→GPIO19
FAN0: PWM→GPIO13, TACHO→GPIO15
FAN1: PWM→GPIO14, TACHO→GPIO16
```

## 🚀 Быстрый старт

```bash
pip install platformio
git clone https://github.com/krikz/robot_sensor_hub.git
cd robot_sensor_hub
pio run -t upload
python3 sensor_client.py /dev/ttyUSB0
```

## 🐍 Python API

```python
from sensor_client import SensorHubClient

client = SensorHubClient('/dev/ttyUSB0', 115200)
sensors = client.get_sensors()
data = client.read_sensor(0, 0)
client.set_fan_speed(0, 0.75)
all_data = client.get_all_data()
client.close()
```

## 🤖 ROS интеграция

```python
import rclpy
from rclpy.node import Node
from sensor_client import SensorHubClient

class SensorHubNode(Node):
    def __init__(self):
        super().__init__('sensor_hub_node')
        self.client = SensorHubClient('/dev/ttyUSB0')
        self.timer = self.create_timer(1.0, self.read_sensors)
    
    def read_sensors(self):
        data = self.client.get_all_data()
        # Публикация в ROS топики...
```

## 📦 Структура

```
robot_sensor_hub/
├── platformio.ini
├── src/main.cpp              # Request-response protocol
├── src/sensors/              # AHT30, HX711, Fan drivers
└── sensor_client.py          # Python client for RPi
```

## 🔧 Troubleshooting

- **Датчики не находятся**: I2C GPIO21/22, 3.3V
- **Вентиляторы не работают**: PWM GPIO13/14, 12V
- **Permission denied**: `sudo usermod -a -G dialout $USER`

---

**v2.1** - Request-response | **v2.0** - PlatformIO | **v1.0** - ESP-IDF+ROS

*Автор: krikz | Apache 2.0*
