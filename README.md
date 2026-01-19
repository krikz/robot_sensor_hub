# Robot Sensor Hub - Документация v2.1

## 📋 Описание проекта

**Robot Sensor Hub** - система на базе ESP32 для сбора данных с датчиков и управления устройствами через **request-response протокол** для интеграции с Raspberry Pi и ROS.

## 🎯 Возможности

- 8× AHT30 датчиков (температура/влажность через TCA9548A)
- 1× HX711 (вес)
- 2× вентилятора (PWM + тахометр)
- **Конфигурация через targets** (как в ELRS)
- Request-response протокол через Serial
- Python клиент для Raspberry Pi
- Готово для интеграции с ROS

## 🎨 Система Targets

Targets позволяют настроить какие датчики и на каких портах используются.

### Доступные targets:

**`default`** - 2 AHT30 + HX711 + 2 вентилятора
```bash
pio run -e default -t upload
```

**`full`** - 8 AHT30 + HX711 + 2 вентилятора
```bash
pio run -e full -t upload
```

### Создание своего target:

1. Скопируйте `src/targets/default.h` в `src/targets/my_config.h`
2. Измените настройки:
```c
// Включить/выключить AHT30 на каналах
#define AHT30_CHANNEL_0 1  // 1=включен, 0=выключен
#define AHT30_CHANNEL_1 0
...

// Изменить GPIO пины
#define FAN0_PWM_PIN 25
#define HX711_DOUT_PIN 4
```

3. Добавьте в `platformio.ini`:
```ini
[env:my_config]
platform = ${common.platform}
board = ${common.board}
framework = ${common.framework}
build_flags = -DUSE_TARGET_my_config
```

4. Прошивка: `pio run -e my_config -t upload`

См. `src/targets/README.md` для деталей.

## 📡 Протокол

### Запросы (RPi → ESP32)
```
0                # Список датчиков
1,TYPE,ID        # Читать датчик
2,FAN_ID,SPEED   # Управление вентилятором (0.0-1.0)
3                # Тарировать весы
4                # Все данные
5                # Версия прошивки
```

### Ответы (ESP32 → RPi)
JSON формат:
```json
{"status":0,"sensors":[...]}
{"status":0,"type":0,"id":0,"data":[...]}
{"status":0,"version":{"firmware":"2.1.0","project":"Robot Sensor Hub",...}}
```

Коды: 0=OK, 1=ERROR, 2=INVALID_CMD, 3=INVALID_PARAM

Типы устройств: 0=AHT30, 1=HX711, 2=FAN

Типы данных: 1=Temp(°C), 2=Humidity(%), 3=Weight(g), 4=Speed, 5=RPM

## 🔌 Подключение (по умолчанию)

```
I2C: SDA→GPIO21, SCL→GPIO22
HX711: DAT→GPIO18, CLK→GPIO19
FAN0: PWM→GPIO13, TACHO→GPIO15
FAN1: PWM→GPIO14, TACHO→GPIO16
```

*Пины настраиваются в target файлах*

## 🚀 Быстрый старт

```bash
pip install platformio
git clone https://github.com/krikz/robot_sensor_hub.git
cd robot_sensor_hub

# Прошивка с default конфигурацией
pio run -e default -t upload

# Или с full конфигурацией (все 8 датчиков)
pio run -e full -t upload

# Python клиент
python3 sensor_client.py /dev/ttyUSB0
```

## 🔄 Обновление прошивки

Используйте встроенный bootloader ESP32 для безопасного обновления:

```bash
# Собрать прошивку
pio run -e default

# Обновить ESP32
python3 firmware_update.py /dev/ttyUSB0 .pio/build/default/firmware.bin
```

См. `FIRMWARE_UPDATE.md` для деталей.

## 🐍 Python API

```python
from sensor_client import SensorHubClient

client = SensorHubClient('/dev/ttyUSB0', 115200)
version = client.get_version()
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
├── platformio.ini          # Конфигурация targets
├── src/
│   ├── target.h           # Выбор target
│   ├── targets/           # Target конфигурации
│   │   ├── default.h      # Стандартная
│   │   ├── full_config.h  # Полная
│   │   └── README.md      # Документация
│   ├── main.cpp           # Request-response protocol
│   └── sensors/           # Драйверы (используют target.h)
└── sensor_client.py       # Python client для RPi
```

## 🔧 Troubleshooting

- **Датчики не находятся**: Проверьте target config и I2C GPIO
- **Вентиляторы не работают**: Проверьте PWM GPIO в target
- **Permission denied**: `sudo usermod -a -G dialout $USER`

---

**v2.1** - Targets + Request-response | **v2.0** - PlatformIO | **v1.0** - ESP-IDF+ROS

*Автор: krikz | Apache 2.0*
