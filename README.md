# Robot Sensor Hub - Документация v2.0

## 📋 Описание проекта

**Robot Sensor Hub** - это система на базе ESP32 для сбора данных с различных датчиков и управления устройствами. Проект использует **PlatformIO** и предоставляет простой Serial интерфейс для передачи данных и приёма команд.

## 🎯 Основная функциональность

### 📊 Сбор данных с датчиков
- **Датчики температуры/влажности AHT30** (1-8 устройств по I2C через мультиплексор TCA9548A)
- **Тензодатчик HX711** (измерение веса с возможностью калибровки)
- **2 вентилятора/кулера** с полным контролем:
  - PWM управление скоростью (0-100%)
  - Тахометр для измерения RPM (оборотов в минуту)
  - Определение состояния вращения

### 🔧 Исполнение команд
- Установка скорости вентиляторов по команде через Serial
- Калибровка тензодатчика (тарирование) по команде
- Простой текстовый протокол команд

### 📡 Коммуникация
- **Serial UART** для передачи данных и команд
- Скорость: 115200 baud
- JSON формат для данных датчиков
- Простой CSV формат для команд

## ⚙️ Аппаратная конфигурация

### Обязательные компоненты
- **ESP32** (с поддержкой Arduino framework)
- **TCA9548A** - I2C мультиплексор
- **1-8× датчиков AHT30** (подключаются к мультиплексору)
- **1× тензодатчик HX711** 
- **2× 4-пиновых вентилятора** (PWM + тахометр)

### Подключение оборудования
```
TCA9548A (I2C Multiplexer):
  SDA → GPIO21
  SCL → GPIO22  
  VCC → 3.3V
  GND → GND
  
AHT30 Sensors (через TCA9548A):
  Подключаются к каналам 0-7 мультиплексора

HX711 (Weight Sensor):
  DAT → GPIO18
  CLK → GPIO19
  VCC → 3.3V
  GND → GND

FAN0:
  PWM → GPIO13
  TACHO → GPIO15
  
FAN1:
  PWM → GPIO14  
  TACHO → GPIO16
```

## 🚀 Быстрый старт

### Требования к окружению
- **PlatformIO** (VS Code extension или CLI)
- **Python 3.6+** (для PlatformIO)
- USB кабель для прошивки ESP32

### Вариант 1: Через VS Code (рекомендуется)

#### 1. Установка PlatformIO IDE
1. Установите [VS Code](https://code.visualstudio.com/)
2. Установите расширение **PlatformIO IDE** из marketplace
3. Перезапустите VS Code

#### 2. Открытие проекта
```bash
git clone https://github.com/krikz/robot_sensor_hub.git
cd robot_sensor_hub
code .
```

#### 3. Сборка и прошивка
1. Откройте PlatformIO: нажмите на иконку "дома" в левой панели
2. Выберите "Build" для сборки проекта
3. Подключите ESP32 через USB
4. Выберите "Upload" для прошивки
5. Выберите "Monitor" для просмотра Serial выхода

### Вариант 2: Через PlatformIO CLI

#### 1. Установка PlatformIO Core
```bash
# Установка через pip
pip install -U platformio

# Или через curl (Linux/Mac)
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py -o get-platformio.py
python3 get-platformio.py
```

#### 2. Клонирование и сборка
```bash
# Клонирование репозитория
git clone https://github.com/krikz/robot_sensor_hub.git
cd robot_sensor_hub

# Сборка проекта
pio run

# Прошивка ESP32
pio run --target upload

# Мониторинг Serial порта
pio device monitor
```

## 📦 Структура проекта

```
robot_sensor_hub/
├── platformio.ini              # Конфигурация PlatformIO
├── src/                        # Исходный код прошивки
│   ├── main.cpp               # Основная логика
│   └── sensors/               # Драйверы датчиков
│       ├── aht30_reader.h/cpp    # AHT30 температура/влажность
│       ├── hx711_reader.h/cpp    # HX711 тензодатчик
│       └── fan_controller.h/cpp  # Управление вентиляторами
└── README.md
```

## 🔧 Настройка проекта

### Изменение Serial порта
Отредактируйте `platformio.ini`:
```ini
[env:esp32dev]
upload_port = /dev/ttyUSB0  ; Для Linux/Mac
; upload_port = COM3          ; Для Windows
monitor_port = /dev/ttyUSB0
```

### Изменение скорости Serial
В `platformio.ini`:
```ini
monitor_speed = 115200  ; Измените на нужную скорость
```

## 📡 Протокол обмена данными

### Формат данных (ESP32 → Компьютер)
Данные передаются в JSON формате каждую секунду:
```json
{
  "devices": [
    {"type":0,"id":0,"data_type":1,"value":25.30,"error":0},
    {"type":0,"id":0,"data_type":2,"value":45.20,"error":0},
    {"type":1,"id":0,"data_type":3,"value":123.45,"error":0},
    {"type":2,"id":0,"data_type":4,"value":0.75,"error":0},
    {"type":2,"id":0,"data_type":5,"value":1850,"error":0}
  ]
}
```

Где:
- `type`: тип устройства (0=AHT30, 1=HX711, 2=FAN)
- `id`: ID устройства
- `data_type`: тип данных (1=temp, 2=humidity, 3=weight, 4=speed, 5=RPM)
- `value`: значение
- `error`: код ошибки (0 = нет ошибки)

### Формат команд (Компьютер → ESP32)
Команды отправляются в формате CSV через Serial:
```
TYPE,ID,COMMAND,PARAMETER
```

Примеры:
```bash
# Установить скорость вентилятора 0 на 75%
2,0,0,0.75

# Установить скорость вентилятора 1 на 50%
2,1,0,0.50

# Тарировать весы
1,0,1,0
```

Коды команд:
- `0` - установить скорость вентилятора (для TYPE=2)
- `1` - тарировать весы (для TYPE=1)

## 🖥️ Использование

### Просмотр данных через Serial Monitor

#### VS Code + PlatformIO
1. Нажмите на иконку "Serial Monitor" в нижней панели
2. Данные будут отображаться каждую секунду

#### PlatformIO CLI
```bash
pio device monitor
```

#### Screen (Linux/Mac)
```bash
screen /dev/ttyUSB0 115200
```

#### Minicom (Linux)
```bash
minicom -D /dev/ttyUSB0 -b 115200
```

### Отправка команд

#### Через Serial Monitor в VS Code/PlatformIO
Введите команду в поле ввода и нажмите Enter:
```
2,0,0,0.75
```

#### Через терминал (Linux/Mac)
```bash
echo "2,0,0,0.75" > /dev/ttyUSB0
```

#### Через Python скрипт
```python
import serial
import time

# Подключение к ESP32
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)

# Установка скорости вентилятора
ser.write(b'2,0,0,0.75\n')

# Чтение данных
while True:
    if ser.in_waiting:
        line = ser.readline().decode('utf-8').strip()
        print(line)
```

## 🐍 Пример Python скрипта для чтения данных

Создайте файл `read_sensors.py`:

```python
import serial
import json
import time

def main():
    # Подключение к ESP32
    port = '/dev/ttyUSB0'  # Измените на ваш порт
    baudrate = 115200
    
    print(f"Connecting to {port}...")
    ser = serial.Serial(port, baudrate, timeout=1)
    time.sleep(2)
    
    print("Reading sensor data...")
    buffer = ""
    
    while True:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            buffer += data
            
            # Поиск полного JSON объекта
            if '{' in buffer and '}' in buffer:
                start = buffer.index('{')
                end = buffer.index('}', start) + 1
                json_str = buffer[start:end]
                buffer = buffer[end:]
                
                try:
                    data = json.loads(json_str)
                    print(f"\n--- Sensor Snapshot ---")
                    for device in data['devices']:
                        print(f"Type: {device['type']}, ID: {device['id']}, "
                              f"DataType: {device['data_type']}, Value: {device['value']}")
                except json.JSONDecodeError:
                    pass

if __name__ == '__main__':
    main()
```

Запуск:
```bash
python3 read_sensors.py
```

## 🔍 Диагностика и отладка

### Проблема: Датчики не определяются

**Проверка I2C устройств:**
- Убедитесь в правильности подключения SDA (GPIO21) и SCL (GPIO22)
- Проверьте питание 3.3V на всех устройствах
- В Serial Monitor должны быть сообщения "[AHT30] Sensor found on channel X"

### Проблема: Вентиляторы не вращаются

**Решение:**
- Проверьте подключение PWM пинов (GPIO13, GPIO14)
- Убедитесь что вентиляторы подключены к питанию (обычно 12V)
- Отправьте команду установки скорости: `2,0,0,0.75`
- Минимальная рабочая скорость обычно 10-20%

### Проблема: Не удается прошить ESP32

**Решение:**
- Проверьте USB кабель (должен поддерживать передачу данных)
- Убедитесь что выбран правильный порт в `platformio.ini`
- Попробуйте нажать кнопку BOOT на ESP32 при прошивке
- Проверьте драйверы USB-UART (CP2102, CH340 и т.д.)

### Включение отладочных сообщений

В `platformio.ini`:
```ini
build_flags = 
    -DCORE_DEBUG_LEVEL=5  ; 0=None, 1=Error, 2=Warn, 3=Info, 4=Debug, 5=Verbose
```

## 🛠️ Расширение функциональности

### Добавление нового датчика

1. Создайте файлы драйвера в `src/sensors/`:
```cpp
// new_sensor.h
#pragma once
void init_new_sensor();
float read_new_sensor();

// new_sensor.cpp
#include "new_sensor.h"
void init_new_sensor() { /* ... */ }
float read_new_sensor() { /* ... */ }
```

2. Подключите в `main.cpp`:
```cpp
#include "sensors/new_sensor.h"

void setup() {
    // ...
    init_new_sensor();
}

void loop() {
    float value = read_new_sensor();
    // Добавьте в publish_sensor_data()
}
```

### Добавление библиотеки

Отредактируйте `platformio.ini`:
```ini
lib_deps = 
    adafruit/Adafruit AHTX0@^2.0.5
    bogde/HX711@^0.7.5
    your-library/YourLib@^1.0.0  ; Добавьте сюда
```

## 📊 Технические характеристики

- **Частота публикации данных:** 1 Гц (1 раз в секунду)
- **Скорость Serial:** 115200 baud
- **PWM частота вентиляторов:** 25 кГц
- **PWM разрешение:** 8 бит (0-255)
- **I2C частота:** 100 кГц (стандартный режим)
- **Поддерживаемые датчики AHT30:** до 8 штук

## 🔄 История версий

### v2.0 (Текущая версия) - PlatformIO
- ✅ Переход на PlatformIO
- ✅ Удаление зависимостей micro-ROS
- ✅ Простой Serial интерфейс
- ✅ JSON формат данных
- ✅ CSV формат команд
- ✅ Arduino framework
- ✅ Упрощенная сборка и прошивка

### v1.0 (Старая версия) - ESP-IDF + micro-ROS
- ESP-IDF v5.5.1
- micro-ROS интеграция
- UART транспорт для micro-ROS
- ROS2 топики

## 📝 Лицензия

Проект использует лицензию Apache 2.0.

## 🤝 Контакты

GitHub: https://github.com/krikz/robot_sensor_hub

---

*Последнее обновление: январь 2026*  
*Совместимость: PlatformIO, Arduino Framework, ESP32*  
*Автор: krikz*
