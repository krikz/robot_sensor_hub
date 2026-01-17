# Быстрый старт / Quick Start Guide

## 🚀 Установка и запуск за 5 минут

### Шаг 1: Установка PlatformIO

**Вариант А: VS Code (рекомендуется)**
1. Скачайте [VS Code](https://code.visualstudio.com/)
2. Установите расширение **PlatformIO IDE** из Marketplace
3. Перезапустите VS Code

**Вариант Б: CLI**
```bash
pip install -U platformio
```

### Шаг 2: Клонирование проекта

```bash
git clone https://github.com/krikz/robot_sensor_hub.git
cd robot_sensor_hub
```

### Шаг 3: Сборка и прошивка

**VS Code:**
1. Откройте папку проекта в VS Code
2. Откройте PlatformIO (иконка "дома" слева)
3. Нажмите **Build** (🔨)
4. Подключите ESP32 через USB
5. Нажмите **Upload** (➡️)
6. Нажмите **Monitor** (🔌) для просмотра вывода

**CLI:**
```bash
# Сборка
pio run

# Прошивка (измените порт при необходимости)
pio run --target upload

# Мониторинг
pio device monitor
```

### Шаг 4: Просмотр данных

**Вариант 1: Встроенный Serial Monitor**
- В VS Code: нажмите **Monitor** в PlatformIO
- В CLI: `pio device monitor`

**Вариант 2: Python скрипт**
```bash
python3 read_sensors.py /dev/ttyUSB0 115200
```

### Шаг 5: Отправка команд

**Установить скорость вентилятора на 75%:**
```bash
python3 send_command.py /dev/ttyUSB0 "2,0,0,0.75"
```

**Тарировать весы:**
```bash
python3 send_command.py /dev/ttyUSB0 "1,0,1,0"
```

---

## 📋 Подключение оборудования

### Минимальная конфигурация

```
ESP32 Connections:
├── I2C (Temperature/Humidity sensors)
│   ├── SDA → GPIO21
│   └── SCL → GPIO22
├── HX711 (Weight sensor)
│   ├── DAT → GPIO18
│   └── CLK → GPIO19
└── FANs (Cooling fans)
    ├── FAN0 PWM → GPIO13
    ├── FAN0 TACHO → GPIO15
    ├── FAN1 PWM → GPIO14
    └── FAN1 TACHO → GPIO16
```

### Схема подключения TCA9548A

```
TCA9548A Multiplexer:
  VCC → 3.3V
  GND → GND
  SDA → GPIO21 (ESP32)
  SCL → GPIO22 (ESP32)
  
  Channel 0-7 → AHT30 sensors
```

---

## 🔧 Решение проблем

### Проблема: Не могу прошить ESP32

**Решение:**
1. Проверьте USB кабель (должен поддерживать данные)
2. Измените порт в `platformio.ini`:
   ```ini
   upload_port = /dev/ttyUSB0  ; Linux/Mac
   ; upload_port = COM3         ; Windows
   ```
3. Нажмите кнопку BOOT при прошивке

### Проблема: Датчики не определяются

**Решение:**
1. Проверьте подключение SDA (21) и SCL (22)
2. Проверьте питание 3.3V
3. В Serial Monitor должно быть: `[AHT30] Sensor found on channel X`

### Проблема: Permission denied на Linux

**Решение:**
```bash
sudo usermod -a -G dialout $USER
# Перелогиньтесь
```

---

## 📡 Формат данных

### JSON вывод (каждую секунду)
```json
{
  "devices": [
    {"type":0,"id":0,"data_type":1,"value":25.3,"error":0},
    {"type":2,"id":0,"data_type":4,"value":0.75,"error":0}
  ]
}
```

### Команды (CSV формат)
```
TYPE,ID,COMMAND,PARAMETER

Примеры:
2,0,0,0.75  - Fan 0 speed to 75%
1,0,1,0     - Tare scale
```

---

## 📚 Дополнительная информация

- **Полная документация:** `README.md`
- **Архивные файлы:** `ARCHIVE.md`
- **Примеры Python:** `read_sensors.py`, `send_command.py`

---

## 💡 Совет

Для быстрой проверки работоспособности:
1. Прошейте ESP32
2. Откройте Serial Monitor (115200 baud)
3. Вы должны увидеть JSON данные каждую секунду
4. Отправьте команду `2,0,0,0.5` для проверки управления

**Готово! Ваш Robot Sensor Hub работает! 🎉**
