# Robot Sensor Hub - Документация

## 📋 Описание проекта

**Robot Sensor Hub** - это интеллектуальная система на базе ESP32 для сбора данных с различных датчиков и управления устройствами через micro-ROS. Система предоставляет сырые данные и выполняет команды, а сложная логика управления реализуется на стороне ROS2.

## 🎯 Основная функциональность

### 📊 Сбор данных с датчиков
- **Датчики температуры/влажности AHT30** (1-8 устройств по I2C через мультиплексор TCA9548A)
- **Тензодатчик HX711** (измерение веса с возможностью калибровки)
- **2 вентилятора/кулера** с полным контролем:
  - PWM управление скоростью (0-100%)
  - Тахометр для измерения RPM (оборотов в минуту)
  - Определение состояния вращения

### 🔧 Исполнение команд
- Установка скорости вентиляторов по команде
- Калибровка тензодатчика (тарирование) по команде
- Пассивное ожидание управляющих команд от ROS2-нод

### 📡 Коммуникация
- **Micro-ROS** для полной интеграции с ROS2 экосистемой
- **UART транспорт** для связи с micro-ROS агентом (вместо UDP)
- Два основных топика:
  - `device/snapshot` - публикация данных всех датчиков (1 Гц)
  - `device/command` - подписка на команды управления

### 🔌 UART Конфигурация
Система использует **два независимых UART порта**:

**UART0 (GPIO1/GPIO3)** - Консоль ESP32
- Прошивка и отладка через USB-UART
- Вывод логов ESP_LOG
- Скорость: 115200 baud

**UART2 (GPIO27/GPIO26)** - micro-ROS Agent
- Связь с micro-ROS Agent на компьютере
- TX: GPIO 27
- RX: GPIO 26  
- Скорость: 115200 baud

## 🏗️ Структура данных

### Сообщение DeviceSnapshot
```yaml
devices: DeviceData[]  # Массив данных со всех активных устройств
```

### Сообщение DeviceData
```yaml
device_type: uint8    # 0=AHT30, 1=HX711, 2=FAN
device_id: uint8      # ID устройства (0-7 для AHT30, 0 для HX711, 0-1 для FAN)
data_type: uint8      # 1=temp, 2=humidity, 3=weight, 4=speed, 5=RPM
value: float         # Значение измерения
error_code: uint8    # Код ошибки (0=нет ошибки)
```

### Сообщение DeviceCommand
```yaml
device_type: uint8    # Тип целевого устройства
device_id: uint8      # ID целевого устройства  
command_code: uint8   # 0=set_speed, 1=tare_scale
param_1: float       # Параметр (скорость для FAN: 0.0-1.0)
param_2: float       # Запасной параметр
```

## ⚙️ Аппаратная конфигурация

### Обязательные компоненты
- **ESP32** (с поддержкой WiFi/Ethernet)
- **TCA9548A** - I2C мультиплексор (обязателен)
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

UART (micro-ROS):
  TX → GPIO27 (к RX USB-UART конвертера)
  RX → GPIO26 (к TX USB-UART конвертера)
  GND → GND (общая земля обязательна!)

UART0 (Console/Flash):
  TX → GPIO1 (стандартный UART0)
  RX → GPIO3 (стандартный UART0)
```

**⚠️ Важно:** Требуется **два** USB-UART адаптера:
- Один для консоли/прошивки (GPIO1/3)
- Второй для micro-ROS Agent (GPIO27/26)

## 🔧 Программная конфигурация

### Обязательные настройки menuconfig
```bash
idf.py menuconfig
```
- **Micro-ROS → Transport** → WiFi или Ethernet
- **Micro-ROS Agent IP/Port** → Адрес агента
- **I2C Settings** → Проверить пины SDA/SCL
- **GPIO Settings** → Настроить пины для HX711 и FAN

### Особенности работы
- Система автоматически определяет наличие датчиков AHT30
- Поддерживается от 1 до 8 датчиков (гибкая конфигурация)
- Отсутствующие датчики игнорируются (возвращают NaN)
- Вентиляторы работают независимо с индивидуальным управлением

## 🚀 Быстрый старт

### Требования к окружению
- **ESP-IDF v5.2+** (рекомендуется v5.5.1)
- **Docker** (опционально, для изолированной сборки)
- **ROS2 Humble** на хост-машине
- **Два USB-UART адаптера** (один для прошивки, один для micro-ROS)

---

## 🏗️ Сборка проекта

### Вариант 1: Сборка через Docker (рекомендуется)

Docker обеспечивает чистое окружение без конфликтов с хостовой системой ROS2.

#### 1.1. Подготовка
```bash
git clone --recursive https://github.com/krikz/robot_sensor_hub.git
cd robot_sensor_hub
```

#### 1.2. Запуск Docker-контейнера
```bash
cd docker
docker-compose up -d
docker exec -it robot_sensor_hub_builder bash
```

#### 1.3. Сборка внутри контейнера
```bash
cd /workspace/firmware
source /opt/esp/idf/export.sh

# Чистая сборка
idf.py fullclean
idf.py build
```

#### 1.4. Прошивка (из контейнера)
```bash
# Найдите порт на хост-машине
idf.py -p /dev/ttyUSB0 flash monitor
```

---

### Вариант 2: Сборка на хост-машине

**⚠️ Внимание:** При наличии ROS2 на хост-системе могут возникать конфликты переменных окружения!

#### 2.1. Установка ESP-IDF
```bash
mkdir -p ~/esp
cd ~/esp
git clone -b v5.5.1 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32
```

#### 2.2. Клонирование проекта
```bash
cd ~
git clone --recursive https://github.com/krikz/robot_sensor_hub.git
cd robot_sensor_hub/firmware
```

#### 2.3. Сборка с изоляцией ROS2
```bash
# ВАЖНО: Отключаем переменные ROS2 перед сборкой
cd ~/robot_sensor_hub/firmware
source ~/esp/esp-idf/export.sh

# Очистка конфликтующих переменных
unset RMW_IMPLEMENTATION
export RMW_IMPLEMENTATION=rmw_microxrcedds

# Чистая сборка micro-ROS
idf.py clean-microros
idf.py build
```

#### 2.4. Прошивка
```bash
# Прошивка через UART0 (GPIO1/3)
idf.py -p /dev/ttyUSB0 flash

# Мониторинг логов
idf.py -p /dev/ttyUSB0 monitor
```

---

## 🔧 Настройка проекта (menuconfig)

### Изменение конфигурации UART для micro-ROS
```bash
idf.py menuconfig
```

Навигация:
```
→ micro-ROS Settings
  → micro-ROS network interface select
    ✓ Micro XRCE-DDS over UART
  → UART Settings
    → UART TX pin: 27
    → UART RX pin: 26
```

### Другие важные настройки
```
→ Component config
  → ESP System Settings
    → Channel for console output
      ✓ UART0 (default)
```

---

## 🖥️ Запуск micro-ROS Agent

### На хост-машине (после прошивки ESP32)

#### Через Docker (рекомендуется)
```bash
# Подключите второй USB-UART к GPIO27/26
# Определите порт (обычно /dev/ttyUSB1 если ttyUSB0 занят консолью)

docker run -it --rm \
  -v /dev:/dev \
  --privileged \
  --net=host \
  microros/micro-ros-agent:humble \
  serial --dev /dev/ttyUSB1 -v6
```

#### Нативная установка
```bash
# Установка (если ещё не установлен)
sudo apt install ros-humble-micro-ros-agent

# Запуск
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB1 -v6
```

#### Проверка подключения
```bash
# В другом терминале
ros2 topic list

# Должны появиться топики:
# /device/snapshot
# /device/command
# /parameter_events
# /rosout
```

---

## 🔍 Диагностика и отладка

### Проверка портов
```bash
# Список USB устройств
ls -l /dev/ttyUSB*

# Мониторинг консоли ESP32 (UART0 на GPIO1/3)
idf.py -p /dev/ttyUSB0 monitor

# Micro-ROS Agent должен быть на другом порту (UART2 на GPIO27/26)
# Обычно это /dev/ttyUSB1
```

### Логи ESP32
```bash
# При подключении к консоли вы увидите:
# [sensor_hub] Initializing i2cdev driver...
# [sensor_hub] i2cdev driver initialized.
# [sensor_hub] UART2 transport configured for micro-ROS (TX=27, RX=26)
# [sensor_hub] micro-ROS node created
```

### Решение проблем

**Проблема**: `RMW_IMPLEMENTATION` конфликт при сборке
```bash
# Решение: явно переопределить переменную
unset RMW_IMPLEMENTATION
export RMW_IMPLEMENTATION=rmw_microxrcedds
idf.py clean-microros
idf.py build
```

**Проблема**: Не определяются кастомные сообщения
```bash
# Решение: пересобрать micro-ROS с чистого листа
cd firmware
idf.py clean-microros
rm -rf build
idf.py build
```

**Проблема**: Agent не подключается
- Проверьте правильность подключения UART2 (GPIO27/26)
- Убедитесь что используете правильный порт (`/dev/ttyUSB1`)
- Проверьте общую землю (GND) между ESP32 и USB-UART

---

## 📦 Структура проекта

```
robot_sensor_hub/
├── firmware/                          # Прошивка ESP32
│   ├── main/
│   │   ├── main.c                    # Основная логика
│   │   ├── esp32_serial_transport.c  # UART транспорт для micro-ROS
│   │   ├── esp32_serial_transport.h
│   │   └── sensors/                  # Драйверы датчиков
│   │       ├── aht30_reader.c        # AHT30 температура/влажность
│   │       ├── hx711_reader.c        # HX711 тензодатчик
│   │       └── fan_controller.c      # Управление вентиляторами
│   ├── components/
│   │   └── micro_ros_espidf_component/  # Компонент micro-ROS
│   │       └── colcon.meta           # Конфигурация сборки (UART!)
│   └── CMakeLists.txt
├── robot_sensor_hub_msg/             # ROS2 сообщения
│   └── msg/
│       ├── DeviceData.msg
│       ├── DeviceSnapshot.msg
│       └── DeviceCommand.msg
├── docker/                           # Docker окружение
│   ├── Dockerfile
│   └── docker-compose.yml
├── cooling_controller.py             # Пример ROS2 контроллера
└── README.md
```

---

## 🔧 Технические детали

### Изменения транспорта (UDP → UART)

**Было (старая версия):**
- UDP транспорт через WiFi
- Требовалась настройка сети
- CONFIG_MICRO_ROS_AGENT_IP и PORT

**Стало (текущая версия):**
- UART транспорт через GPIO27/26
- Не требуется настройка сети
- Более стабильное соединение
- Два независимых UART (консоль + micro-ROS)

### Файлы конфигурации транспорта

**firmware/components/micro_ros_espidf_component/colcon.meta:**
```json
{
    "names": {
        "rmw_microxrcedds": {
            "cmake-args": [
                "-DRMW_UXRCE_TRANSPORT=custom"  // Вместо "udp"
            ]
        }
    }
}
```

**firmware/main/main.c:**
```c
// Использование UART2 вместо UART0
static size_t uart_port = UART_NUM_2;
rmw_uros_set_custom_transport(
    true,
    (void *) &uart_port,
    esp32_serial_open,
    esp32_serial_close,
    esp32_serial_write,
    esp32_serial_read
);
```

---

## 🚀 Быстрый старт (краткая версия)

### 1. Клонирование и настройка
## 🚀 Быстрый старт (краткая версия)

### 1. Сборка
```bash
git clone --recursive https://github.com/krikz/robot_sensor_hub.git
cd robot_sensor_hub/firmware

# Через Docker (рекомендуется)
cd ../docker && docker-compose up -d
docker exec -it robot_sensor_hub_builder bash
cd /workspace/firmware && source /opt/esp/idf/export.sh
idf.py build

# Или на хосте
source ~/esp/esp-idf/export.sh
unset RMW_IMPLEMENTATION && export RMW_IMPLEMENTATION=rmw_microxrcedds
idf.py build
```

### 2. Прошивка (через UART0 на GPIO1/3)
```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

### 3. Запуск micro-ROS Agent (через UART2 на GPIO27/26)
```bash
# В отдельном терминале
docker run -it --rm -v /dev:/dev --privileged --net=host \
  microros/micro-ros-agent:humble serial --dev /dev/ttyUSB1 -v6
```

### 4. Проверка
```bash
# Список топиков
ros2 topic list

# Просмотр данных
ros2 topic echo /device/snapshot
```

---

## 🎛️ Управление системой

### Ручное управление вентиляторами
```bash
# Установка скорости вентилятора 0 на 75%
ros2 topic pub /device/command robot_sensor_hub_msg/msg/DeviceCommand "
device_type: 2
device_id: 0  
command_code: 0
param_1: 0.75"
```

### Калибровка тензодатчика
```bash
# Тарирование весов
ros2 topic pub /device/command robot_sensor_hub_msg/msg/DeviceCommand "
device_type: 1
device_id: 0
command_code: 1"
```

### Мониторинг данных
```bash
# Просмотр всех данных
ros2 topic echo /device/snapshot

# Только температура
ros2 topic echo /device/snapshot | grep "temperature"
```

## 🐍 Пример использования (Python ROS2 нода)

Система предназначена для управления высокоуровневыми ROS2-нодами. Пример реализации ПИД-регулятора:

### Запуск примера управления
```bash
python3 cooling_controller.py
```

### Принцип работы внешнего контроллера
1. **Подписка** на топик `/device/snapshot` для получения данных
2. **Анализ** температур с датчиков (вход/выход системы)
3. **Расчет** управляющего воздействия по ПИД-алгоритму
4. **Публикация** команд в топик `/device/command`

### Конфигурация внешнего контроллера
```python
# В cooling_controller.py
SETPOINT_DELTA = -2.0    # Целевая разница температур (выход - вход)
MIN_FAN_SPEED = 0.1      # Минимальная скорость вентилятора (10%)
SAMPLE_TIME = 1.0        # Интервал обновления (1 секунда)
```

## 📊 Визуализация данных

Пример скрипта включает графический интерфейс для мониторинга:

### Отображаемые параметры
- Температура на входе и выходе системы
- Разница температур (outlet - inlet)
- Ошибка регулирования
- Скорость вентилятора (PWM %)
- RPM вентилятора (обороты/минуту)

## 🔧 Диагностика и устранение неисправностей

### Проверка подключения UART
```bash
# Список всех USB-UART устройств
ls -l /dev/ttyUSB*

# Должно быть минимум 2 устройства:
# /dev/ttyUSB0 - консоль ESP32 (GPIO1/3)
# /dev/ttyUSB1 - micro-ROS Agent (GPIO27/26)
```

### Мониторинг логов ESP32
```bash
# Подключение к консоли через UART0
idf.py -p /dev/ttyUSB0 monitor

# При старте должны появиться сообщения:
# [sensor_hub] Initializing i2cdev driver...
# [sensor_hub] UART2 transport configured for micro-ROS (TX=27, RX=26)
# [sensor_hub] micro-ROS node created
```

### Проверка связи с Agent
```bash
# Запуск Agent с подробными логами
docker run -it --rm -v /dev:/dev --privileged --net=host \
  microros/micro-ros-agent:humble serial --dev /dev/ttyUSB1 -v6

# При успешном подключении увидите:
# [1234567890.123456] info | UDPv4AgentLinux.cpp | recv_message | ...
```

### Частые проблемы

#### 1. Датчики AHT30 не определяются
```bash
# Проверка I2C (из ESP32 monitor)
# Должны появиться адреса 0x70 (TCA9548A) и 0x38 (AHT30)
```
**Решение:**
- Проверить подключение TCA9548A к GPIO21/22
- Проверить питание 3.3V на всех устройствах
- Убедиться в правильной распайке каналов мультиплексора

#### 2. Ошибка сборки: "RMW_IMPLEMENTATION conflict"
```bash
# Проблема: Переменные окружения ROS2 конфликтуют с micro-ROS
```
**Решение:**
```bash
cd firmware
unset RMW_IMPLEMENTATION
export RMW_IMPLEMENTATION=rmw_microxrcedds
idf.py clean-microros
idf.py build
```

#### 3. Не определяются кастомные сообщения robot_sensor_hub_msg
```bash
# Проблема: libmicroros.a не содержит кастомные типы
```
**Решение:**
```bash
# Полная пересборка micro-ROS
cd firmware
idf.py clean-microros
rm -rf build
idf.py build
```

#### 4. micro-ROS Agent не подключается
```bash
# Проблема: Нет связи по UART2
```
**Решение:**
- Проверьте подключение GPIO27 (TX ESP32) → RX USB-UART
- Проверьте подключение GPIO26 (RX ESP32) → TX USB-UART  
- **ОБЯЗАТЕЛЬНО** соедините GND ESP32 и USB-UART
- Убедитесь что используете правильный порт (обычно `/dev/ttyUSB1`)
- Проверьте права доступа: `sudo chmod 666 /dev/ttyUSB1`

#### 5. Вентиляторы не вращаются
#### 5. Вентиляторы не вращаются
```bash
# Проблема: Нет PWM сигнала или неправильное подключение
```
**Решение:**
- Проверить подключение PWM пинов (GPIO13 и GPIO14)
- Убедиться что вентиляторы подключены к питанию (обычно 12V)
- Убедиться в отправке команд управления через `/device/command`
- Проверить что минимальная скорость > 0.1 (10%)

#### 6. Сборка в Docker не работает
```bash
# Проблема: Не хватает прав или порты не проброшены
```
**Решение:**
```bash
# Добавить пользователя в группу docker
sudo usermod -aG docker $USER
# Перелогиниться

# Пробросить USB устройства в контейнер
docker run ... --device=/dev/ttyUSB0 --device=/dev/ttyUSB1 ...
```

---

## � Дополнительные ресурсы

### Документация
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/)
- [micro-ROS Documentation](https://micro.ros.org/docs/overview/features/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)

### Примеры кода
- `cooling_controller.py` - Пример ПИД-регулятора на Python
- `firmware/main/` - Примеры работы с датчиками и actuators

### Инструменты отладки
```bash
# ROS2 утилиты
ros2 topic list              # Список топиков
ros2 topic echo /topic_name  # Просмотр сообщений
ros2 topic hz /topic_name    # Частота публикации
ros2 topic info /topic_name  # Информация о топике

# ESP-IDF утилиты  
idf.py monitor              # Мониторинг последовательного порта
idf.py menuconfig           # Настройка проекта
idf.py size                 # Размер прошивки
idf.py app-flash            # Прошивка только приложения (быстрее)
```

---

## 🎓 Обучающие материалы

### Архитектура системы
```
┌─────────────────────────────────────────────────┐
│                  ROS2 Host PC                    │
│  ┌───────────────────────────────────────────┐  │
│  │   ROS2 Node (cooling_controller.py)       │  │
│  │   - Подписка на /device/snapshot          │  │
│  │   - ПИД-регулятор                         │  │
│  │   - Публикация в /device/command          │  │
│  └───────────────────────────────────────────┘  │
│                       ↕                          │
│  ┌───────────────────────────────────────────┐  │
│  │      micro-ROS Agent (serial)             │  │
│  │      /dev/ttyUSB1 (115200 baud)          │  │
│  └───────────────────────────────────────────┘  │
└─────────────────────────────────────────────────┘
                        ↕
              UART2 (GPIO27/26)
                        ↕
┌─────────────────────────────────────────────────┐
│              ESP32 (Robot Sensor Hub)            │
│  ┌───────────────────────────────────────────┐  │
│  │   micro-ROS Client                        │  │
│  │   - Publisher: /device/snapshot (1 Hz)    │  │
│  │   - Subscriber: /device/command           │  │
│  └───────────────────────────────────────────┘  │
│  ┌───────────────────────────────────────────┐  │
│  │   Sensor Drivers                          │  │
│  │   - TCA9548A + 8× AHT30 (I2C)            │  │
│  │   - HX711 (Weight sensor)                 │  │
│  │   - 2× FANs (PWM + Tachometer)           │  │
│  └───────────────────────────────────────────┘  │
│                                                  │
│  Console: UART0 (GPIO1/3) → /dev/ttyUSB0       │
└─────────────────────────────────────────────────┘
```

### Рекомендуемый workflow

1. **Разработка**
   ```bash
   # Редактирование кода
   vim firmware/main/main.c
   
   # Сборка
   cd firmware && idf.py build
   
   # Прошивка и мониторинг
   idf.py -p /dev/ttyUSB0 flash monitor
   ```

2. **Тестирование**
   ```bash
   # Терминал 1: Monitor ESP32
   idf.py -p /dev/ttyUSB0 monitor
   
   # Терминал 2: micro-ROS Agent
   docker run -it --rm -v /dev:/dev --privileged --net=host \
     microros/micro-ros-agent:humble serial --dev /dev/ttyUSB1 -v6
   
   # Терминал 3: ROS2 команды
   ros2 topic echo /device/snapshot
   ```

3. **Отладка**
   ```bash
   # Добавление логов в код
   ESP_LOGI(TAG, "Debug message: %d", value);
   
   # Просмотр логов
   idf.py monitor
   ```

---

## 📝 Лицензия

Проект использует лицензию Apache 2.0. Подробности в файле LICENSE.

## 🤝 Разработка и поддержка

### История изменений

**v2.0 (Текущая версия)**
- ✅ Переход с UDP на UART транспорт для micro-ROS
- ✅ Использование двух независимых UART портов
- ✅ Улучшенная стабильность связи
- ✅ Упрощенная конфигурация (не требуется настройка сети)
- ✅ Поддержка ESP-IDF v5.5.1
- ✅ Обновленная документация

**v1.0 (Старая версия)**
- UDP транспорт через WiFi
- Единственный UART для консоли
- ESP-IDF v5.1-5.2

### Известные ограничения
- Максимум 8 датчиков AHT30 (ограничение мультиплексора TCA9548A)
- Требуется два USB-UART адаптера
- Тахометр вентиляторов требует 4-pin PWM вентиляторы

### Roadmap
- [ ] Поддержка Ethernet транспорта (альтернатива UART)
- [ ] Web-интерфейс для мониторинга
- [ ] Поддержка дополнительных типов датчиков
- [ ] OTA (Over-The-Air) обновления прошивки

### Структура репозитория
```
robot_sensor_hub/
├── firmware/                          # Прошивка ESP32
│   ├── main/
│   │   ├── main.c                    # Основная логика
│   │   ├── esp32_serial_transport.c  # UART транспорт
│   │   └── sensors/                  # Драйверы датчиков
│   ├── components/
│   │   └── micro_ros_espidf_component/
│   │       └── colcon.meta           # Конфигурация UART
│   └── sdkconfig                     # Настройки проекта
├── robot_sensor_hub_msg/             # ROS2 сообщения
│   └── msg/
│       ├── DeviceData.msg
│       ├── DeviceSnapshot.msg
│       └── DeviceCommand.msg
├── docker/                           # Docker окружение
│   ├── Dockerfile
│   └── docker-compose.yml
├── cooling_controller.py             # Пример контроллера
└── README.md
```

### Внесение изменений

**Добавление нового датчика:**
1. Создать драйвер в `firmware/main/sensors/new_sensor.c`
2. Добавить заголовочный файл `new_sensor.h`
3. Зарегистрировать в `main.c` (инициализация и чтение)
4. Добавить новый `device_type` в сообщения
5. Обновить документацию

**Добавление новой команды:**
1. Добавить `command_code` в `DeviceCommand.msg`
2. Обработать в `command_callback()` в `main.c`
3. Реализовать логику выполнения
4. Обновить документацию и примеры

**Процесс разработки:**
```bash
# Форк репозитория
git clone https://github.com/your-username/robot_sensor_hub.git
cd robot_sensor_hub

# Создание ветки
git checkout -b feature/new-sensor

# Внесение изменений и тестирование
cd firmware && idf.py build flash monitor

# Коммит и push
git add .
git commit -m "Add new sensor driver"
git push origin feature/new-sensor

# Создание Pull Request на GitHub
```

---

*Последнее обновление: 18 октября 2025*  
*Совместимость: ESP-IDF v5.5.1, ROS2 Humble, micro-ROS (UART transport)*  
*Автор: krikz*