# Архив / Archive

Эта папка содержала файлы старых версий проекта.

## Удалённые файлы

### v1.0 (ESP-IDF + micro-ROS)
- `firmware/` - ESP-IDF прошивка с micro-ROS
- `docker/` - Docker окружение для сборки
- `robot_sensor_hub_msg/` - ROS2 message definitions
- `cooling_controller.py` - ROS2 контроллер
- `run_docker.sh` - Docker helper

### v2.0 (PlatformIO - старый протокол)
- `read_sensors.py` - Continuous push reader
- `send_command.py` - Old command sender

## Текущая версия: v2.1

- **Request-response протокол** вместо continuous push
- **sensor_client.py** для Raspberry Pi + ROS
- Нет зависимостей от ROS/Docker

## История

| Версия | Особенности |
|--------|-------------|
| v1.0 | ESP-IDF + micro-ROS + Docker |
| v2.0 | PlatformIO + continuous push |
| v2.1 | PlatformIO + request-response |

---

Для использования старой версии: `git checkout v1.0`
