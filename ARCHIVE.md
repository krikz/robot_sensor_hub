# Архивные файлы / Archived Files

Эта директория содержит файлы из старой версии проекта (v1.0), которая использовала ESP-IDF и micro-ROS.

This directory contains files from the old version of the project (v1.0) that used ESP-IDF and micro-ROS.

## Устаревшие компоненты / Deprecated Components

- `firmware/` - Старая прошивка на ESP-IDF с micro-ROS
- `robot_sensor_hub_msg/` - ROS2 сообщения для micro-ROS
- `docker/` - Docker окружение для сборки ESP-IDF
- `cooling_controller.py` - ROS2 контроллер (требует micro-ROS)
- `run_docker.sh` - Скрипт запуска Docker
- `tools/` - Утилиты для старой версии

## Новая версия / New Version

Текущая версия проекта (v2.0) использует:
- **PlatformIO** вместо ESP-IDF
- **Arduino Framework** вместо ESP-IDF FreeRTOS
- **Serial UART** вместо micro-ROS
- **JSON/CSV протокол** вместо ROS2 топиков

The current version (v2.0) uses:
- **PlatformIO** instead of ESP-IDF
- **Arduino Framework** instead of ESP-IDF FreeRTOS
- **Serial UART** instead of micro-ROS
- **JSON/CSV protocol** instead of ROS2 topics

## Миграция / Migration

Если вы хотите использовать старую версию с micro-ROS, переключитесь на тег v1.0:

If you want to use the old version with micro-ROS, switch to tag v1.0:

```bash
git checkout v1.0
```

Или обратитесь к старому README:

Or refer to the old README:

```bash
cat README_OLD.md
```
