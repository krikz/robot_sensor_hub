# Используем официальный образ ESP-IDF
FROM espressif/idf:release-v5.1

# Устанавливаем необходимые зависимости для micro-ROS
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*

# Устанавливаем Python пакеты для micro-ROS
RUN pip3 install catkin_pkg lark-parser colcon-common-extensions

# Создаем структуру проекта
WORKDIR /project

# Создаем новый проект robot_sensor_hub
RUN . /opt/esp/idf/export.sh && \
    idf.py create-project robot_sensor_hub && \
    mv robot_sensor_hub/* . && \
    rm -rf robot_sensor_hub

# Устанавливаем target ESP32
RUN . /opt/esp/idf/export.sh && idf.py set-target esp32

# Создаем папку components и клонируем micro-ROS компоненты
RUN mkdir -p components && \
    git clone -b humble https://github.com/micro-ROS/micro_ros_espidf_component.git components/micro_ros_espidf_component

# Копируем пользовательские сообщения в extra_packages
COPY robot_sensor_hub_msg/ components/micro_ros_espidf_component/extra_packages/robot_sensor_hub_msg

# Собираем проект с компонентами (без исходников)
RUN . /opt/esp/idf/export.sh && idf.py build || echo "Build completed with components"

# Копируем наши исходники
COPY firmware/main/ /project/main/
COPY firmware/CMakeLists.txt /project/
COPY firmware/sdkconfig /project/

# Копируем скрипт для запуска
COPY docker/scripts/entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/entrypoint.sh

# Добавляем пользователя в группу dialout для доступа к USB (Linux)
RUN if [ -x "$(command -v usermod)" ]; then \
        usermod -a -G dialout root; \
    fi

# Устанавливаем переменные окружения
ENV IDF_PATH=/opt/esp/idf

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["/bin/bash"]