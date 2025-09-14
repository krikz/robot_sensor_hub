FROM espressif/idf:release-v5.1

# Установка дополнительных зависимостей для MicroROS
RUN apt-get update && \
    apt-get install -y \
    python3-pip \
    python3-venv \
    git \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Установка MicroROS dependencies
RUN pip3 install catkin_pkg lark empy

# Клонирование и настройка MicroROS компонента для ESP-IDF
# Используем версию, совместимую с ROS2 Humble
ARG MICROROS_VERSION=humble
RUN git clone https://github.com/micro-ROS/micro_ros_espidf_component.git /opt/micro_ros_espidf \
    && cd /opt/micro_ros_espidf \
    && git checkout ${MICROROS_VERSION}

# Создаем рабочую директорию
WORKDIR /project

# Копируем скрипты для удобства
COPY ./docker/scripts/entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/entrypoint.sh

# Для Linux: добавляем пользователя в группу dialout для доступа к USB
RUN if [ -x "$(command -v usermod)" ]; then \
        usermod -a -G dialout root; \
    fi

# Устанавливаем переменные окружения
ENV IDF_PATH=/opt/esp/idf
ENV MICROROS_PATH=/opt/micro_ros_espidf

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["/bin/bash"]