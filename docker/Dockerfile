# Используем официальный образ ROS 2 Humble
FROM ros:humble

# Устанавливаем необходимые зависимости для ESP-IDF
RUN apt-get update && apt-get install -y \
    git \
    wget \
    flex \
    bison \
    gperf \
    python3 \
    python3-pip \
    python3-venv \
    cmake \
    ninja-build \
    ccache \
    libffi-dev \
    libssl-dev \
    dfu-util \
    libusb-1.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Устанавливаем дополнительные зависимости для micro-ROS
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Устанавливаем ESP-IDF v5.2
WORKDIR /opt
RUN git clone -b v5.2 --recursive https://github.com/espressif/esp-idf.git
WORKDIR /opt/esp-idf
RUN ./install.sh all

# Добавляем ESP-IDF в PATH
ENV IDF_PATH=/opt/esp-idf
ENV PATH="${IDF_PATH}/tools:${PATH}"

# Создаем рабочее пространство для micro-ROS
WORKDIR /uros_ws
#RUN source /opt/ros/$ROS_DISTRO/setup.bash
RUN git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
RUN sudo apt update && rosdep update
RUN rosdep install --from-paths src --ignore-src -y
RUN sudo apt-get install python3-pip
RUN colcon build
RUN source install/local_setup.sh

# Клонируем компонент micro-ROS для ESP-IDF
RUN mkdir -p /project/components
RUN git clone -b humble https://github.com/micro-ROS/micro_ros_espidf_component.git /project/components/micro_ros_espidf_component

# Копируем пользовательские сообщения
COPY robot_sensor_hub_msg/ /project/components/micro_ros_espidf_component/extra_packages/robot_sensor_hub_msg/

# Копируем исходники проекта
COPY firmware/ /project/

# Устанавливаем переменные окружения
ENV IDF_PATH=/opt/esp-idf
ENV PATH="${IDF_PATH}/tools:${PATH}"

# Собираем проект
WORKDIR /project
RUN source /opt/ros/humble/setup.sh && \
    source /uros_ws/install/local_setup.sh && \
    idf.py build

ENTRYPOINT ["/bin/bash"]