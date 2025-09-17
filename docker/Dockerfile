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

# Инициализируем rosdep
RUN  rosdep update

# Добавляем ESP-IDF в PATH
ENV IDF_PATH=/opt/esp-idf
ENV PATH="${IDF_PATH}/tools:${PATH}"

# Устанавливаем ESP-IDF v5.2
WORKDIR /opt
RUN git clone -b v5.2 --recursive https://github.com/espressif/esp-idf.git
WORKDIR /opt/esp-idf
RUN ./install.sh all

# Создаем рабочее пространство для micro-ROS
#WORKDIR /uros_ws
#RUN git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Обновляем списки пакетов и устанавливаем libcurl4-openssl-dev
#RUN apt-get update && apt-get install -y libcurl4-openssl-dev

# Устанавливаем зависимости через rosdep
##RUN . /opt/ros/humble/setup.sh && \
#    rosdep install --from-paths src --ignore-src -y

# Собираем micro_ros_setup с активированным ROS окружением
#RUN . /opt/ros/humble/setup.sh && \
#    colcon build

# Создаем символьную ссылку python -> python3
#RUN apt-get update && \
#    apt-get install -y python3 python3-pip && \
#    ln -s /usr/bin/python3 /usr/bin/python
RUN . $IDF_PATH/export.sh && pip3 install catkin_pkg lark-parser colcon-common-extensions
# Обновляем pip и устанавливаем актуальные версии Python-пакетов
#RUN pip3 install --upgrade pip

# Клонируем компонент micro-ROS для ESP-IDF
RUN mkdir -p /project/components
RUN git clone https://github.com/micro-ROS/micro_ros_espidf_component.git /project/components/micro_ros_espidf_component
COPY robot_sensor_hub_msg/ /project/components/micro_ros_espidf_component/extra_packages/robot_sensor_hub_msg/

# Копируем проект
COPY firmware/ /project/

WORKDIR /project/components
RUN . $IDF_PATH/export.sh && idf.py build
WORKDIR /project

# Собираем проект
RUN . $IDF_PATH/export.sh && idf.py build

ENTRYPOINT ["/bin/bash"]