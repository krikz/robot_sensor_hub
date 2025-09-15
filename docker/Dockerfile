FROM espressif/idf:release-v5.1

# Устанавливаем необходимые зависимости
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install catkin_pkg lark-parser colcon-common-extensions
# Устанавливаем catkin_pkg в виртуальное окружение ESP-IDF
RUN /opt/esp/python_env/idf5.1_py3.8_env/bin/python -m pip install catkin_pkg

WORKDIR /project

# Создаем проект ESP-IDF
COPY firmware/ /project/

# Создаем папку components и клонируем micro-ROS компоненты
RUN mkdir -p components && \
    git clone -b humble https://github.com/micro-ROS/micro_ros_espidf_component.git components/micro_ros_espidf_component

# Копируем пользовательские сообщения
COPY robot_sensor_hub_msg/ components/micro_ros_espidf_component/extra_packages/robot_sensor_hub_msg/

RUN . /opt/esp/idf/export.sh && idf.py build

# Устанавливаем переменные окружения
ENV IDF_PATH=/opt/esp/idf

ENTRYPOINT ["/bin/bash"]