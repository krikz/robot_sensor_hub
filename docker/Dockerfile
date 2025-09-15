FROM espressif/idf:release-v5.2

# Устанавливаем необходимые зависимости
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN . $IDF_PATH/export.sh && pip3 install catkin_pkg lark-parser colcon-common-extensions
RUN pip3 install --upgrade empy
WORKDIR /project


# Создаем папку components и клонируем micro-ROS компоненты
RUN mkdir -p components && \
    git clone -b humble https://github.com/micro-ROS/micro_ros_espidf_component.git components/micro_ros_espidf_component

# Копируем пользовательские сообщения
COPY robot_sensor_hub_msg/ components/micro_ros_espidf_component/extra_packages/robot_sensor_hub_msg/

# Копируем исходники проекта
COPY firmware/ /project/

# Собираем проект
RUN  . $IDF_PATH/export.sh && idf.py build

# Устанавливаем переменные окружения
ENV IDF_PATH=/opt/esp/idf

ENTRYPOINT ["/bin/bash"]