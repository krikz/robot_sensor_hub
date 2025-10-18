idf_component_register(SRCS "main.c"
                   "sensors/aht30_reader.c"
                   "sensors/hx711_reader.c"
                   "sensors/fan_controller.c"
                   "esp32_serial_transport.c"
                    INCLUDE_DIRS "."
                    REQUIRES micro_ros_espidf_component)