# robot_sensor_hub/SystemMeta.msg
uint32 sampling_rate_hz    # частота опроса (Гц)
uint32 active_devices      # количество активных устройств
uint32 total_readings      # общее количество показаний
uint8 system_status        # статус системы (0=normal, 1=warning, 2=error)