#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robot_sensor_hub_msg.msg import DeviceSnapshot, DeviceCommand
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import time
import numpy as np
from collections import deque
import threading
import sys
import traceback

# Устанавливаем бэкенд Matplotlib, совместимый с ROS 2
import matplotlib
matplotlib.use('Qt5Agg')  # Используем Qt5 вместо Tkinter

# Константы типов устройств (должны совпадать с прошивкой ESP32)
DEVICE_TYPE_AHT30 = 0
DEVICE_TYPE_FAN = 2

# Константы типов данных (должны совпадать с прошивкой ESP32)
DATA_TYPE_TEMPERATURE = 1
DATA_TYPE_HUMIDITY = 2
DATA_TYPE_WEIGHT = 3
DATA_TYPE_SPEED = 4
DATA_TYPE_RPM = 5

# Константы команд
COMMAND_SET_SPEED = 0

# Параметры системы
SETPOINT_DELTA = -2.0  # Целевая разница температур (выход - вход) = -2°C
SAMPLE_TIME = 1.0  # Интервал между измерениями (секунды)
MAX_HISTORY = 120  # Увеличено количество точек для отображения в графике
MIN_FAN_SPEED = 0.1  # Минимальная скорость вентилятора для предотвращения полной остановки

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint, sample_time=SAMPLE_TIME, min_output=MIN_FAN_SPEED, max_output=1.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.sample_time = sample_time
        self.min_output = min_output
        self.max_output = max_output
        
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
        # Ограничения для интегральной части (anti-windup)
        self.integral_min = min_output
        self.integral_max = max_output
    
    def update(self, measured_value):
        current_time = time.time()
        dt = current_time - self.last_time
        
        # Проверяем, прошел ли достаточный интервал времени
        if dt >= self.sample_time:
            # ПРАВИЛЬНОЕ ОПРЕДЕЛЕНИЕ ОШИБКИ
            error = measured_value - self.setpoint
            
            # Пропорциональная часть
            proportional = error
            
            # Интегральная часть
            self.integral += error * dt
            # Анти-обгон (anti-windup)
            self.integral = max(self.integral_min, min(self.integral, self.integral_max))
            
            # Дифференциальная часть
            derivative = (error - self.last_error) / dt if dt > 0 else 0.0
            
            # Вычисляем выходное значение
            output = self.Kp * proportional + self.Ki * self.integral + self.Kd * derivative
            
            # Ограничиваем выходное значение в диапазоне [min_output, max_output]
            output = max(self.min_output, min(self.max_output, output))
            
            # Сохраняем данные для следующего цикла
            self.last_error = error
            self.last_time = current_time
            
            return output
        else:
            return None

class CoolingSystemController(Node):
    def __init__(self):
        super().__init__('cooling_system_controller')
        
        # Инициализация данных
        self.inlet_temp = None  # Температура на входе (датчик 0)
        self.outlet_temp = None  # Температура на выходе (датчик 7)
        self.fan_speed = MIN_FAN_SPEED  # Текущая скорость вентилятора (начинаем с минимальной)
        self.fan_rpm = 0  # Текущая скорость вращения вентилятора в RPM (начинаем с 0)
        self.last_data_time = 0  # Время последнего получения данных
        
        # Инициализация ПИД-регулятора
        # Коэффициенты подобраны экспериментально для системы с интервалом 1 секунда
        self.pid = PIDController(
            Kp=0.15,  # Пропорциональный коэффициент
            Ki=0.02,  # Интегральный коэффициент
            Kd=0.05,  # Дифференциальный коэффициент
            setpoint=SETPOINT_DELTA,
            min_output=MIN_FAN_SPEED  # Установка минимальной скорости
        )
        
        # Публикатор для отправки команд
        self.command_publisher = self.create_publisher(
            DeviceCommand,
            '/device/command',
            10)
        
        # Подписка на топик снапшота данных
        self.subscription = self.create_subscription(
            DeviceSnapshot,
            '/device/snapshot',
            self.snapshot_callback,
            10)
        
        # Инициализация истории данных для графиков
        self.time_history = deque(maxlen=MAX_HISTORY)
        self.inlet_temp_history = deque(maxlen=MAX_HISTORY)
        self.outlet_temp_history = deque(maxlen=MAX_HISTORY)
        self.fan_speed_history = deque(maxlen=MAX_HISTORY)
        self.fan_rpm_history = deque(maxlen=MAX_HISTORY)  # Новая история для RPM
        self.error_history = deque(maxlen=MAX_HISTORY)
        self.temp_diff_history = deque(maxlen=MAX_HISTORY)  # Новая история для разницы температур
        
        self.get_logger().info('Cooling system controller started')
        self.get_logger().info(f'Target temperature difference (outlet - inlet): {SETPOINT_DELTA}°C')
        self.get_logger().info(f'Minimum fan speed: {MIN_FAN_SPEED}')
    
    def snapshot_callback(self, msg):
        """Обработка входящего снапшота данных"""
        current_time = time.time()
        self.last_data_time = current_time  # Обновляем время последнего получения данных
        
        for device_data in msg.devices:
            # Обработка датчиков AHT30
            if device_data.device_type == DEVICE_TYPE_AHT30 and device_data.data_type == DATA_TYPE_TEMPERATURE:
                if device_data.device_id == 0:  # Датчик на входе
                    self.inlet_temp = device_data.value
                    self.get_logger().debug(f'Inlet temperature (sensor 0): {self.inlet_temp:.2f}°C')
                elif device_data.device_id == 7:  # Датчик на выходе
                    self.outlet_temp = device_data.value
                    self.get_logger().debug(f'Outlet temperature (sensor 7): {self.outlet_temp:.2f}°C')
            
            # Обработка данных вентилятора
            elif device_data.device_type == DEVICE_TYPE_FAN:
                # Обрабатываем ТОЛЬКО вентилятор с ID 0 (основной)
                if device_data.device_id == 0:
                    # Скорость PWM
                    if device_data.data_type == DATA_TYPE_SPEED:
                        self.fan_speed = device_data.value
                        self.get_logger().debug(f'Fan speed: {self.fan_speed:.2f}')
                    # RPM от тахометра
                    elif device_data.data_type == DATA_TYPE_RPM:
                        self.fan_rpm = device_data.value
                        self.get_logger().debug(f'Fan RPM: {self.fan_rpm} RPM')
        
        # Обновление истории данных
        if self.inlet_temp is not None and self.outlet_temp is not None:
            # Добавляем данные в историю
            self.time_history.append(current_time)
            self.inlet_temp_history.append(self.inlet_temp)
            self.outlet_temp_history.append(self.outlet_temp)
            self.fan_speed_history.append(self.fan_speed)
            self.fan_rpm_history.append(self.fan_rpm)  # Добавляем RPM в историю
            
            # Вычисляем текущую разницу температур
            temp_diff = self.outlet_temp - self.inlet_temp
            self.temp_diff_history.append(temp_diff)
            
            # Вычисляем текущую ошибку
            current_error = temp_diff - SETPOINT_DELTA
            self.error_history.append(current_error)
            self.get_logger().debug(f'Current error: {current_error:.2f}°C, temp_diff={temp_diff:.2f}°C')
    
    def pid_update(self):
        """Обновление ПИД-регулятора"""
        if self.inlet_temp is None or self.outlet_temp is None:
            # Проверяем, давно ли не получали данные
            if time.time() - self.last_data_time > 5.0:
                self.get_logger().warn('No data received for more than 5 seconds')
            return
        
        # Вычисляем текущую разницу температур
        temp_diff = self.outlet_temp - self.inlet_temp
        
        # Обновляем ПИД-регулятор
        new_speed = self.pid.update(temp_diff)
        
        if new_speed is not None:
            # Отправляем команду на изменение скорости
            self.send_speed_command(0, new_speed)
            self.fan_speed = new_speed
            self.get_logger().info(f'PID update: temp_diff={temp_diff:.2f}°C, fan_speed={new_speed:.2f}')

    def send_speed_command(self, fan_id, speed):
        """Отправка команды изменения скорости вентилятора"""
        cmd_msg = DeviceCommand()
        cmd_msg.device_type = DEVICE_TYPE_FAN
        cmd_msg.device_id = fan_id
        cmd_msg.command_code = COMMAND_SET_SPEED
        cmd_msg.param_1 = speed
        cmd_msg.param_2 = 0.0
        cmd_msg.timestamp = int(time.time() * 1e9)
        
        self.command_publisher.publish(cmd_msg)
        self.get_logger().debug(f'Sent command: Fan {fan_id} speed set to {speed:.2f}')

def setup_plot():
    """Настройка графиков"""
    print("Setting up plot...")
    style.use('ggplot')
    
    try:
        fig = plt.figure(figsize=(12, 14))
        
        # График температур
        ax1 = fig.add_subplot(5, 1, 1)
        ax1.set_title('Temperature Monitoring')
        ax1.set_ylabel('Temperature (°C)')
        inlet_line, = ax1.plot([], [], 'b-', label='Inlet (sensor 0)')
        outlet_line, = ax1.plot([], [], 'r-', label='Outlet (sensor 7)')
        setpoint_line = ax1.axhline(y=SETPOINT_DELTA, color='g', linestyle='--', label=f'Setpoint (Δ={SETPOINT_DELTA}°C)')
        ax1.legend(loc='upper right')
        ax1.grid(True)
        
        # График разницы температур
        ax2 = fig.add_subplot(5, 1, 2)
        ax2.set_title('Temperature Difference')
        ax2.set_ylabel('Difference (°C)')
        temp_diff_line, = ax2.plot([], [], 'm-', label='Current difference (outlet - inlet)')
        setpoint_line2 = ax2.axhline(y=SETPOINT_DELTA, color='g', linestyle='--', label=f'Setpoint ({SETPOINT_DELTA}°C)')
        ax2.legend(loc='upper right')
        ax2.grid(True)
        
        # График ошибки
        ax3 = fig.add_subplot(5, 1, 3)
        ax3.set_title('PID Error')
        ax3.set_ylabel('Error')
        error_line, = ax3.plot([], [], 'c-', label='Error (current - setpoint)')
        zero_line = ax3.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        ax3.legend(loc='upper right')
        ax3.grid(True)
        
        # График скорости вентилятора (PWM)
        ax4 = fig.add_subplot(5, 1, 4)
        ax4.set_title('Fan Speed (PWM)')
        ax4.set_ylabel('Speed')
        fan_line, = ax4.plot([], [], 'y-', label='Fan 0 speed')
        min_speed_line = ax4.axhline(y=MIN_FAN_SPEED, color='r', linestyle='--', label=f'Min speed ({MIN_FAN_SPEED})')
        ax4.legend(loc='upper right')
        ax4.grid(True)
        
        # График RPM вентилятора
        ax5 = fig.add_subplot(5, 1, 5)
        ax5.set_title('Fan RPM')
        ax5.set_ylabel('RPM')
        ax5.set_xlabel('Time (s)')
        rpm_line, = ax5.plot([], [], 'g-', label='Fan 0 RPM')
        ax5.legend(loc='upper right')
        ax5.grid(True)
        
        plt.tight_layout()
        
        # Позиционируем окно
        try:
            # Для Qt5
            fig.canvas.manager.window.move(100, 100)
        except AttributeError:
            try:
                # Для Tkinter
                fig.canvas.manager.window.wm_geometry("+%d+%d" % (100, 100))
            except AttributeError:
                pass  # Не поддерживается для этого бэкенда
        
        print("Plot setup completed successfully")
        return fig, (ax1, ax2, ax3, ax4, ax5), (inlet_line, outlet_line, temp_diff_line, error_line, fan_line, rpm_line)
    except Exception as e:
        print(f"Error setting up plot: {str(e)}")
        traceback.print_exc()
        raise

def update_plot(frame, controller, lines, axes):
    """Обновление графиков для анимации"""
    inlet_line, outlet_line, temp_diff_line, error_line, fan_line, rpm_line = lines
    ax1, ax2, ax3, ax4, ax5 = axes
    
    # Обрабатываем новые сообщения ROS 2
    rclpy.spin_once(controller, timeout_sec=0.01)
    
    # Обновляем ПИД-регулятор
    controller.pid_update()
    
    # Если есть данные для отображения
    if len(controller.time_history) > 0:
        # Нормализуем временные метки
        times = np.array(controller.time_history) - controller.time_history[0]
        
        # Обновляем график температур
        inlet_line.set_data(times, controller.inlet_temp_history)
        outlet_line.set_data(times, controller.outlet_temp_history)
        
        # Обновляем график разницы температур
        temp_diff_line.set_data(times, controller.temp_diff_history)
        
        # Обновляем график ошибки
        error_line.set_data(times, controller.error_history)
        
        # Обновляем график скорости (PWM)
        fan_line.set_data(times, controller.fan_speed_history)
        
        # Обновляем график RPM
        rpm_line.set_data(times, controller.fan_rpm_history)
        
        # Настройка осей
        for ax in [ax1, ax2, ax3, ax4, ax5]:
            ax.relim()
            ax.autoscale_view()
    
    return lines

def main():
    print("Starting cooling controller...")
    
    # Проверка, доступен ли тип сообщения
    try:
        from robot_sensor_hub_msg.msg import DeviceSnapshot, DeviceCommand
        print("robot_sensor_hub_msg is available")
    except ImportError:
        print("Error: robot_sensor_hub_msg is not available. Make sure you have sourced your ROS 2 workspace.")
        print("Try: source ~/ros2_ws/install/setup.bash")
        return
    
    # Инициализация ROS 2
    try:
        rclpy.init()
        print("ROS 2 initialized successfully")
    except Exception as e:
        print(f"Error initializing ROS 2: {str(e)}")
        traceback.print_exc()
        return
    
    # Создание контроллера
    try:
        controller = CoolingSystemController()
        print("CoolingSystemController created successfully")
    except Exception as e:
        print(f"Error creating CoolingSystemController: {str(e)}")
        traceback.print_exc()
        rclpy.shutdown()
        return
    
    # Настройка графиков
    try:
        fig, axes, lines = setup_plot()
        print("Plot setup completed")
    except Exception as e:
        print(f"Error setting up plot: {str(e)}")
        traceback.print_exc()
        controller.destroy_node()
        rclpy.shutdown()
        return
    
    # Проверка подключения к ROS 2
    print("\n" + "="*50)
    print("Waiting for data from micro-ROS...")
    print("Make sure:")
    print("1. micro-ROS agent is running: micro-ros-agent udp4 --port 8888")
    print("2. ESP32 is connected and publishing data to /device/snapshot")
    print("3. ROS 2 workspace is sourced: source ~/ros2_ws/install/setup.bash")
    print("="*50 + "\n")
    
    # Создаем анимацию
    ani = animation.FuncAnimation(
        fig, 
        update_plot, 
        fargs=(controller, lines, axes),
        interval=500,  # Обновляем каждые 500 мс
        blit=False
    )
    
    print("\n=== Displaying plot window ===")
    print("If you don't see the plot window, check if it's minimized or behind other windows.")
    print("Close the plot window to exit the program.")
    
    # Запускаем анимацию и блокируем выполнение до закрытия окна
    plt.show()
    
    # Очистка (выполняется только после закрытия окна)
    print("\nCleaning up...")
    controller.destroy_node()
    rclpy.shutdown()
    print("Cleanup completed. Exiting.")

if __name__ == '__main__':
    main()