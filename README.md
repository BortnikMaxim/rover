# ROS2 Rover Simulation

Симуляция колёсного ровера в Gazebo на базе ROS2 Jazzy.

## Требования

- ROS2 Jazzy
- Gazebo
- colcon
- teleop_twist_keyboard

## Сборка

```bash
cd ~/ros2_ws
colcon build --packages-select kolestel_rover_description
```

## Подключение окружения после сборки

```bash
source install/setup.bash
```

## Подключение системного окружения ROS2

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

После каждого перезапуска терминала нужно выполнять эти команды заново.

## Запуск симуляции

```bash
ros2 launch kolestel_rover_description gazebo.launch.py
```

## Управление с клавиатуры

Установка пакета:

```bash
sudo apt install ros-jazzy-teleop-twist-keyboard -y
```

Запуск:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Команды движения публикуются в топик `/cmd_vel`.
