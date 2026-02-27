🚀 ROS2 Rover Simulation

Этот проект содержит симуляцию ровера в Gazebo на базе ROS2.

⸻

📦 Требования

Перед запуском убедитесь, что установлены:
	•	ROS2 Jazzy
	•	Gazebo
	•	colcon
	•	teleop_twist_keyboard

⸻

🛠 Сборка проекта

cd ~/ros2_ws
colcon build --packages-select kolestel_rover_description

Что делает эта команда
	•	переходит в рабочее пространство ROS2 (ros2_ws)
	•	собирает пакет kolestel_rover_description
	•	создаёт папки build, install, log

⸻

🔧 Подключение окружения после сборки

source install/setup.bash

Что это делает
	•	подключает собранные пакеты
	•	позволяет ROS2 видеть launch-файлы и ноды

⸻

▶️ Запуск симуляции Gazebo

ros2 launch kolestel_rover_description gazebo.launch.py

Что происходит
	•	запускается Gazebo
	•	загружается модель ровера
	•	стартует симуляция

⸻

🌍 Подключение системного окружения ROS2

source /opt/ros/jazzy/setup.bash

Зачем это нужно
	•	подключает глобальную установку ROS2 Jazzy
	•	делает доступными стандартные пакеты ROS2

⸻

🔁 Подключение workspace

source ~/ros2_ws/install/setup.bash

Что делает
	•	добавляет локальные пакеты workspace в окружение
	•	необходимо перед запуском нод

⸻

🎮 Установка управления с клавиатуры

sudo apt install ros-jazzy-teleop-twist-keyboard -y

Что делает
	•	устанавливает пакет управления роботом с клавиатуры

⸻

🕹 Управление ровером

ros2 run teleop_twist_keyboard teleop_twist_keyboard

Что происходит
	•	запускается управление с клавиатуры
	•	публикуются команды движения (cmd_vel)

⸻

📌 Если после перезапуска терминала ничего не работает

Выполните снова:

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

:::
