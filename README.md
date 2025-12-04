# Ejercicio1_Robotica

Este repositorio contiene la implementación del algoritmo Deep Q-Network (DQN) en ROS 2 (Humble) para entrenar un robot TurtleBot3 (modelo Burger) a navegar y evadir obstáculos utilizando únicamente datos del sensor LiDAR 2D.

El modelo entrenado se incluye para su evaluación directa, cumpliendo el requisito de una Tasa de Éxito mínima del 30 - 40%.

# Instalar los paquetes Python requeridos para el agente DQN:

pip3 install scikit-learn numpy matplotlib

# Crear workspace
mkdir -p ~/dqn_navigation_ws/src
cd ~/dqn_navigation_ws/src

# Clona el repositorio en 'src'
git clone https://github.com/belm23/Examen2_Robotica.git
mv Examen2_Robotica/dqn_robot_nav .
cd ~/dqn_navigation_ws

# Configurar robot
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc

# Colocar los resultados en el workspace
ls ~/dqn_navigation_ws/src/dqn_robot_nav/
mv ~/dqn_navigation_ws/src/dqn_robot_nav/results_20251204_162200 ~/dqn_navigation_ws/

# Terminal 1
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2
cd ~/dqn_navigation_ws
colcon build --packages-select dqn_robot_nav
source install/setup.bash
ros2 run dqn_robot_nav test_node src/dqn_robot_nav/results_20251204_162200/model_final.pkl








