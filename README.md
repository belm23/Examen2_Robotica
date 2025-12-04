# Ejercicio1_Robotica

Este repositorio contiene la implementación del algoritmo Deep Q-Network (DQN) en ROS 2 (Humble) para entrenar un robot TurtleBot3 (modelo Burger) a navegar y evadir obstáculos utilizando únicamente datos del sensor LiDAR 2D.

El modelo entrenado se incluye para su evaluación directa, cumpliendo el requisito de una Tasa de Éxito mínima del 30 - 40%.

Enlace para acceder a Google Drive: https://drive.google.com/drive/folders/17pf6HzhlNq3hmx_akqkE1gx96_V3cqTk 

# Instalar los paquetes Python requeridos para el agente DQN:

```bash
pip3 install scikit-learn numpy matplotlib
```

# Crear workspace

```bash
mkdir -p ~/dqn_navigation_ws/src

cd ~/dqn_navigation_ws/src
```

# Clona el repositorio en 'src'

```bash
git clone https://github.com/belm23/Examen2_Robotica.git

cd Examen2_Robotica
git checkout ejercicio1

mv dqn_robot_nav ..

cd ..
rm -rf Examen2_Robotica

ls
```

# Configurar robot

```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc

source ~/.bashrc
```
# Colocar los resultados en el workspace

```bash
ls ~/dqn_navigation_ws/src/dqn_robot_nav/

mv ~/dqn_navigation_ws/src/dqn_robot_nav/results_20251204_162200 ~/dqn_navigation_ws/
```
# Terminal 1

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
# Terminal 2

```bash
cd ~/dqn_navigation_ws

colcon build --packages-select dqn_robot_nav

source install/setup.bash

ros2 run dqn_robot_nav test_node src/dqn_robot_nav/results_20251204_162200/model_final.pkl
```






