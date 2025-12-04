
# Ejercicio 2. Control Visual Turtlebot 

Este proyecto tiene como objetivo controlar un **TurtleBot3** mediante ROS2 y Micro-ROS, usando datos de percepción obtenidos desde una cámara o una grabación de Kinect.

## Requisitos
* ROS2 Humble
* ESP32 con Micro-ROS Agent
* TurtleBot3 (simulado en Gazebo)
* Video de Kinect (opcional, si no se usa cámara en vivo)
* Docker

---
Luego de clonar el repositorio eleminar de la carpera de src las carpetas de buil,install y log estas luego se generaran luego de compilar localmente
## 1. Configuración de Micro-ROS

Levantar el **agente de Micro-ROS** en Docker:

```bash
docker run -it --rm \
    --device=/dev/ttyUSB0 \
    microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -b 115200
```

> Nota: Asegurarse de que el código correspondiente esté subido a la ESP32. Este código se encuentra en el **Branch `ESP32_ejercicio2`**.

---

## 2. Verificación de nodos y topics

Comprobar que los nodos se están publicando:

```bash
ros2 node list
ros2 topic list
```

Verificar los mensajes enviados al topic `/cmd_vel`:

```bash
ros2 topic echo /cmd_vel
```

---

## 3. Configuración de TurtleBot3 en Gazebo

Correr el entorno de simulación:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

---

## 4. Configuración del nodo de percepción

En el archivo `perception_node.py`, elegir entre usar la cámara en vivo o una grabación:

```python
# Cambiar a True para usar la cámara en vivo, False para usar video
node = PosePerceptionNode(use_camera=False)
```

### 4.1. Ejecutar con cámara en vivo

```bash
colcon build --packages-select perception_lab
source install/setup.bash
ros2 run perception_lab perception_node
```

### 4.2. Ejecutar con grabación

1. Ejecutar el nodo:

```bash
colcon build --packages-select perception_lab
source install/setup.bash
ros2 run perception_lab perception_node
```

2. Reproducir el video en la carpeta donde se encuentra el archivo:

```bash
ros2 bag play kinect_data2
```
### Ejercicio Extra.
Requisitos

Tener configurado micro-ROS en ESP32.

Nodo de ROS2 depth_heatmap_node corriendo y publicando /proximity_zones.

TurtleBot3 configurado en Gazebo o entorno real.

Instrucciones

Subir código ESP32

Cambiar a la rama del ejercicio extra en tu repositorio ESP32.

Subir y flashear el código en la placa ESP32.

Ejecutar nodo de proximidad en ROS2

Compilar el paquete como en el ejercicio 2:

```bash
colcon build --packages-select perception_lab
source install/setup.bash
```

Ejecutar el nodo extra:
```bash
ros2 run perception_lab extra
```
