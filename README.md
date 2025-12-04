
# Ejercicio 2. Control Visual Turtlebot 

Este proyecto tiene como objetivo controlar un **TurtleBot3** mediante ROS2 y Micro-ROS, usando datos de percepción obtenidos desde una cámara o una grabación de Kinect.

## Requisitos
* ROS2 Humble
* ESP32 con Micro-ROS Agent
* TurtleBot3 (simulado en Gazebo)
* Video de Kinect (opcional, si no se usa cámara en vivo)

---

## 1. Configuración de Micro-ROS

Levantar el **agente de Micro-ROS** en Docker:

```bash
docker run -it --rm \
    --device=/dev/ttyACM0 \
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
ros2 run perception_lab perception_node
```

### 4.2. Ejecutar con grabación

1. Ejecutar el nodo:

```bash
ros2 run perception_lab perception_node
```

2. Reproducir el video en la carpeta donde se encuentra el archivo:

```bash
ros2 bag play kinect_data2
```
