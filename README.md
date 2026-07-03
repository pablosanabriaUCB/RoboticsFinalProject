# 🤖👁️ Eye Tracker Arm - Brazo Robótico con Seguimiento de Ojos

Control de un brazo robótico de **4 grados de libertad** que sigue la posición del rostro/ojos humanos usando **visión por computadora** (MediaPipe), **control visual directo en el espacio articular**, y **micro-ROS** (UART/Serial) para comunicación con el ESP32.

## Arquitectura del Sistema

```text
┌─────────────────┐      PointStamped       ┌──────────────────┐    Float32MultiArray    ┌─────────────┐
│  Nodo Visión    │─────────────────────────►│  Nodo IK         │──────────────────────►  │  micro-ROS  │
│  (MediaPipe)    │  /eye_tracker/           │  (Visual         │  /arm/joint_commands    │   Agent     │
│                 │   target_position        │   Servoing)      │                         │  (Docker)   │
└─────────────────┘                          └──────────────────┘                         └──────┬──────┘
       ▲                                              │                                          │
       │                                              │ JointState                        Cable USB
    Webcam                                            │ /arm/joint_states                 (UART)
                                                      ▼                                          │
                                                ┌──────────┐                             ┌──────▼──────┐
                                                │  RViz2   │                             │   ESP32     │
                                                │  (URDF)  │                             │  4 Servos   │
                                                └──────────┘                             │  PWM        │
                                                                                         └─────────────┘
```

## Estructura del Repositorio

El proyecto cumple con las normativas estándar de repositorios estructurados para la investigación y la robótica:

```text
eye_tracker_arm_ws/
├── src/                             # Código fuente ROS 2
│   ├── eye_tracker_vision/          # Paquete ROS2 - Detección visual (MediaPipe)
│   ├── eye_tracker_ik/              # Paquete ROS2 - Servoing Visual
│   └── eye_tracker_bringup/         # Paquete ROS2 - Configuración y launch
├── firmware/                        # Código fuente del microcontrolador
│   └── eye_tracker_servo_controller/ # Firmware ESP32 (Arduino + micro-ROS)
├── data/                            # Datos y logs de calibración y pruebas
├── results/                         # Gráficas, mediciones de latencia y desempeño
├── docs/                            # Documentación adicional y diagramas
├── LICENSE                          # Licencia MIT del proyecto
└── README.md                        # Instrucciones y directrices (este archivo)
```

## Hardware

| Componente | Descripción |
|---|---|
| Computador principal | Jetson Orin Nano (Ubuntu 22.04 + ROS 2 Humble) |
| Microcontrolador | ESP32 DevKit v1 |
| Servos | 4x SG90/MG996R (PWM estándar) |
| Cámara | Webcam USB |
| Comunicación | Serial / UART vía USB (micro-ROS) a 115200 baudios |

### Pinout ESP32 - Servos

| Servo | Articulación | Pin GPIO |
|---|---|---|
| Servo 1 | Base (Joint 1 / Pan) | GPIO 27 |
| Servo 2 | Hombro (Joint 2 / Tilt) | GPIO 14 |
| Servo 3 | Codo (Joint 3) | GPIO 12 |
| Servo 4 | Muñeca (Joint 4) | GPIO 13 |

## Requisitos e Instalación

### Jetson Orin Nano (Ubuntu 22.04 + ROS 2 Humble)

1. Cargar el entorno de ROS 2:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
2. Instalar dependencias de Python (preferiblemente en un entorno virtual como `env_vision`):
   ```bash
   pip3 install mediapipe opencv-python numpy
   ```
3. Descargar la imagen de micro-ROS para Docker:
   ```bash
   sudo docker pull microros/micro-ros-agent:humble
   ```

### Arduino IDE (para el ESP32)

1. Instalar Arduino IDE v2.x
2. Agregar la tarjeta ESP32 a la lista de tarjetas en las Preferencias.
3. Instalar la librería **micro_ros_arduino** compatible con Humble.
4. Instalar la librería **ESP32Servo**.

## Compilación del Workspace ROS 2

Para compilar el código fuente en la computadora:

```bash
cd ~/DEV/ROBOTICA/eye_tracker_arm_ws_uart

# Compilar
colcon build --symlink-install

# Activar el workspace
source install/setup.bash
```

## Configuración del Firmware ESP32

El firmware utiliza la conexión Serial (UART) por defecto. No requiere configuración de WiFi. 
Compilar y flashear usando el Arduino IDE asegurándose de seleccionar:
- Board: `ESP32 Dev Module`

## Ejecución del Proyecto

### 1. Iniciar el Agente micro-ROS
Con el ESP32 conectado por USB (normalmente en `/dev/ttyUSB0`), ejecuta el agente de micro-ROS para establecer la comunicación serial:

```bash
sudo docker run -it --rm -v /dev:/dev --privileged microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -v6
```
*(Si no se conecta de inmediato, presiona el botón RST o EN de tu ESP32).*

### 2. Lanzar los Nodos de ROS 2
En otra terminal, activa tu entorno y lanza el sistema de visión y control:

```bash
cd ~/DEV/ROBOTICA/eye_tracker_arm_ws_uart
source /opt/ros/humble/setup.bash
source install/setup.bash
source /home/pablo/DEV/env_vision/bin/activate

ros2 launch eye_tracker_bringup eye_tracker_arm.launch.py
```

## Reproducción de Resultados (Experimentos)

Si deseas replicar las mediciones de latencia o probar el control de redundancia (estudiado en los resultados de nuestra investigación):

1. **Latencia del Lazo**: La tasa de refresco principal (100Hz) está forzada en el nodo de visión para no permitir el almacenamiento en búfer de OpenCV, y un envío de control de 15Hz hacia el ESP32. Estos parámetros se hallan en `src/eye_tracker_vision/eye_tracker_vision/vision_node.py` y `src/eye_tracker_ik/eye_tracker_ik/ik_node.py`.
2. **Suavizado Exponencial del Firmware**: En el ESP32 (`eye_tracker_servo_controller.ino`), el parámetro `INTERP_SPEED = 0.15` es vital para reducir la sobreoscilación en el espacio físico y compensar el retraso natural de la red serial.
3. **Log de Datos**: Puedes grabar los experimentos y trayectorias del efector final registrando los `JointState` en una bag de ROS 2:
   ```bash
   ros2 bag record /arm/joint_states /eye_tracker/target_position
   ```
   Luego, los datos pueden ser exportados a la carpeta `/results` para la generación de gráficas usando herramientas como `rqt_plot` o scripts de Python.
