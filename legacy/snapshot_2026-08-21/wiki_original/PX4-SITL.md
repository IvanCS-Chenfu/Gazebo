Utilizar [PX4 SITL](https://docs.px4.io/main/en/simulation/) (Software-In-The-Loop) es una excelente elección para simular drones de manera realista.

# Instalación

## Instalar PX4 Autopilot
Clona el repositorio de PX4 y ejecuta el script de instalación:
```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```
Este script instalará todas las dependencias necesarias, incluyendo Gazebo.

## Instalar ROS2 Humble


## Configurar XRCE-DDS Agent
PX4 utiliza XRCE-DDS para comunicarse con ROS 2. Instala el agente de la siguiente manera:
```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

## Crear y Compilar el espacio de trabajo ROS2
Crea un espacio de trabajo para ROS 2 y clona los paquetes necesarios:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git
cd ..
source /opt/ros/humble/setup.bash
colcon build
```

## Ejecutar PX4 SITL con Gazebo
Desde el directorio de PX4-Autopilot, ejecuta:
```bash
make px4_sitl gazebo
```
Esto iniciará la simulación con el modelo predeterminado (por ejemplo, Iris) en Gazebo.

## Ejecutar el agente XRCE-DDS
En otra terminal, inicia el agente XRCE-DDS:
```bash
MicroXRCEAgent udp4 -p 8888
```
## Verificar la comunicación entre PX4 y ROS2
Después de compilar el espacio de trabajo de ROS2, verifica que los tópicos estén disponibles:
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic list
```
Deberías ver una lista de tópicos relacionados con PX4.