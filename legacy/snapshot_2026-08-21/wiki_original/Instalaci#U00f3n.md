# Instalación Gazebo
# Plugin Gazebo-ROS2
## Instalación Estándar
Si ya tienes Gazebo instalado y ROS2 Humble o similar (utilizando ```sudo apt install ros-humble-desktop```) tienes incluido automáticamente instalado:
- ```gazebo_ros``` la interfaz de ROS 2 con Gazebo (nodo puente, plugins base, etc.)

- ```gazebo_dev``` cabeceras y CMake config para compilar plugins personalizados

- ```plugins``` como ```gazebo_ros_diff_drive```, ```camera```, ```imu```, etc.

## Instalación Mínima

Si en la istalación de ROS utilizate algo similar a ```ros-humble-ros-base``` necesitarás instalar a mano
```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
```
Esto incluye:
- ```libgazebo_ros_factory.so``` (para ```/spawn_entity```)

- El plugin ```gazebo_ros_init.so```

- El nodo ```gazebo_ros_node```

- Macros para ```xacro``` y soporte SDF

## CMakeList y Package
Se deben incluir estas dependencias
En ```CMakeList.txt```
```cmake
find_package(gazebo_ros REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
```

En ```package.xml```
```xml
<depend>gazebo_ros</depend>
```

## Comprobación
Puedes verificar si el plugin ```gazebo_ros``` está disponible con:
```bash
ros2 pkg list | grep gazebo
```
Y comprobar que tienes archivos como:
```bash
/opt/ros/humble/lib/libgazebo_ros_factory.so
/opt/ros/humble/share/gazebo_ros
```
