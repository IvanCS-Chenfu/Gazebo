# Explicación Total
Se ejecuta el nodo que hace aparecer al robot llamando al servicio ```/spawn_entity```. Este nodo carga el ```URDF``` del robot. Una vez cargado el ```URDF``` se ejecuta la linea de ```<plugin>``` que hay dentro de él. Esta linea referencia al plugin y carga la librería ```.so```. Al ejecutarse dicha linea se ejecuta el nodo del plugin, el cual asocia los valores recibidos por un tópcio con los ```joints``` del robot.

# Diferential Drive
El plugin que vamos a explicar sirve como puente para controlar un robot diferencial en Gazebo.
El plugin:
- Escucha comandos de velocidad (```/cmd_vel```) enviados desde ROS2
- Calcula la velocidad de cada rueda
- Aplica esas velocidades a las articulaciones  (```joint```) del modelo simulado.

## En el archivo [diff_drive_plugin.hpp](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/my_box_robot/include/my_box_robot/diff_drive_plugin.hpp)

### Librerías

```cpp
#ifndef DIFF_DRIVE_PLUGIN_HPP
#define DIFF_DRIVE_PLUGIN_HPP
```
Esto es un include guard, evita que el archivo se incluya dos veces por error durante la compilación.

```cpp
#include <gazebo/common/Plugin.hh>
```
Incluye clases base para plugins de Gazebo como ```ModelPlugin```.

```cpp
#include <gazebo/physics/physics.hh>
```
Permite acceder a entidades físicas de Gazebo: modelos, joints, etc.

```cpp
#include <sdf/sdf.hh>
```
Accede a parámetros definidos en el URDF/SDF que acompaña al modelo.

```cpp
#include <gazebo_ros/node.hpp>
```
Permite crear nodos ROS 2 dentro de Gazebo.

```cpp
#include <rclcpp/rclcpp.hpp>
```
API de ROS 2 C++ (rclcpp) para usar nodos, logs, suscripciones...

```cpp
#include <geometry_msgs/msg/twist.hpp>
```
Mensaje estándar para velocidades lineales y angulares (```cmd_vel```).

### Clase
```cpp
class DiffDrivePlugin : public ModelPlugin
```
Tu clase hereda de ```ModelPlugin```. Esto hace que Gazebo la cargue automáticamente como plugin cuando encuentra esto en el URDF.

```cpp
DiffDrivePlugin();
```
Constructor por defecto. No hace nada en este caso, pero se define por buena práctica

```cpp
void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;
```
Se ejecuta automáticamente al cargar el plugin. ```model``` es el robot; ```sdf``` contiene los parámetros del plugin.

```cpp
void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);
```
Función callback que recibe los mensajes ```/cmd_vel```.

```cpp
gazebo::physics::ModelPtr model_;
```
Puntero al modelo en el que se ha insertado el plugin.

```cpp
gazebo::physics::JointPtr left_wheel_joint_;
gazebo::physics::JointPtr right_wheel_joint_;
```
Punteros a las articulaciones de las ruedas.

```cpp
gazebo_ros::Node::SharedPtr ros_node_;
```
Nodo ROS dentro del plugin.

```cpp
rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
```
Suscripción a ```/cmd_vel```.

```cpp
double wheel_radius_;
double wheel_separation_;
```
Parámetros leídos del URDF para calcular cinemática diferencial.

## En el archivo [diff_drive_plugin.cpp](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/my_box_robot/src/diff_drive_plugin.cpp)
### Librerías
```cpp
#include "my_box_robot/diff_drive_plugin.hpp"
```
Incluye el header propio del plugin.

```cpp
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
```
Incluye lo necesario para manipular joints, crear nodo ROS y suscribirse a ```/cmd_vel```.

### Carga de Modelo
```cpp
void DiffDrivePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
```
Se llama automáticamente cuando Gazebo carga el modelo que tiene este plugin.

```cpp
model_ = model;
```
Nos guardamos el puntero al modelo para usarlo más adelante.

```cpp
auto nh = gazebo_ros::Node::Get(sdf);
ros_node_ = nh;
```
Esto crea el nodo ROS 2 dentro del entorno de Gazebo, con namespace si es necesario.

```cpp
wheel_radius_ = sdf->Get<double>("wheel_radius", 0.05).first;
wheel_separation_ = sdf->Get<double>("wheel_separation", 0.26).first;
```
```sdf->Get<T>("nombre", valor_por_defecto).first``` lee valores del plugin definidos en el URDF. 

Si no se definen, se usan 0.05 y 0.26 como fallback

```cpp
left_wheel_joint_ = model_->GetJoint("left_wheel_joint");
right_wheel_joint_ = model_->GetJoint("right_wheel_joint");
```
Esto busca en el modelo las articulaciones con esos nombres exactos. Si no las encuentra, no se podrá mover.

```cpp
if (!left_wheel_joint_ || !right_wheel_joint_)
```
Si alguno no existe, lo reportamos y cancelamos el plugin.

```cpp
cmd_vel_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&DiffDrivePlugin::OnCmdVel, this, std::placeholders::_1)
);
```
Creamos una suscripción al topic ```cmd_vel```

```10``` es la cola de mensajes (buffer)

```std::bind(...)``` conecta el callback ```OnCmdVel```

### Callback Subscriptor
```cpp
void DiffDrivePlugin::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
```
Este callback se ejecuta cada vez que llega un mensaje al topic ```/cmd_vel```.

```cpp
double v = msg->linear.x;
double w = msg->angular.z;
```
```v```: velocidad lineal hacia adelante

```w```: velocidad angular en Z (giro sobre su eje)

```cpp
double v_left = (v - w * wheel_separation_ / 2.0) / wheel_radius_;
double v_right = (v + w * wheel_separation_ / 2.0) / wheel_radius_;
```
Fórmulas estándar de cinemática diferencial

Transforma ```v``` y ```w``` en velocidades angulares de cada rueda

```cpp
left_wheel_joint_->SetParam("fmax", 0, 10.0);
left_wheel_joint_->SetParam("vel", 0, v_left);
```
```fmax```: esfuerzo máximo permitido para girar

```vel```: velocidad angular deseada

### Registrar Plugin
```cpp
GZ_REGISTER_MODEL_PLUGIN(DiffDrivePlugin)
```
Esta macro registra tu plugin en Gazebo para que pueda cargarlo desde un archivo URDF/SDF automáticamente.

## En el archivo [CMAkeLists.txt](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/my_box_robot/CMakeLists.txt)
```cmake
add_library(diff_drive_plugin SHARED src/diff_drive_plugin.cpp)
```
Compila tu plugin como librería compartida ```.so```.

```cmake
ament_target_dependencies(diff_drive_plugin
  rclcpp
  gazebo_ros
  geometry_msgs
)
```
Incluye las dependencias necesarias.

```cmake
install(TARGETS diff_drive_plugin DESTINATION lib)
```
Instala el plugin en ```install/lib/```, donde Gazebo lo buscará.

## En el archivo [package.xml](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/my_box_robot/package.xml)

```xml
<depend>rclcpp</depend>
<depend>gazebo_ros</depend>
<depend>geometry_msgs</depend>
```
Declara que tu paquete depende de esas bibliotecas.