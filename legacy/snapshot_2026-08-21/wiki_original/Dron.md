En esta página se explicará al completo como se ha programado un dron con inteligencia artificial

# HardWare y Plugin

## Parámetros

Todos los parámetros del diseño del dron se encontrarán en el archivo [dron.yaml](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/my_quadracopter/config/dron.yaml).

En este archivo se dirán las dimensiones del dron

```yaml
carcasa:
  peso_base: 1.0           # kg
  peso_brazos: 0.1         # kg (por brazo)
  peso_motor: 0.05         # kg (por motor)
  longitud_brazo: 0.2      # metros
  altura_base: 0.05        # metros
  radio_motor: 0.02        # metros
```

Características de las hélices
```yaml
helices:
  radio_helice: 0.05       # metros
  coeficiente_fuerza: 1.1  # N·s²/m⁴
  coeficiente_par: 0.01    # N·m·s²/m⁴
  sentido_giro:            # 1 = CW, -1 = CCW (alternado)
    - 1
    - -1
    - 1
    - -1
```

Finalmente características del plugin del motor para cambiar de la señal ```PWM``` con la que se alimenta a velocidad angular del mismo.

```yaml
plugin:
  intensidad_ruido: 0.02         # fuerza aleatoria ±N
  coeficiente_pwm_w: 0.05        # rad/s por unidad de PWM
  frecuencia_actualizacion: 100  # Hz
  max_pwm: 255                   # valor PWM máximo absoluto
```

## URDF
En un principio se intentó acceder a los parámetros del ```.yaml``` para la creación del URDF. Sin embargo, por ahora no está implementada dicha opción

El archivo [quadcopter.xacro](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/my_quadracopter/urdf/quadcopter.xacro) tiene, a parte del ```link``` de la base, una macro la cual pone tanto los 4 brazos como los 4 motores.

Se llaman de la siguiente forma
```xacro
  <xacro:arm_with_motor name="front"  parent="base_link" dx="0" dy="0.2" yaw="0"/>
  <xacro:arm_with_motor name="back"   parent="base_link" dx="0" dy="-0.2" yaw="0"/>
  <xacro:arm_with_motor name="left"   parent="base_link" dx="-0.2" dy="0" yaw="1.5708"/>
  <xacro:arm_with_motor name="right"  parent="base_link" dx="0.2" dy="0" yaw="1.5708"/>
```

Esto crea los brazos, los motores, y las juntas correspondientes.
El dron se verá así

![image](https://github.com/user-attachments/assets/5147c78a-c1a4-4497-b47d-24020bbc135c)

Finalmente se implementa el plugin.
```xacro
  <gazebo>
    <plugin name="quadcopter_plugin" filename="libquadcopter_plugin.so">
      <ros_topic>/pwm_signals</ros_topic>
    </plugin>
  </gazebo>
```

## Plugin
En este plugin introducimos un valor de ```PWM``` el cual se transformará mediante perámetros del ```.yaml``` en las fuerzas y en los torques producidos por los motores.

### Librerías
```cpp
#include "my_quadracopter/quadcopter_plugin.hpp"
```
Incluye el header del plugin, donde está definida la clase ```QuadcopterPlugin```.

```cpp
#include <ament_index_cpp/get_package_share_directory.hpp>
```
Permite obtener la ruta de instalación del paquete ROS (```my_quadracopter```) para localizar el archivo YAML.

```cpp
#include <yaml-cpp/yaml.h>
```
Incluye la librería para leer y parsear archivos ```.yaml``` (usado para los parámetros del dron).

```cpp
#include <ignition/math/Vector3.hh>
```
Define vectores 3D para aplicar fuerzas y torques.

```cpp
using namespace gazebo;
```
Usa el espacio de nombres ```gazebo``` para no escribir ```gazebo::``` todo el tiempo.


### Constructor

```cpp
QuadcopterPlugin::QuadcopterPlugin() : links_initialized_(false) {}
```
Constructor de la clase. Inicializa ```links_initialized_``` a false (se usará más tarde para saber si los links del dron ya se han localizado en el modelo).

### Load

```cpp
void gazebo::QuadcopterPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
```
Función llamada automáticamente por Gazebo al cargar el plugin.

```_model```: el modelo del dron en el mundo.

```_sdf```: parámetros del plugin desde el archivo ```.sdf```.

```cpp
model_ = _model;
```
Guarda el puntero al modelo para usarlo luego (acceder a los links del dron, etc).

```cpp
if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
}
```
Inicializa ROS2 si aún no ha sido inicializado.

```cpp
ros_node_ = rclcpp::Node::make_shared("quadcopter_plugin_node");
```
Crea un nodo ROS2 llamado ```quadcopter_plugin_node``` para poder suscribirse a tópicos.

```cpp
auto yaml_path = ament_index_cpp::get_package_share_directory("my_quadracopter") + "/config/dron.yaml";
```
Construye la ruta absoluta al archivo de configuración ```dron.yaml```.

```cpp
auto config = YAML::LoadFile(yaml_path);
```
Carga el archivo ```.yaml```.

```cpp
coef_pwm_w_ = config["plugin"]["coeficiente_pwm_w"].as<double>();
coef_force_ = config["helices"]["coeficiente_fuerza"].as<double>();
coef_torque_ = config["helices"]["coeficiente_par"].as<double>();
radius_ = config["helices"]["radio_helice"].as<double>();
```
Lee los parámetros desde el ```.yaml``` y los guarda en atributos de clase.

```cpp
motor_links_ = { "front_motor", "back_motor", "left_motor", "right_motor" };
```
Define los nombres de los motores (los ```links``` que representan hélices).

```cpp
pwm_values_.resize(4, 0);
```
Inicializa el vector de señales ```PWM``` con cuatro valores en cero.

```cpp
std::string topic = "/pwm_signals";
if (_sdf->HasElement("ros_topic")) {
    topic = _sdf->Get<std::string>("ros_topic");
}
```
Define el tópico por defecto ```/pwm_signals```, pero permite sobreescribirlo desde el ```.sdf```.

```cpp
sub_ = ros_node_->create_subscription<std_msgs::msg::Int32MultiArray>(
    topic, 10, std::bind(&QuadcopterPlugin::OnRosMsg, this, std::placeholders::_1)
);
```
Se suscribe al tópico para recibir mensajes ```PWM``` como un ```Int32MultiArray```.

```cpp
std::thread([node = ros_node_]() { rclcpp::spin(node); }).detach();
```
Lanza el nodo ROS en un hilo separado para que pueda escuchar mensajes mientras Gazebo corre.

```cpp
gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&QuadcopterPlugin::ApplyForces, this));
```
Registra una función callback ```ApplyForces``` que se ejecutará en cada ciclo de simulación.

### Callback 

```cpp
void QuadcopterPlugin::OnRosMsg(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    pwm_values_ = msg->data;
    ApplyForces();
}
```
Al recibir un mensaje ```PWM```, guarda los valores y llama a ```ApplyForces``` inmediatamente para aplicarlos.

```cpp
void gazebo::QuadcopterPlugin::ApplyForces()
```
Método llamado en cada ciclo de simulación para aplicar fuerza/torque en cada motor.

```cpp
if (!model_) return;
```
Evita errores si el modelo aún no está cargado.

```cpp
if (!links_initialized_) {
    links_.clear();
```
Si aún no se encontraron los ```links``` motores, intenta localizarlos.

```cpp
    for (const auto& name : motor_links_) {
        auto link = model_->GetLink(name);
        if (!link) return;
        links_.push_back(link);
    }
    links_initialized_ = true;
}
```
Busca cada ```link``` motor por su nombre. Si falta alguno, sale y lo intentará de nuevo en el siguiente ciclo.

```cpp
for (size_t i = 0; i < pwm_values_.size() && i < links_.size(); ++i) {
```
Itera sobre cada motor (y valor ```PWM``` recibido).

```cpp
double w = coef_pwm_w_ * pwm_values_[i];
double thrust = coef_force_ * w * w * radius_ * radius_;
double torque_z = coef_torque_ * w;
```
Convierte el ```PWM``` a velocidad angular ```w```, y luego calcula fuerza y torque.

```cpp
links_[i]->AddRelativeForce({0, 0, thrust});
links_[i]->AddRelativeTorque({0, 0, torque_z});
```
Aplica fuerza y torque en el eje Z en el sistema local del link (por eso se usa ```AddRelativeForce```).

### Final

```cpp
GZ_REGISTER_MODEL_PLUGIN(QuadcopterPlugin)
```
Macro de Gazebo para registrar el plugin y permitir que sea cargado desde un archivo ```SDF``` o ```URDF```.

## Prueba

Para comprobar si todo está correcto se ha realizado el nodo [prueba.cpp](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/my_quadracopter/src/prueba.cpp)
Este nodo eleva y desciende el valor de PWM según las teclas pulsadas en la terminal.

# Software