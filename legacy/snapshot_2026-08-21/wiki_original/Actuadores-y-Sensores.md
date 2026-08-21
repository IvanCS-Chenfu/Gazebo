# Índice

- [1. Introducción](#1-Introducción)

- [2. Actuadores](#2-Actuadores)
   - [2.1. Links](#21-Links)
   - [2.2. Joints](#22-Joints)
   - [2.3. Mundo Gazebo](#23-Mundo-Gazbo)

- [3. Sensores](#3-Sensores) 
   - [3.1. Links](#31-Links)
   - [3.2. Joints](#32-Joints)
   - [3.3. Otros Sensores](#33-Otros-Sensores)


# 1. Introducción
Los robots a simular en gazebo tendrán tanto actuadores (ejercer una fuerza, motor...) como sensores (cámara, lidar, IMU...). Los actuadores deberán ser capaces de obtener datos de tópicos ROS2 y actuar en gazebo (velocidades queridas de unas ruedas, fuerza ejercida...). Los sensores deberán ser capaces de obtener datos del entorno de gazebo y publicar en tópicos dichos datos.

Para realizar esto se utilizarán plugins de ROS2. Estos plugins no son nodos de ROS2, son scripts los cuales obtienen información del modelo URDF durante la simulación.

Estos plugins no se insertan muy bien utilizando ```colcon build```, muchas veces es necesario borrar las carpetas ```build```, ```install``` y ```log``` del "Workspace" y luego realizar el ```colcon build```.

Para insertar un plugin en un el paquete de ROS2, se deberá poner lo siguiente en el archivo [CMakeLists](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/actuadores_gazebo/CMakeLists.txt)

```CMake
add_library(<NOMBRE_EJECUTABLE_PLUGIN> SHARED src/<NOMBRE_SCRIPT_PLUGIN>.cpp)
ament_target_dependencies(<NOMBRE_EJECUTABLE_PLUGIN> gazebo_ros rclcpp geometry_msgs)
target_include_directories(<NOMBRE_EJECUTABLE_PLUGIN> PRIVATE include ${gazebo_ros_INCLUDE_DIRS})
target_link_libraries(<NOMBRE_EJECUTABLE_PLUGIN> ${gazebo_ros_LIBRARIES})


install(TARGETS
  <NOMBRE_EJECUTABLE_PLUGIN_1>
  <NOMBRE_EJECUTABLE_PLUGIN_2>

  LIBRARY DESTINATION lib
)
```

La dependencia importante a implementar es ```gazebo_ros```.
- En [CMakeList](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/actuadores_gazebo/CMakeLists.txt)
```CMake
find_package(gazebo_ros REQUIRED)
```

- En [package](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/actuadores_gazebo/package.xml)
```xml
<exec_depend>gazebo_ros</exec_depend>
```

# 2. Actuadores
## 2.1. Links
En cuanto a los "Links", se podrán crear plugins que hagan distintas cosas. En los ejemplos se utilizará [este modelo](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/actuadores_gazebo/urdf/model_links.xacro) en el cual se relaciona el plugin con los "Links" y los "tópicos" de la siguiente manera.

```xml
<gazebo> 
  <plugin name="<NOMBRE_SCRIPT_PLUGIN>" filename="lib<NOMBRE_SCRIPT_PLUGIN>.so">
    <link_name1><NOMBRE_LINK_1>/link_name1>     <!-- link_name1 es el texto que está escrito entre "" en el plugin-->
    <topic1><NOMBRE_TÓPICO_LINK_1></topic1>     <!-- topic1 es el texto que está escrito entre "" en el plugin-->

    <link_name2><NOMBRE_LINK_2></link_name2>
    <topic2><NOMBRE_TÓPICO_LINK_2></topic2>

    <link_name3><NOMBRE_LINK_3></link_name3>
    <topic3><NOMBRE_TÓPICO_LINK_3></topic3>
  </plugin>
</gazebo>
```

### 2.1.1. Aplicar Fuerzas y Torques
Para ello se seguira [este plugin](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/actuadores_gazebo/src/fuerzas_links_plugin.cpp).

En este plugin se creará una clase con el nombre del plugin en mayúsculas y sin espacios.

```cpp
namespace gazebo 
{
   class FuerzasLinksPlugin : public ModelPlugin
   {
      /* CUERPO CLASE */

   }
   GZ_REGISTER_MODEL_PLUGIN(FuerzasLinksPlugin)
}
```

En la parte pública de la clase existirá una función ```void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {...}``` la cual obtendrá los "Links" del modelo y los asignará cada uno a un tópico. 

Se obtiene el modelo y los nombre de los "Links" y tópicos dados en el archivo ```.xacro```.
```cpp
model_    = model;
ros_node_ = gazebo_ros::Node::Get(sdf);

// --- Leer tres pares <link_name, topic_name> del SDF ---
link_names_[0]  = sdf->Get<std::string>("link_name1");     // Estos nombres son los que están entre <> en el archivo .xacro
link_names_[1]  = sdf->Get<std::string>("link_name2");
link_names_[2]  = sdf->Get<std::string>("link_name3");

topic_names_[0] = sdf->Get<std::string>("topic1");
topic_names_[1] = sdf->Get<std::string>("topic2");
topic_names_[2] = sdf->Get<std::string>("topic3");
```

Se obtienen los punteros a cada "Link" y se crea un subscriptor por cada "Link" utilizando como función "callback" ```OnWrench()```.
```cpp
links_[i] = model_->GetLink(link_names_[i]);

subs_[i] = ros_node_->create_subscription<geometry_msgs::msg::Wrench>
(topic_names_[i], 10, [this, i](const geometry_msgs::msg::Wrench::SharedPtr msg)
{
    this->OnWrench(i, msg);
});
```

En esta función ```Load()``` también se le dirá a la función ```OnUpdate()``` (función la cual aplicará las fuerzas) que actúe en cada instante de simulación de gazebo.

```cpp
update_conn_ = event::Events::ConnectWorldUpdateBegin(std::bind(&FuerzasLinksPlugin::OnUpdate, this));
```

En la parte privada de la clase tenemos la función "callback" ```OnWrench()``` la cual obtiene las fuerzas a aplicar del tópico correspondiente.
```cpp
void OnWrench(int idx, const geometry_msgs::msg::Wrench::SharedPtr msg) 
{
  last_forces_[idx]  = {msg->force.x,  msg->force.y,  msg->force.z};
  last_torques_[idx] = {msg->torque.x, msg->torque.y, msg->torque.z};
}
```

También estará la función ```OnUpdate()``` la cual aplicará esta fuerza a cada "Link".

```cpp
void OnUpdate() 
{
  for (int i = 0; i < 3; ++i)
  {
    if (links_[i])
    {
      links_[i]->AddRelativeForce(last_forces_[i]);     // Sobre el centro de masas
      links_[i]->AddRelativeTorque(last_torques_[i]);   // Sobre el centro de masas
    }
  }
}
```

También se puede aplicar las fuerzas a los "Links" de estas formas: ```link_->AddForce(last_force_);```, ```link_->AddTorque(last_torque_);```, ```link->AddForceAtWorldPosition( F, world_pos );```, y ```link->AddRelativeForceAtRelativePosition( F, rel_pos );```.

Una vez implantado el plugin, se lanzará en cuanto se ejecute el [nodo generador](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/actuadores_gazebo/src/sim_URDF.cpp) utilizando el [launch](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/actuadores_gazebo/launch/sim_world.launch.py). Se podrá obsevar con ```ros2 topic list``` como los tópicos están listos para publicar en ellos.

<p align="center">
   <img width="121" height="61" alt="image" src="https://github.com/user-attachments/assets/587d3346-7686-4b84-aa4a-3535dd7e9c69" />
</p>

Para publicar, utilizar (cambiando los valores de fuerzas):
```bash
ros2 topic pub <NOMBRE_TÓPICO> geometry_msgs/msg/Wrench "{force: {x: 0.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

### 2.1.2. Aplicar una Velocidad
Para ello se seguira [este plugin](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/actuadores_gazebo/src/velocidades_links_plugin.cpp).

La función ```Load()``` será exáctamente igual aunque cambiando el tipo de dato del tópico del subscriptor a ```geometry_msgs::msg::Twist``` y la función "callback" a ```OnTwist()```.

En la parte privada, la función "callback" cambia al recibir otro tipo de dato.

```cpp
void OnTwist(int idx, const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_lin_[idx] = {msg->linear.x,  msg->linear.y,  msg->linear.z};
  last_ang_[idx] = {msg->angular.x, msg->angular.y, msg->angular.z};
}
```

La función ```OnUpdate()``` aplica las velocidades de la siguiente manera. Estas velocidades serán aplicadas de forma instantánea (el "Link" no acelerará hasta obtener la velocidad deseada).
```cpp
void OnUpdate()
{
  for (int i = 0; i < 3; ++i)
  {
    if (links_[i])
    {
      // Setea velocidades (modo “ideal/kinemático”)
      links_[i]->SetLinearVel(last_lin_[i]);
      links_[i]->SetAngularVel(last_ang_[i]);
    }
  }
}
```

Para publicar, utilizar (cambiando los valores de velocidades):
```bash
ros2 topic pub <NOMBRE_TÓPICO> geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

### 2.1.3. Teletransportar a un Punto

## 2.2. Joints
En cuanto a los "Joints", se podrán crear plugins que hagan distintas cosas. En los ejemplos se utilizará [este modelo](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/actuadores_gazebo/urdf/model_joints.xacro) en el cual se relaciona el plugin con los "Joints" y los "tópicos" de la siguiente manera.

```xml
<gazebo> 
    <plugin name="<NOMBRE_SCRIPT_PLUGIN>" filename="lib<NOMBRE_SCRIPT_PLUGIN>.so">
      <joint_name1><NOMBRE_JOINT_1></joint_name1>    <!-- joint_name1 es el texto que está escrito entre "" en el plugin-->
      <topic1><NOMBRE_TÓPICO_JOINT_1></topic1>     <!-- topic1 es el texto que está escrito entre "" en el plugin-->

      <joint_name2><NOMBRE_JOINT_2></joint_name2>
      <topic2><NOMBRE_TÓPICO_JOINT_2></topic2>

      <joint_name3><NOMBRE_JOINT_3></joint_name3>
      <topic3><NOMBRE_TÓPICO_JOINT_3></topic3>
    </plugin>
  </gazebo>
```
### 2.2.1. Aplicar Fuerzas y Torques
Para ello se seguira [este plugin](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/actuadores_gazebo/src/fuerzas_joints_plugin.cpp).

En este plugin se creará una clase con el nombre del plugin en mayúsculas y sin espacios.

```cpp
namespace gazebo 
{
   class FuerzasJointsPlugin : public ModelPlugin
   {
      /* CUERPO CLASE */

   }
   GZ_REGISTER_MODEL_PLUGIN(FuerzasJointsPlugin)
}
```

En la parte pública de la clase existirá una función ```void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {...}``` la cual obtendrá los "Joints" del modelo y los asignará cada uno a un tópico. 

Se obtiene el modelo y los nombre de los "Links" y tópicos dados en el archivo ```.xacro```.
```cpp
model_    = model;
ros_node_ = gazebo_ros::Node::Get(sdf);

// --- Leer tres pares <joint_name, topic_name> del SDF ---
joint_names_[0] = sdf->Get<std::string>("joint_name1");   // Estos nombres son los que están entre <> en el archivo .xacro
joint_names_[1] = sdf->Get<std::string>("joint_name2");
joint_names_[2] = sdf->Get<std::string>("joint_name3");

topic_names_[0] = sdf->Get<std::string>("topic1");
topic_names_[1] = sdf->Get<std::string>("topic2");
topic_names_[2] = sdf->Get<std::string>("topic3");
```

Se obtienen los punteros a cada "Joint" y se crea un subscriptor por cada "Joint" utilizando como función "callback" ```OnEffort()```. En esta caso como el "Joint" actúa sobre un solo eje, el mensaje a recibir será de tipo ```std_msgs::msg::Float64```.
```cpp
joints_[i] = model_->GetJoint(joint_names_[i]);

subs_[i] = ros_node_->create_subscription<std_msgs::msg::Float64>
(topic_names_[i], 10, [this, i](const std_msgs::msg::Float64::SharedPtr msg)
{
    this->OnEffort(i, msg);
});
```

En esta función ```Load()``` también se le dirá a la función ```OnUpdate()``` (función la cual aplicará las fuerzas) que actúe en cada instante de simulación de gazebo.

```cpp
update_conn_ = event::Events::ConnectWorldUpdateBegin(std::bind(&FuerzasJointsPlugin::OnUpdate, this));
```

En la parte privada de la clase tenemos la función "callback" ```OnWrench()``` la cual obtiene las fuerzas a aplicar del tópico correspondiente.
```cpp
void OnEffort(int idx, const std_msgs::msg::Float64::SharedPtr msg)
{
  last_effort_[idx] = msg->data;
}
```

También estará la función ```OnUpdate()``` la cual aplicará esta fuerza a cada "Joint".

```cpp
void OnUpdate()
{
  for (int i = 0; i < 3; ++i)
  {
    if (joints_[i])
    {
      // axis index 0: joints 1DOF (revolute/continuous/prismatic)
      joints_[i]->SetForce(0, last_effort_[i]);
    }
  }
}
```

Para publicar, utilizar (cambiando los valores de fuerzas):
```bash
ros2 topic pub <NOMBRE_TÓPICO> std_msgs/msg/Float64 "{data: 0.0}" --once
```

### 2.2.2. Aplicar una Velocidad
Para ello se seguira [este plugin](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/actuadores_gazebo/src/velocidades_joints_plugin.cpp).

La función ```Load()``` será exáctamente igual aunque cambiando la función "callback" a ```OnTwist()``` y añadiendo lo sigiuente:

```cpp
if (sdf->HasElement("max_effort"))
{
  max_effort_ = sdf->Get<double>("max_effort");
}
else
{
  max_effort_ = 2.0;
}

joints_[i]->SetParam("fmax", 0, max_effort_);
```

Esto es necesario para que el "Joint" actúe de manera realista y cambie su velocidad de forma instantánea sino que vaya acelerando con un esfuerzo máximo. El código ```sdf->HasElement("max_effort")``` es por si el archivo ```.xacro``` en el tag ```<plugin>``` tiene el subtag ```<max_effort><VALOR_MAX_EFFORT></max_effort>```.

En la parte privada, la función "callback" no cambia prácticamente nada

```cpp
void OnTwist(int idx, const std_msgs::msg::Float64::SharedPtr msg) 
{
  last_vel_[idx] = msg->data;
}
```

La función ```OnUpdate()``` aplica las velocidades de la siguiente manera. Estas velocidades no serán aplicadas de forma instantánea (el "Joint" acelerará hasta obtener la velocidad deseada).
```cpp
void OnUpdate()
{
  for (int i = 0; i < 3; ++i)
  {
    if (joints_[i])
    {
      // axis 0 para joints 1DOF
      joints_[i]->SetVelocity(0, last_vel_[i]);
    }
  }
}
```

Para publicar, utilizar (cambiando los valores de velocidades):
```bash
ros2 topic pub <NOMBRE_TÓPICO> geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

## 2.3. Mundo Gazebo

# 3. Sensores 
## 3.1. Links
El [archivo .xacro](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/sensores_gazebo/urdf/model_links.xacro) es prácticamente el mismo que para los actuadores. Sin embargo, se añaden varios "subtags" útiles para los plugins.

```xml
<gazebo> 
  <plugin name="<NOMBRE_SCRIPT_PLUGIN>" filename="lib<NOMBRE_SCRIPT_PLUGIN>.so">
    <link_name1><NOMBRE_LINK_1>/link_name1>     <!-- link_name1 es el texto que está escrito entre "" en el plugin-->
    <topic1><NOMBRE_TÓPICO_LINK_1></topic1>     <!-- topic1 es el texto que está escrito entre "" en el plugin-->

    <link_name2><NOMBRE_LINK_2></link_name2>
    <topic2><NOMBRE_TÓPICO_LINK_2></topic2>

    <link_name3><NOMBRE_LINK_3></link_name3>
    <topic3><NOMBRE_TÓPICO_LINK_3></topic3>

    <publish_rate>50.0</publish_rate>  <!-- Hz -->
    <frame_id>world</frame_id>         <!-- nombre para header.frame_id -->
    <frame_propio>true</frame_propio>  <!-- Vel o Acc respecto a su propio frame -->
  </plugin>
</gazebo>
```

### 3.1.1. Obtener Pose
Para ello se seguira [este plugin](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/sensores_gazebo/src/pose_links_plugin.cpp).

La función ```Load()``` será exáctamente igual aunque cambiando subscriptores por publicadores (ya que ahora el plugin publicará la pose del "Link").

```cpp
pubs_[i] = ros_node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_names_[i], 10);
```

También se cambiará la forma de conectar ```OnUpdate()``` con el mundo (ya que se va a obtener información de Gazebo).
```cpp
update_conn_ = event::Events::ConnectWorldUpdateBegin(std::bind(&PoseLinksPlugin::OnUpdate, this, std::placeholders::_1));
```

Este publicador no utilizará el tiempo de los nodos de ROS2 (utiliza el tiempo de Gazebo), por ello no se utilizará de la mima forma que el [este tutorial](https://github.com/IvanCS-Chenfu/ROS2/wiki/Publisher-Subscriber#2-Publisher). Para seguir el ```publish_rate_``` querido por el usuario, se tomará un tiempo en un momento y se comparará con el actual para ver si ha pasado ese ```publish_rate_```. Si ha pasado ese tiempo, se publicará. Todo esto ocurre en la función ```OnUpdate()```.

```cpp
enviar = false;
const double dt = (sim_time - last_sim_time_).Double();
if (dt >= (1.0 / publish_rate_))
{
  enviar = true;
  last_sim_time_ = sim_time;
}


if (enviar)
{
  // Para pasar al header del mensaje
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(sim_time.sec);
  stamp.nanosec = static_cast<uint32_t>(sim_time.nsec);

  for (int i = 0; i < 3; ++i)
  {
    if (links_[i] && pubs_[i])
    {   
       const auto pose = links_[i]->WorldPose();

       geometry_msgs::msg::PoseStamped msg;
       msg.header.stamp = stamp;
       msg.header.frame_id = frame_id_;

       msg.pose.position.x = pose.Pos().X();
       msg.pose.position.y = pose.Pos().Y();
       msg.pose.position.z = pose.Pos().Z();

       msg.pose.orientation.x = pose.Rot().X();
       msg.pose.orientation.y = pose.Rot().Y();
       msg.pose.orientation.z = pose.Rot().Z();
       msg.pose.orientation.w = pose.Rot().W();

       pubs_[i]->publish(msg);
     }
  }
}
```

Una vez instalado el plugin, se puede obtener la pose:
```bash
ros2 topic echo <NOMBRE_TÓPICO>
```

### 3.1.2. Obtener Velocidad
Para ello se seguira [este plugin](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/sensores_gazebo/src/vel_links_plugin.cpp).

La función ```Load()``` será exáctamente igual.

Dentro de la función ```OnUpdate()``` lo único que cambiará será la obtención de la velocidad del "Link". Aquí se obtiene la pose también con el fin de trasladar la velocidad obtenida (respecto al "Frame" del mundo) al "Frame" del propio "Link".

```cpp
if (enviar)
{
  // Para pasar al header del mensaje
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(sim_time.sec);
  stamp.nanosec = static_cast<uint32_t>(sim_time.nsec);

  for (int i = 0; i < 3; ++i)
  {
    if (links_[i] && pubs_[i])
    {   
      auto v_lin = links_[i]->WorldLinearVel();   // ignition::math::Vector3d
      auto v_ang = links_[i]->WorldAngularVel();  // ignition::math::Vector3d
          
          
      if (frame_propio_)
      {
        const auto pose = links_[i]->WorldPose();
        const auto R = pose.Rot();  // world -> link

        v_lin  = R.RotateVectorReverse(v_lin);
        v_ang  = R.RotateVectorReverse(v_ang);

        frame_id_ = link_names_[i];
      }

      geometry_msgs::msg::TwistStamped msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = frame_id_;

      msg.twist.linear.x  = v_lin.X();
      msg.twist.linear.y  = v_lin.Y();
      msg.twist.linear.z  = v_lin.Z();

      msg.twist.angular.x = v_ang.X();
      msg.twist.angular.y = v_ang.Y();
      msg.twist.angular.z = v_ang.Z();

      pubs_[i]->publish(msg);
    }
  }
}
```

### 3.1.2. Obtener Aceleración
Para ello se seguira [este plugin](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/sensores_gazebo/src/acc_links_plugin.cpp).

Para calcular la velocidad no existe una función concreta. Se calculará por derivación discreta ($a = \frac{v_f-v_0}{t_f-t_0}$).

La función ```Load()``` será exáctamente igual, aunque se obtendrán las velocidades de los "Links" que servirán como $v_0$.
```cpp
// Inicializar prev con velocidad actual para evitar “picos” al arrancar
prev_v_lin_[i] = links_[i]->WorldLinearVel();
prev_v_ang_[i] = links_[i]->WorldAngularVel();
has_prev_[i]   = false;  // publicaremos la 2ª vez con derivada válida
```

Dentro de la función ```OnUpdate()``` lo único que cambiará será el cálculo de la aceleración del "Link". También se obtiene la pose con el fin de trasladar la aceleración obtenida (respecto al "Frame" del mundo) al "Frame" del propio "Link".

```cpp
if (enviar)
{
// Tiempo para derivada (desde última publicación)
const double dt = (sim_time - prev_time_).Double();

/* CUERPO */

auto v_lin = links_[i]->WorldLinearVel();   // ignition::math::Vector3d
auto v_ang = links_[i]->WorldAngularVel();  // ignition::math::Vector3d

/* CUERPO */

ignition::math::Vector3d a_lin = (v_lin - prev_v_lin_[i]) / dt;
ignition::math::Vector3d a_ang = (v_ang - prev_v_ang_[i]) / dt;

// Actualizar prev para la próxima
prev_v_lin_[i] = v_lin;
prev_v_ang_[i] = v_ang;
}
```

## 3.2. Joints
El [archivo .xacro](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/sensores_gazebo/urdf/model_joints.xacro) es prácticamente el mismo que para los actuadores. Sin embargo, se añaden varios "subtags" útiles para los plugins.

```xml
<gazebo> 
  <plugin name="<NOMBRE_SCRIPT_PLUGIN>" filename="lib<NOMBRE_SCRIPT_PLUGIN>.so">
    <joint_name1><NOMBRE_JOINT_1>/joint_name1>     <!-- link_name1 es el texto que está escrito entre "" en el plugin-->
    <topic1><NOMBRE_TÓPICO_JOINT_1></topic1>     <!-- topic1 es el texto que está escrito entre "" en el plugin-->

    <joint_name2><NOMBRE_JOINT_2></joint_name2>
    <topic2><NOMBRE_TÓPICO_JOINT_2></topic2>

    <joint_name3><NOMBRE_JOINT_3></joint_name3>
    <topic3><NOMBRE_TÓPICO_JOINT_3></topic3>

    <publish_rate>50.0</publish_rate>  <!-- Hz -->
  </plugin>
</gazebo>
```

### 3.2.1. Obtener Pose
Para ello se seguira [este plugin](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/sensores_gazebo/src/pose_joints_plugin.cpp). 

Lo único que cambia es el cuerpo de ```if(enviar)``` dentro de ```OnUpdate()``` (a parte del uso de ```joints_``` en vez de ```links_```):

```cpp
if (enviar)
{
  for (int i = 0; i < 3; ++i)
  {
    if (joints_[i] && pubs_[i])
    {   
      // axis 0 para joints 1DOF:
      // revolute/continuous => rad
      // prismatic          => m
      const double q = joints_[i]->Position(0);

      std_msgs::msg::Float64 msg;
      msg.data = q;

      pubs_[i]->publish(msg);
    }
  }
}
```

### 3.2.2. Obtener Velocidad
Para ello se seguira [este plugin](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/sensores_gazebo/src/pose_joints_plugin.cpp). 

Lo único que cambia es el uso de ```const double vel = joints_[i]->GetVelocity(0);``` en vez de ```const double q = joints_[i]->Position(0);``` en el cuerpo de ```if(enviar)```.

### 3.2.2. Obtener Aceleración
Para ello se seguira [este plugin](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/sensores_gazebo/src/pose_joints_plugin.cpp). 

Lo único que cambia es el cuerpo de ```if(enviar)``` dentro de ```OnUpdate()``` (a parte del uso de la obtención de velocidades y tiempos para calcular la aceleración como derivada discreta de la velocidad):

```cpp
if (enviar)
{
  // Tiempo para derivada (desde última publicación)
  const double dt = (sim_time - prev_time_).Double();
   
  for (int i = 0; i < 3; ++i)
  {
    if (joints_[i] && pubs_[i])
    {   
      const double vel = joints_[i]->GetVelocity(0);  // rad/s o m/s

      // Si es la primera vez (o dt muy pequeño), no publicamos derivada
      if (has_prev_[i] && dt > 1e-9)
      {
        const double acc = (vel - prev_vel_[i]) / dt;  // rad/s^2 o m/s^2

        prev_vel_[i] = vel;

        std_msgs::msg::Float64 msg;
        msg.data = acc;
        pubs_[i]->publish(msg);
      }
      else
      {
        prev_vel_[i] = vel;
        has_prev_[i] = true;
      }
    }
  }
  prev_time_ = sim_time;
}
```
## 3.3. Otros Sensores