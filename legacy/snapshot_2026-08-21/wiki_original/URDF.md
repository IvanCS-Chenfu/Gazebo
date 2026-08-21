# Índice

- [1. Introducción](#1-Introducción)

- [2. Componentes Físicos](#2-Componentes-Físicos)
   - [2.1. Links](#21-Links)
   - [2.2. Joints](#22-Joints)
   - [2.3. Frames](#23-Frames)
   - [2.4. Gazebo](#24-Gazebo)

- [3. Macros](#3-Macros)

- [4. Utilizar datos ROS2](#4-Utilizar-datos-ROS2)

# 1. Introducción
Un archivo [URDF](https://en.wikipedia.org/wiki/URDF) es aquel que tiene toda la información física de nuestro robot (componentes, articulaciones, sensores, colores, físicas...). Los archivos ```.urdf``` se limitan solamente a la información del robot, sin embargo, utilizando archivos ```.xacro``` se pueden utilizar macros que simplifican el código y comunicar el archivo con un nodo ROS2.

En esta hoja se mostrarán los frames de los "links" y los "joints" utilizando el tópico ```/tf``` y "RViz". Para obtener el tópico ```/tf``` seguir los pasos de la [hoja anterior](https://github.com/IvanCS-Chenfu/Gazebo/wiki/Generar-URDF#3-Crear-Launch). Para activar "RViz", escribir lo siguiente en la terminal:

```bash
rviz2
```

En el software "RViz" se deberá cambiar el valor de ```Global Options >> Fixed Frame``` por el link principal de tu modelo URDF. También se deberá añadir los subapartados ```TF``` y ```RobotModel``` (en este último habrá que poner ```/robot_description``` en ```RobotModel >> Description Topic```).

<p align="center">
   <img width="339" height="367" alt="image" src="https://github.com/user-attachments/assets/3a040490-a8ff-4b17-93cf-8913af2be059" />
</p>

Cualquier archivo ```.xacro``` tendrá el siguiente formato:

```xml
<?xml version="1.0"?>
<robot name="<NOMBRE_ROBOT>" xmlns:xacro="http://www.ros.org/wiki/xacro">

   <!-- CUERPO ROBOT (LINKS, JOINTS, MACROS, GAZEBO...) -->

</robot>
```

# 2. Componentes Físicos
## 2.1. Links
Un "Link" es un sólido rígido el cual forma parte del robot. Un "Link" se compone de un nombre y 3 partes de la siguiente manera:

```xml
<link name="<NOMBRE_LINK>">

  <visual>
     <!-- CUERPO VISUAL -->
  </visual>

  <collision>
     <!-- CUERPO COLISIÓN -->
  </collision>

  <inertial>
     <!-- CUERPO INERCIA -->
  </inertial>

</link>
```

- Cuerpo de ```<visual>```

```xml
<visual>

  <origin xyz="<PX> <PY> <PZ>" rpy="<ROLL> <PITCH> <YAW>"/> <!-- OPCIONAL -->

  <geometry>
     <!-- CUERPO GEOMETRÍA -->
  </geometry>

  <material name="<NOMBRE_MATERIAL>">
     <!-- CUERPO MATERIAL -->
  </material>

</visual>
```

- Cuerpo de ```<collision>```
```xml
<collision>

  <origin xyz="<PX> <PY> <PZ>" rpy="<ROLL> <PITCH> <YAW>"/> <!-- OPCIONAL -->

  <geometry>
     <!-- CUERPO GEOMETRÍA -->
  </geometry>

</collision>
```

- Cuerpo de ```<inertial>```
```xml
<inertial>

  <origin xyz="<PX> <PY> <PZ>" rpy="<ROLL> <PITCH> <YAW>"/> <!-- OPCIONAL -->

  <mass value="<MASA>"/>

  <inertia ixx="<IXX>" ixy="<IXY>" ixz="<IXZ>" iyy="<IYY>" iyz="<IYZ>" izz="<IZZ>"/>

</inertial>
```

- Cuerpo de ```<geometry>```

```xml
<geometry>

    <!-- ELEGIR UNO DE LOS SIGUIENTES -->
    <box size="<X> <Y> <Z>"/>                <!-- Paralelepípedo -->
    <cylinder radius="<R>" length="<L>"/>    <!-- Cilindro -->
    <sphere radius="<R>"/>                   <!-- Esfera -->
    <mesh filename="package://mi_pkg/meshes/pieza.stl" scale="1 1 1"/>    <!-- Malla escalada (formatos comunes: .stl, .dae, .obj)-->

</geometry>
```

- Cuerpo de ```<material>```

```xml
<material name="<NOMBRE_MATERIAL>">

   <!-- ELEGIR UNO DE LOS SIGUIENTES -->
   <color rgba="<R> <G> <B> <Transparencia>"/>   <!-- Valores del 0 al 1 -->
   <texture filename="package://mi_pkg/textures/metal.png"/>    <!-- Textura dada por una imagen -->

</material>
```

## 2.2. Joints
Un "Joint" es una articulación que une dos "Joints". Crea el "Frame" del "Link" hijo (se habla de ello en el apartado [Frames](https://github.com/IvanCS-Chenfu/Gazebo/wiki/URDF#23-Frames)). El formato de un "Joint" es el siguiente:

```xml
<!-- OBLIGATORIO -->
<joint name="..." type="...">
  <parent link="..."/>
  <child link="..."/>
  
  <!-- MUY COMÚN -->
  <origin xyz="..." rpy="..."/>
  
  <!-- DEPENDE DEL TIPO DE JOINT -->
  <axis xyz="..."/>
  <limit lower="..." upper="..." effort="..." velocity="..."/>

  <!-- OPCIONES AVANZADAS -->
  <dynamics damping="..." friction="..."/>
  <mimic joint="..." multiplier="..." offset="..."/>
  <calibration rising="..." falling="..."/>
  <safety_controller soft_lower_limit="..." soft_upper_limit="..." k_position="..." k_velocity="..."/>

</joint>
```

- Nombre y Tipo: Es necesario nombrar a un "Joint" para referenciarlo y decir el tipo de articulación que queremos entre los 2 "Links"

```xml
<joint name="<NOMBRE_JOINT>" type="<TIPO_JOINT>">
```

Los tipos son los siguientes:
   - ```fixed```: Rotación con límites (servomotor).
   - ```revolute```: Rotación sin límites (motor).
   - ```continuous```: Traslación con límites.
   - ```prismatic```: Sin movimiento
   - ```floating```: 6 DoF libres (pose completa)
   - ```planar```: Movimiento en un plano (2 traslaciones + 1 rotación)

- Padre e Hijo: Dice respecto a qué "Frame" (padre) se va a crear el nuevo "Frame" (hijo).
```xml
<parent link="<NOMBRE_LINK_PADRE>"/>
<child link="<NOMBRE_LINK_HIJO>"/>
```

- Pose relativo entre "Frames": Respecto al "Frame" del padre, se dice donde se va a colocar el "Frame" del hijo.
```xml
<origin xyz="<PX> <PY> <PZ>" rpy="<ROLL> <PITCH> <YAW>"/>
```

- Eje de Movimiento: En que eje se produce el giro/traslación. Gazebo/ROS normalizan el vector.
```xml
<axis xyz="<AX> <AY> <AZ>"/>   <!-- Normalmente no se usa en "fixed", "floating" y "planar" -->
```

- Límites en el Movimiento: Tanto en rango de desplazamiento como en esfuerzo y velocidad.
```xml
<limit lower="<DESP_MIN>" upper="<DESP_MAX>" effort="<ESFUERZO_MAX>" velocity="<VEL_MAX>"/>
<!-- Normalmente no se usa en "fixed", "floating" y "planar" -->
<!-- En "revolute" (rad, N*m y rad/s) y "prismatic" (m, N y m/s) se usan todos -->
<!-- En "continuous" (rad, N*m y rad/s) no se usan "lower" y "upper" -->
```

- Dinámica: Amortiguamiento y fricción en la articulación (entre "Links").
```xml
<dynamics damping="<VALOR_AMORTIGUAMIENTO>" friction="<VALOR_FRICCIÓN>"/>
```

- Mímica: Cuando queremos que una articulación copie el movimiento de otra
```xml
<mimic joint="<NOMBRE_JOINT_A_COPIAR>" multiplier="<GANANCIA>" offset="<OFFSET>"/>
<!-- q_mimic = multiplier * q_a_copiar + offset -->
```

- Calibración: Se puede usar para sensores. Pone la articulación en una pose cuando el sensor detecta un flanco ascendente o uno descendente.
```xml
<calibration rising="<POSE_FLANCO_ASCENDENTE>" falling="<POSE_FLANCO_ASCENDENTE>"/>
```

- Límites Suaves y Ganancias para Seguridad: util sobre todo en ros_control/robots reales.
```xml
<safety_controller
  soft_lower_limit="-1.2"   <!-- Limites suaves antes del límite "hard" anterior -->
  soft_upper_limit="1.2"
  k_position="10.0"         <!-- ganancia "virtual" para empujar hacia dentro del rango -->
  k_velocity="1.0"/>        <!-- ganancia asociada a la velocidad -->
```

## 2.3. Frames

El Sistema de Referencia (o "Frame") ```world``` será siempre el centro del "tablero". Los siguientes sistemas de referencia coincidirán con la posición y orientación de las articulaciones creadas ("Joints").

En el archivo ```.xacro```, los "Links" que no tengan padre (no exista un "Joint" en el cual dicho "Link" no sea el hijo de otro "Link"), su "Frame" se encontrará en el "Frame" ```world```.

(En cuanto a "norma general" me voy a referir a cuando el campo ```<origin>``` dentro de ```<link>``` no existe o es 0).

Por norma general, el centro de todos los "Links" se encontrará en su "Frame" el cual es creado de la siguiente manera: El tag ```<joint>``` crea un nuevo frame (child) a una distancia y orientación relativa respecto al "Frame" del padre.

```xml
<joint name="joint1" type="fixed">
  <parent link="base_link"/>
  <child link="link1"/>
  <origin xyz="<PX> <PY> <PZ>" rpy="<ROLL> <PITCH> <YAW>"/>
</joint>
```

En el ejemplo donde se [modifican los "Joints"](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/generar_URDF/urdf/model_joints.xacro), se puede observar esta norma.

```xml
<joint name="joint1" type="fixed">
  <parent link="base_link"/>
  <child link="link1"/>
  <origin xyz="1 0 0" rpy="0 0 0"/>
</joint>

<joint name="joint2" type="fixed">
  <parent link="link1"/>
  <child link="link2"/>
  <origin xyz="0 0 1" rpy="0 0 1.57"/> 
</joint>
```

<p align="center">
   <img width="520" height="510" alt="image" src="https://github.com/user-attachments/assets/94dbaf92-613d-4db8-9f3a-0cf75d4bf459" />
</p>


En cuanto a la modificación del campo ```<origin>``` dentro de ```<link>``` (no "norma general"), el "Link" será desplazado lo decidido por el campo ```<origin>``` respecto a su "Frame" (el cual se sigue creando con el ```<joint>```).

```xml
<origin xyz="<PX> <PY> <PZ>" rpy="<ROLL> <PITCH> <YAW>"/>
```

En el ejemplo donde se [modifican los "Links"](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/generar_URDF/urdf/model_links.xacro), sin modificar los "Joints", se observa como todos los frames coinciden pero los "Links" están en lugares diferentes.

<p align="center">
   <img width="448" height="512" alt="image" src="https://github.com/user-attachments/assets/385ab0d6-9ed9-4864-a606-05b3c92dda4b" />
</p>

## 2.4. Gazebo
Gazebo es un simulador de físicas el cual utiliza "plugins" para conectarse al URDF de manera correcta. Debido a ello no se observan los colores dados por el campo ```<material>```. Para ello se utiliza un campo llamado ```<gazebo>``` para darle ciertas condiciones físicas a un "Link" ya creado.

```xml
<gazebo reference="<NOMBRE_LINK_ELEGIDO>">

  <!-- Dar Color en Gazebo (<NOMBRE_COLOR> = Red/Blue/Grey/White...) -->
  <material>Gazebo/<NOMBRE_COLOR></material>
  
  <!-- Gazebo funciona con 2 fricciones en 2 direcciones perpendiculares (preguntar valores) -->
  <mu1><VALOR_FRICCIÓN_1></mu1>
  <mu2><VALOR_FRICCIÓN_2></mu2>
  
  <!-- Spring costant (kp) y damping (kd). Preguntar valores -->
  <kp><VALOR_RIGIDEZ></kp>
  <kd><VALOR_AMORTIGUAMIENTO></kd>
  
  <!-- Activa la colisión con otros "Links" y Activa la gravedad -->
  <selfCollide><TRUE_OR_FALSE></selfCollide>
  <gravity><TRUE_OR_FALSE></gravity>

</gazebo>
```

# 3. Macros
Existen varios macros los cuales sirven para reducir el tamaño del código en el archivo ```.xacro```. La mayoría de macros tienen este formato:

```xml
<!-- DECLARACIÓN DE LA MACRO -->
<xacro:macro name="<NOMBRE_MACRO>" params="<NOMBRE_PARAMETRO_1> <NOMBRE_PARAMETRO_2>">
  <!-- CUERPO MACRO -->
</xacro:macro>

<!-- USO DE LA MACRO -->
<xacro:<NOMBRE_MACRO> <NOMBRE_PARAMETRO_1>="<VALOR_PARAMETRO_1>" <NOMBRE_PARAMETRO_2>="<VALOR_PARAMETRO_2>"/>
```

Se pueden observar 2 ejemplos [joints](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/generar_URDF/urdf/macro_joints.xacro) y [links](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/generar_URDF/urdf/macro_links.xacro) en los cuales se usan macros (generales, propiedades y argumentos).

- Ejemplo Macro General: Se pueden utilizar valores predeterminados (default) utilizando ```:=``` en el apartado de ```params```. Como valores "default" se pueden usar tanto valores únicos (como en la masa) como vectores (como en el tamaño del cubo). Dentro del cuerpo de la macro se referenciarán los parámetros utilizando ```${<NOMBRE_PARAMETRO_i>}```. Dentro de los corchetes ```${...}``` se podrá realizar cualquier operación matemática.

```xml
<!-- DECLARACIÓN DE LA MACRO -->
<xacro:macro name="simple_link" params="name size:= '0 0 0' mass:= 1.0">
  <link name="${name}">
    <visual>
      <geometry>
        <box size="${size}"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="${mass}"/>
      <inertia ixx="${0.01*m}" ixy="0" ixz="0"
               iyy="${0.01*m}" iyz="0"
               izz="${0.01*m}"/>
    </inertial>
  </link>
</xacro:macro>
```

En cuanto al uso de la macro, se deberán poner todos los parámetros que no tengan valor "default" (como el nombre) y si se desea cambiar dichos valores (como en el caso de link1) se deberán poner los demás parámetros.
```xml
<!-- USO DE LA MACRO -->
<xacro:simple_link name="link1" mass="5.0" size="0.4 0.3 0.1"/>
<xacro:simple_link name="link2"/>
```

- Ejemplo Macro con Bloques: Se puede utilizar como parámetro un bloque entero utilizando ```*```.

```xml
<!-- DECLARACIÓN DE LA MACRO -->
<xacro:macro name="visual_with_origin" params="name *origin">
  <link name="${name}">
    <visual>
      <xacro:insert_block name="origin"/>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>
</xacro:macro>
```

```xml
<!-- USO DE LA MACRO -->
<xacro:visual_with_origin name="v1">
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</xacro:visual_with_origin>
```

- Ejemplo Macro Condicional: Solo se ejecuta dicha macro si ```${flag}``` es "1" o "true" (en caso de ```xacro:if```) o si ```${flag}``` es "0" o "false" (en caso de ```xacro:unless```).

```xml
<!-- USO DE LA MACRO -->
<xacro:if value="${flag_i}">
  <geometry>
    <mesh filename="package://pkg/meshes/model.stl"/>
  </geometry>
</xacro:if>
```

```xml
<!-- USO DE LA MACRO -->
<xacro:unless value="${flag_i}">
  <geometry>
    <box size="0.2 0.2 0.2"/>
  </geometry>
</xacro:unless>
```

- Ejemplo Macro Propiedades: Son variables a las cuales se le asigna un valor. Pueden encontrarse dentro de otra macro (variable local para la macro) o fuera (variable global para todo el script).

```xml
<!-- DECLARACIÓN DE LA MACRO -->
<xacro:property name="wheel_radius" value="0.1"/>
<xacro:property name="wheel_mass" value="2.0"/>
<xacro:property name="wheel_inertia" value="${(wheel_mass/2)*wheel_radius*wheel_radius}"/>
```

```xml
<!-- USO DE LA MACRO -->
${<NOMBRE_PROPIEDAD>}
```

- Ejemplo Macro Include: Se pueden icluir otros archivos ```.xacro``` utilizando ```xacro:include``` y el path del archivo.
```xml
<!-- USO DE LA MACRO -->
<xacro:include filename="$(find my_robot)/urdf/links.xacro"/>
<xacro:include filename="$(find my_robot)/urdf/joints.xacro"/>
```

- Ejemplo Macro Argumentos: Útil oara oasar valores desde la terminal, launch, o nodos ROS.
```xml
<!-- DECLARACIÓN DE LA MACRO -->
<xacro:arg name="<NOMBRE_ARGUMENTO>" default="<VALOR_ARGUMENTO>"/>
```

```xml
<!-- USO DE LA MACRO -->
$(arg <NOMBRE_ARGUMENTO>)
```

En cuanto al uso desde la terminal:
```bash
xacro robot.xacro robot_name:=demo
```

- Todas estas macros se pueden componer unas dentro de otras.

# 4. Utilizar datos ROS2
Para conectar el nodo generador con el archivo ```.xacro``` será necesario utilizar macros de argumentos. En estos dos ejemplos ([joints](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/generar_URDF/urdf/macro_joints.xacro) y [links](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/generar_URDF/urdf/macro_links.xacro)) se utilizan los siguientes argumentos:

```xml
<xacro:arg name="px1" default="1"/>
<xacro:arg name="pz2" default="1"/>
<xacro:arg name="n_90_grad2" default="1"/>

<xacro:property name="n_90_grad2" value="$(arg n_90_grad2)"/> <!-- Se pasa a propiedad para poder hacer cálculos con ella -->
```

Para cambiar el valor "default" de los argumentos se añadirá en el [nodo generador](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/generar_URDF/src/sim_URDF_args.cpp) las líneas siguientes en la función ```read_xacro()```.

```cpp
// Añadimos argumentos al archvo .xacro
std::ostringstream args;
args << " px1:=" << 2
     << " pz2:=" << 2
     << " n_90_grad2:=" << 0.5;

std::string command = "xacro " + xacro_path + args.str();      // Ejecutamos el procesador xacro por terminal
```

Para que en "RViz" también se observen los cambios realizados por los nuevos argumentos, en el [archivo launch](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/generar_URDF/launch/sim_world_args.launch.py) se deberá añadir:

```python
robot_description = Command([
                                    'xacro ', xacro_file,
                                    ' px1:=', '2',
                                    ' pz2:=', '2',
                                    ' n_90_grad2:=', '0.5'
                                ])
```

para que los tópicos ```/robot_description``` y ```/tf``` tengan los datos correctos.

<p align="center">
   <img width="557" height="607" alt="image" src="https://github.com/user-attachments/assets/8b7283b9-afe5-41b8-9a8d-d62dd472b81f" />
   <img width="557" height="607" alt="image" src="https://github.com/user-attachments/assets/7a4ba902-2671-4798-b60a-203ac38b2b25" />
</p>


