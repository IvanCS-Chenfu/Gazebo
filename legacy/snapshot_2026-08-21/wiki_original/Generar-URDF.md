# Índice

- [1. Introducción](#1-Introducción)

- [2. Crear Cliente](#2-Crear-Cliente)
   - [2.1. C++](#21-C)
   - [2.2. Python NO ESCRITO](#22-Python)
   - [2.3. Package, CMakeLists y Setup](#23-Package-CMakeLists-y-Setup)

- [3. Crear Launch](#3-Crear-Launch)

# 1. Introducción

Un archivo URDF es aquel que define los componentes físicos (eslabones ```<link>``` y articulaciones ```<joint>```) de un robot y sus relaciones con nodos de ROS2 (explicado en la [hoja URDF](https://github.com/IvanCS-Chenfu/Gazebo/wiki/URDF)).

Para generar el modelo representado por un archivo URDF, se utilizará un nodo de ROS2 el cual obtendrá los datos URDF de un archivo ```.xacro``` y llamará al servicio ```/spawn_entity``` el cual hará aparecer el modelo del robot en gazebo.
 
# 2. Crear Cliente
## 2.1. C++
Se partirá de la [creación estándar de un cliente](https://github.com/IvanCS-Chenfu/ROS2/wiki/Servicio-Cliente#41-C) de ROS2. El ejemplo a comentar será [este](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/generar_URDF/src/sim_URDF.cpp).

La interfaz de servicio será la siguiente:

```cpp
#include <gazebo_msgs/srv/spawn_entity.hpp>                 // Añadir interfaz usada en el servicio.
```

Además se necesitarán otras 4 librerías:
```cpp
#include <ament_index_cpp/get_package_share_directory.hpp>  // Obtener la dirección del paquete y del Xacro

#include <fstream>
#include <sstream>                // Para usar stringstream
#include <cstdlib>               // Para usar popen y pclose
```

Al haber cambiado la interfaz del servicio debemos cambiar todos los ```paquete_cpp::srv::VarServicio>``` por ```gazebo_msgs::srv::SpawnEntity```.

El servicio a llamar será ```/spawn_entity```.
```cpp
objeto_cliente = this->create_client<SpawnEntity>("/spawn_entity");
```

Se creará en la parte privada de la clase una función la cual obtenga el archivo ```.xacro``` (el cual tiene el modelo URDF en su interior) y lo pase a tipo ```std::string``` para que pueda ser enviado al servicio.

```cpp
std::string read_xacro()
{   
    std::string nombre_paquete = "urdf_gazebo";         // Nombre del paquete
    std::string ruta_xacro = "/urdf/model.xacro";       // Path del .xacro

    std::string xacro_path = ament_index_cpp::get_package_share_directory(nombre_paquete) + ruta_xacro;

    std::string command = "xacro " + xacro_path;      // Ejecutamos el procesador xacro por terminal
    FILE* pipe = popen(command.c_str(), "r");         // Abrimos el comando como un stream
    if (!pipe)
    {
        RCLCPP_ERROR(this->get_logger(), "Error al ejecutar xacro.");
        return "";
    }

    char buffer[256];
    std::stringstream result;
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr)
    {
        result << buffer;                             // Leemos la salida del comando xacro
    }

    pclose(pipe);                                     // Cerramos el proceso
    return result.str();                              // Retornamos el contenido generado por xacro
}
```

Finalmente se usa la función ```enviar_datos()``` para enviar lo recibido por la función anterior (y otros parámetros como: posición de simulación, nombre, frame de referencia...) al servicio ```

```cpp
void enviar_datos()
{
    std::string xml = read_xacro();

    if (xml.empty()) 
    {
        RCLCPP_FATAL(this->get_logger(), "URDF vacío, abortando spawn");
    }
    else
    {
        // Declaramos los valores a enviar al servicio
        request->xml = xml;
        
        request->name = "model";
        request->robot_namespace = "/model";

        request->reference_frame = "world";

        request->initial_pose.position.x = 0.0;
        request->initial_pose.position.y = 0.0;
        request->initial_pose.position.z = 0.15;

        auto future_result = objeto_cliente->async_send_request(request);   // Enviamos los valores al servicio

        // Nos quedamos en bucle esperando hasta recibir la respuesta.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Robot model insertado exitosamente.");   //aqui
        } 
        else 
        {
            RCLCPP_ERROR(this->get_logger(), "Fallo al insertar el robot.");
        }
    }
}
```

Finalmente en el ```int main()``` se llama a la función ```enviar_datos()``` y se elimina el ```rclcpp::spin()``` para que una vez generado el modelo, el nodo se cierre.

```cpp
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto objeto_nodo = std::make_shared<Clase_Cliente>();

    objeto_nodo->enviar_datos();

    // Elimino spin ya que quiero generar el URDF y eliminar el nodo

    rclcpp::shutdown();
    return 0;
}
```

## 2.2. Python

## 2.3. Package, CMakeLists y Setup
En Paquetes C++ mínimo se deben implementar estas dependencias en el archivo [package.xml](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/generar_URDF/package.xml). Estas dependencias sirven tanto para utilizar los mensajes del servidor ```gazebo_msgs``` y utilizar los archivos ```.xacro```.

```xml
<depend>rclcpp</depend>
<depend>gazebo_msgs</depend>
<depend>ament_index_cpp</depend>

<exec_depend>xacro</exec_depend>
```

En cuanto al archivo [CMakeLists.txt](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/generar_URDF/CMakeLists.txt) se deberá implementar:
- Las dependencias:
```CMakeLists
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
```

- El directorio donde se encontrarán los archivos ```.xacro```.
```CMakeLists
install(DIRECTORY
  urdf

  DESTINATION share/${PROJECT_NAME}
)
```

- Ejecutar los nodos e instalarlos
```CMakeLists
add_executable(sim_URDF src/sim_URDF.cpp)
ament_target_dependencies(sim_URDF rclcpp gazebo_msgs ament_index_cpp)

install(TARGETS
  sim_URDF
  DESTINATION lib/${PROJECT_NAME}
)
```

No se implementará el código ```rosidl_generate_interfaces()``` ya que el ```/srv``` no es definido por nosotros.

Para que Gazebo muestre el servicio necesario, no se puede llamar desde la terminal como ```gazebo``` directamente, es necesario el plugin ```libgazebo_ros_factory.so```. Para ello se llamará desde la terminal como:

```bash
gazebo --verbose -s libgazebo_ros_factory.so
```

Finalmente se ejecutará el nodo ROS2 para generar el modelo en Gazebo.

# 3. Crear Launch
Para evitar tener que llamar a gazebo y luego al nodo, se puede crear un archivo ```.launch``` el cual realice ambas cosas a la vez.

Para ello se utilizará un archivo ```.launch.py``` ([este ejemplo](https://github.com/IvanCS-Chenfu/Gazebo/blob/main/generar_URDF/launch/sim_world.launch.py)) en el cual se ejecutará un proceso dentro del [LaunchDescription](https://github.com/IvanCS-Chenfu/ROS2/wiki/Archivo-Launch#52-Python).

```python
ExecuteProcess(
    cmd=[
        'gazebo',
        '--verbose',
        '-s', 'libgazebo_ros_factory.so'
    ],
    output='screen'
),
```

También puede ser bueno observar todos los ```frames``` de los links y los joints. Para ello es necesario los tópicos ```/tf``` y ```/tf_static``` los cuales solo aparecen al ejecutar el nodo ```robot_state_publisher```

```python
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
   pkg = get_package_share_directory('urdf_gazebo')
   xacro_file = os.path.join(pkg, 'urdf', 'model.xacro')

   robot_description = Command(['xacro ', xacro_file])

   return LaunchDescription([

      Node(
          package='robot_state_publisher',
          executable='robot_state_publisher',
          output='screen',
          parameters=[{
              'robot_description': robot_description,
              # opcional si tu URDF usa "world" en gazebo:
              # 'frame_prefix': 'model/'
          }]
      ),

      """
      CUERPO
      """

      ])
```

