#include "rclcpp/rclcpp.hpp"

#include <gazebo_msgs/srv/spawn_entity.hpp>                 // Añadir interfaz usada en el servicio.
#include <ament_index_cpp/get_package_share_directory.hpp>  // Obtener la dirección del paquete y del Xacro

#include <fstream>
#include <sstream>                // Para usar stringstream
#include <cstdlib>               // Para usar popen y pclose

#include <chrono>
using namespace std::chrono_literals;

class Clase_Cliente : public rclcpp::Node 
{
    public:
        // Cambiamos la interfaz del servicio
        using SpawnEntity = gazebo_msgs::srv::SpawnEntity;

        Clase_Cliente() : rclcpp::Node("nombre_cliente_cpp")  
        {   
            // Llamamos al servicio "spawn_entity"
            objeto_cliente = this->create_client<SpawnEntity>("/spawn_entity");

            while (!objeto_cliente->wait_for_service(1s)) 
            {
                RCLCPP_INFO(this->get_logger(), "Servicio no disponible");
            }

            request = std::make_shared<SpawnEntity::Request>();
        }

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

    private:
        // Función que pasa el archivo ".xacro" a una variable tipo "string".
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
        

        rclcpp::Client<SpawnEntity>::SharedPtr objeto_cliente;
        std::shared_ptr<SpawnEntity::Request> request;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto objeto_nodo = std::make_shared<Clase_Cliente>();

    objeto_nodo->enviar_datos();

    // Elimino spin ya que quiero generar el URDF y eliminar el nodo

    rclcpp::shutdown();
    return 0;
}