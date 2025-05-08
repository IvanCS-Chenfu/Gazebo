#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>

class BoxSpawner : public rclcpp::Node {
public:
    BoxSpawner() : Node("box_spawner") {
        auto client = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

        while (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Esperando el servicio /spawn_entity...");
        }

        auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
        request->name = "espinas_bot";              //aqui
        request->xml = read_urdf();
        request->robot_namespace = "/espinas_bot";  //aqui
        request->initial_pose.position.x = 0.0;
        request->initial_pose.position.y = 0.0;
        request->initial_pose.position.z = 0.15;

        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Robot espinas_bot insertado exitosamente.");   //aqui
        } else {
            RCLCPP_ERROR(this->get_logger(), "Fallo al insertar el robot.");
        }
    }

private:
    std::string read_urdf() {
        std::string urdf_path = ament_index_cpp::get_package_share_directory("my_box_robot") + "/urdf/espinas_bot.xacro"; //aqui
        std::ifstream urdf_file(urdf_path);
        std::stringstream buffer;
        buffer << urdf_file.rdbuf();
        return buffer.str();
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BoxSpawner>());
    rclcpp::shutdown();
    return 0;
}