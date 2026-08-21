// src/led_color_plugin.cpp
#include <thread>
#include <memory>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace gazebo
{
  class LedColorPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override
    {
      // 1. Guardamos modelo y world
      this->model = _model;
      this->world = _model->GetWorld();


      // 2. Nodo de Gazebo-transport (para publicar Visual)
      this->gzNode = transport::NodePtr(new transport::Node());
      this->gzNode->Init(this->world->Name());
      this->visPub = this->gzNode->Advertise<gazebo::msgs::Visual>("~/visual", 10);

      // 3. Inicializar RCLCPP (si no lo está aún)
      if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
      }
      // 4. Crear el node ROS 2 para esta instancia del plugin
      this->rosNode = std::make_shared<rclcpp::Node>("led_color_plugin_node");

      // 5. Suscripción al tópico /led_toggle
      this->sub = this->rosNode->create_subscription<std_msgs::msg::Bool>(
        "/led_toggle", 
        10,
        std::bind(&LedColorPlugin::OnToggleRos, this, std::placeholders::_1)
      );

      // 6. Spin en un hilo aparte para que lleguen callbacks ROS 2
      this->rosSpinner = std::thread([node=this->rosNode]() {
        rclcpp::spin(node);
      });

      // 7. Nombre completo del visual en Gazebo
      this->visualName       = this->model->GetName() + "::led_link::VISUAL";
      this->visualParentName = this->model->GetName() + "::led_link"; 
    }

    // Destructor: detener spinner de ROS 2
    ~LedColorPlugin()
    {
      if (rclcpp::ok()) {
        rclcpp::shutdown();
      }
      if (this->rosSpinner.joinable()) {
        this->rosSpinner.join();
      }
    }

  private:
    // Callback ROS 2
    void OnToggleRos(const std_msgs::msg::Bool::SharedPtr msg)
    {
    // 1) Prepara el mensaje Visual
    gazebo::msgs::Visual vizMsg;
    vizMsg.set_name(this->visualName);

    // **¡IMPORTANTE!** pon aquí el parent_name (el nombre del link)
    vizMsg.set_parent_name(this->visualParentName);

    // 2) Ajusta el color diffuse
    auto *diff = vizMsg.mutable_material()->mutable_diffuse();
    if (msg->data) {
        diff->set_r(1.0); diff->set_g(0.0);
        diff->set_b(0.0); diff->set_a(1.0);
    } else {
        diff->set_r(0.5); diff->set_g(0.5);
        diff->set_b(0.5); diff->set_a(1.0);
    }

    // 3) Publica para que Gazebo actualice el visual
    this->visPub->Publish(vizMsg);
    }

    // Miembros
    physics::ModelPtr                model;
    physics::WorldPtr                world;
    transport::NodePtr               gzNode;
    transport::PublisherPtr          visPub;

    std::shared_ptr<rclcpp::Node>    rosNode;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub;
    std::thread                      rosSpinner;

    std::string visualName;
    std::string visualParentName;
  };

  GZ_REGISTER_MODEL_PLUGIN(LedColorPlugin)
}
