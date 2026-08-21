#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <memory>
#include <string>

namespace gazebo {

class VelLinksPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override 
  {
    model_    = model;
    ros_node_ = gazebo_ros::Node::Get(sdf);

    // --- Leer tres pares <link_name, topic_name> del SDF ---
    link_names_[0]  = sdf->Get<std::string>("link_name1");
    link_names_[1]  = sdf->Get<std::string>("link_name2");
    link_names_[2]  = sdf->Get<std::string>("link_name3");

    topic_names_[0] = sdf->Get<std::string>("topic1");
    topic_names_[1] = sdf->Get<std::string>("topic2");
    topic_names_[2] = sdf->Get<std::string>("topic3");

    // --- Obtener punteros a links y crear suscripciones ---
    for (int i = 0; i < 3; ++i)
    {
      links_[i] = model_->GetLink(link_names_[i]);

      if (!links_[i]) 
      {
        RCLCPP_ERROR(ros_node_->get_logger(), "VelLinksPlugin: no encuentro link %s", link_names_[i].c_str());
      }
      else
      {
        // Crear suscripción a cada tópico
        subs_[i] = ros_node_->create_subscription<geometry_msgs::msg::Twist>
        (topic_names_[i], 10, [this, i](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            this->OnTwist(i, msg);
        });

        RCLCPP_INFO(ros_node_->get_logger(),"Subscrito a [%s] para aplicar velocidad en [%s]", topic_names_[i].c_str(), link_names_[i].c_str());
      }
    }

    // Conectar al bucle de simulación
    update_conn_ = event::Events::ConnectWorldUpdateBegin(std::bind(&VelLinksPlugin::OnUpdate, this));
  }

private:
  // Callback parametrizado por índice
  void OnTwist(int idx, const geometry_msgs::msg::Twist::SharedPtr msg) 
  {
    last_lin_[idx] = {msg->linear.x,  msg->linear.y,  msg->linear.z};
    last_ang_[idx] = {msg->angular.x, msg->angular.y, msg->angular.z};
  }

  // En cada tick, reaplicamos todas las velocidades
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

  physics::ModelPtr model_;
  std::shared_ptr<rclcpp::Node> ros_node_;
  event::ConnectionPtr update_conn_;

  std::string link_names_[3];
  std::string topic_names_[3];
  physics::LinkPtr links_[3];

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subs_[3];

  ignition::math::Vector3d last_lin_[3]{{0,0,0},{0,0,0},{0,0,0}};
  ignition::math::Vector3d last_ang_[3]{{0,0,0},{0,0,0},{0,0,0}};
};

GZ_REGISTER_MODEL_PLUGIN(VelLinksPlugin)

} // namespace gazebo
