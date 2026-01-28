#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <memory>
#include <string>

namespace gazebo {
class FuerzasJointsPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override 
  {
    model_    = model;
    ros_node_ = gazebo_ros::Node::Get(sdf);

    // --- Leer tres pares <joint_name, topic_name> del SDF ---
    joint_names_[0] = sdf->Get<std::string>("joint_name1");
    joint_names_[1] = sdf->Get<std::string>("joint_name2");
    joint_names_[2] = sdf->Get<std::string>("joint_name3");

    topic_names_[0] = sdf->Get<std::string>("topic1");
    topic_names_[1] = sdf->Get<std::string>("topic2");
    topic_names_[2] = sdf->Get<std::string>("topic3");

    // --- Obtener punteros a links y crear suscripciones ---
    for (int i = 0; i < 3; ++i)
    {
      joints_[i] = model_->GetJoint(joint_names_[i]);

      if (!joints_[i])
      {
        RCLCPP_ERROR(ros_node_->get_logger(), "FuerzasJointsPlugin: no encuentro joint %s", joint_names_[i].c_str());
      }
      else
      {
        // Crear suscripción a cada tópico
        subs_[i] = ros_node_->create_subscription<std_msgs::msg::Float64>
        (topic_names_[i], 10, [this, i](const std_msgs::msg::Float64::SharedPtr msg)
        {
            this->OnEffort(i, msg);
        });

        RCLCPP_INFO(
        ros_node_->get_logger(), "Subscrito a [%s] para aplicar effort en joint [%s]", topic_names_[i].c_str(), joint_names_[i].c_str());
      }
    }

    // Conectar al bucle de simulación
    update_conn_ = event::Events::ConnectWorldUpdateBegin(std::bind(&FuerzasJointsPlugin::OnUpdate, this));
  }

private:
  // Callback parametrizado por índice
  void OnEffort(int idx, const std_msgs::msg::Float64::SharedPtr msg)
  {
    last_effort_[idx] = msg->data;
  }


  // En cada tick, reaplicamos todas las fuerzas/torques
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

  // Miembros
  physics::ModelPtr model_;
  std::shared_ptr<rclcpp::Node> ros_node_;
  event::ConnectionPtr update_conn_;

  std::string joint_names_[3];
  std::string topic_names_[3];

  physics::JointPtr joints_[3];
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subs_[3];

  double last_effort_[3]{0.0, 0.0, 0.0};
};

GZ_REGISTER_MODEL_PLUGIN(FuerzasJointsPlugin)
}  // namespace gazebo
