#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <memory>
#include <string>

namespace gazebo {

class VelJointsPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override 
  {
    model_    = model;
    ros_node_ = gazebo_ros::Node::Get(sdf);

    // Para que sea realista, el joint no puede acelerar con un valor infinito
    if (sdf->HasElement("max_effort"))
    {
      max_effort_ = sdf->Get<double>("max_effort");
    }
    else
    {
      max_effort_ = 2.0;
    }

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
        RCLCPP_ERROR(ros_node_->get_logger(),"VelJointsPlugin: no encuentro joint %s",joint_names_[i].c_str());
      }
      else
      {
        // Aplicar máximo esfuerzo
        joints_[i]->SetParam("fmax", 0, max_effort_);

        // Crear suscripción a cada tópico
        subs_[i] = ros_node_->create_subscription<std_msgs::msg::Float64>
        (topic_names_[i], 10, [this, i](const std_msgs::msg::Float64::SharedPtr msg)
        {
            this->OnTwist(i, msg);
        });

        RCLCPP_INFO(ros_node_->get_logger(),"Subscrito a [%s] para velocidad en joint [%s] (fmax=%.3f)",topic_names_[i].c_str(), joint_names_[i].c_str(), max_effort_);
      }
    }

    // Conectar al bucle de simulación
    update_conn_ = event::Events::ConnectWorldUpdateBegin(std::bind(&VelJointsPlugin::OnUpdate, this));
  }

private:
  // Callback parametrizado por índice
  void OnTwist(int idx, const std_msgs::msg::Float64::SharedPtr msg) 
  {
    last_vel_[idx] = msg->data;
  }

  // En cada tick, reaplicamos todas las velocidades
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

  physics::ModelPtr model_;
  std::shared_ptr<rclcpp::Node> ros_node_;
  event::ConnectionPtr update_conn_;

  std::string joint_names_[3];
  std::string topic_names_[3];
  physics::JointPtr joints_[3];

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subs_[3];

  double last_vel_[3]{0.0, 0.0, 0.0};
  double max_effort_{2.0};
};

GZ_REGISTER_MODEL_PLUGIN(VelJointsPlugin)

} // namespace gazebo
