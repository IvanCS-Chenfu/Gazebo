// src/joint_control_plugin.cpp

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace gazebo
{
  class JointControlPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
    {
      model_    = model;
      ros_node_ = gazebo_ros::Node::Get(sdf);

      // --- Parámetros desde SDF/XACRO ---
      joint_name_  = sdf->Get<std::string>("joint_name");
      topic_vel_   = sdf->Get<std::string>("topic_velocity");
      topic_eff_   = sdf->Get<std::string>("topic_effort");
      // Leer max_effort si está, o usar límite URDF
      if (sdf->HasElement("max_effort"))
        max_effort_ = sdf->Get<double>("max_effort");
      else
        max_effort_ = joint_->GetEffortLimit(0);

      // Obtener el puntero al joint
      joint_ = model_->GetJoint(joint_name_);
      if (!joint_) {
        RCLCPP_ERROR(ros_node_->get_logger(),
           "JointControlPlugin: no existe joint [%s]", joint_name_.c_str());
        return;
      }

      // Fijar el límite de esfuerzo para el motor de velocidad
      joint_->SetParam("fmax", 0, max_effort_);

      RCLCPP_INFO(ros_node_->get_logger(),
        "JointControlPlugin cargado: joint=[%s], vel_topic=[%s], effort_topic=[%s], max_effort=%.2f",
        joint_name_.c_str(), topic_vel_.c_str(), topic_eff_.c_str(), max_effort_);

      // --- Suscripciones ROS2 ---
      sub_vel_ = ros_node_->create_subscription<std_msgs::msg::Float64>(
        topic_vel_, 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          last_velocity_ = msg->data;
        });

      sub_eff_ = ros_node_->create_subscription<std_msgs::msg::Float64>(
        topic_eff_, 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          last_effort_ = msg->data;
          RCLCPP_INFO(ros_node_->get_logger(),
            "Esfuerzo recibido en [%s]: %.3f",
            topic_eff_.c_str(), last_effort_);
        });


      // --- Conexión al bucle de simulación ---
      update_conn_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&JointControlPlugin::OnUpdate, this));
    }

  private:
    void OnUpdate()
    {
      // 1) Velocidad: el motor interno intentará seguir 'last_velocity_' 
      //    sin exceder 'max_effort_'
      joint_->SetParam("vel", 0, last_velocity_);

      // 2) Esfuerzo puro: inyecta torque/force directamente
      joint_->SetForce(0, last_effort_);
    }

    // Miembros
    physics::ModelPtr                        model_;
    physics::JointPtr                        joint_;
    std::shared_ptr<rclcpp::Node>            ros_node_;
    event::ConnectionPtr                     update_conn_;

    std::string                              joint_name_;
    std::string                              topic_vel_, topic_eff_;
    double                                   max_effort_{0.0};

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
                                             sub_vel_, sub_eff_;
    double                                   last_velocity_{0.0};
    double                                   last_effort_{0.0};
  };

  GZ_REGISTER_MODEL_PLUGIN(JointControlPlugin)
}
