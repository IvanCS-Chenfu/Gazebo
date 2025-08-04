#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>

namespace gazebo {
class WrenchPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    ros_node = gazebo_ros::Node::Get(sdf);

    // Coge el link
    link_name_ = sdf->Get<std::string>("link_name");
    link_ = model_->GetLink(link_name_);
    if (!link_) {
      RCLCPP_ERROR(ros_node->get_logger(), "No encuentro link %s", link_name_.c_str());
      return;
    }

    // Subscripción a Wrench
    sub_ = ros_node->create_subscription<geometry_msgs::msg::Wrench>(
      "apply_wrench", 10,
      std::bind(&WrenchPlugin::OnWrench, this, std::placeholders::_1));

    // Conectar al bucle de actualización de Gazebo
    update_conn_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&WrenchPlugin::OnUpdate, this));

    RCLCPP_INFO(ros_node->get_logger(), "WrenchPlugin [%s] listo", link_name_.c_str());
  }

private:
  void OnWrench(const geometry_msgs::msg::Wrench::SharedPtr msg) {
    RCLCPP_INFO(ros_node->get_logger(), "Wrench recibido: F=(%.1f,%.1f,%.1f)  T=(%.1f,%.1f,%.1f)",
        msg->force.x, msg->force.y, msg->force.z,
        msg->torque.x, msg->torque.y, msg->torque.z);
    last_force_  = {msg->force.x,  msg->force.y,  msg->force.z};
    last_torque_ = {msg->torque.x, msg->torque.y, msg->torque.z};
}


  void OnUpdate() {
    // Reaplicamos en cada paso de física
    //link_->AddForce(last_force_);
    //link_->AddTorque(last_torque_);

    //link->AddForceAtWorldPosition( F, world_pos );
    //link->AddRelativeForceAtRelativePosition( F, rel_pos );

    link_->AddRelativeForce(last_force_);   //Sobre el centro de masas
    link_->AddRelativeTorque(last_torque_); //Sobre el centro de masas
  }

  physics::ModelPtr model_;
  physics::LinkPtr  link_;
  std::string       link_name_;
  ignition::math::Vector3d last_force_{0,0,0}, last_torque_{0,0,0};
  std::shared_ptr<rclcpp::Node>     ros_node; 
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr sub_;
  event::ConnectionPtr update_conn_;
};

GZ_REGISTER_MODEL_PLUGIN(WrenchPlugin)
}
