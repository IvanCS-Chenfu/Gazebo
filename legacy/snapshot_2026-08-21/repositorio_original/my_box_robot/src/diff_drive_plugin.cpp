#include "my_box_robot/diff_drive_plugin.hpp"
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace gazebo
{

DiffDrivePlugin::DiffDrivePlugin() {}

void DiffDrivePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  model_ = model;
  auto nh = gazebo_ros::Node::Get(sdf);

  ros_node_ = nh;

  wheel_radius_ = sdf->Get<double>("wheel_radius", 0.05).first;
  wheel_separation_ = sdf->Get<double>("wheel_separation", 0.26).first;

  left_wheel_joint_ = model_->GetJoint("left_wheel_joint");
  right_wheel_joint_ = model_->GetJoint("right_wheel_joint");

  if (!left_wheel_joint_ || !right_wheel_joint_)
  {
    RCLCPP_ERROR(ros_node_->get_logger(), "No se encontraron las articulaciones de las ruedas");
    return;
  }

  cmd_vel_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&DiffDrivePlugin::OnCmdVel, this, std::placeholders::_1)
  );

  RCLCPP_INFO(ros_node_->get_logger(), "Plugin DiffDrive cargado y escuchando /cmd_vel");
}

void DiffDrivePlugin::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  RCLCPP_INFO(ros_node_->get_logger(), "Recibido cmd_vel: linear.x = %.2f angular.z = %.2f",
              msg->linear.x, msg->angular.z);

  double v = msg->linear.x;
  double w = msg->angular.z;

  double v_left = (v - w * wheel_separation_ / 2.0) / wheel_radius_;
  double v_right = (v + w * wheel_separation_ / 2.0) / wheel_radius_;

  // Activar control de esfuerzo y velocidad
  left_wheel_joint_->SetParam("fmax", 0, 10.0);
  right_wheel_joint_->SetParam("fmax", 0, 10.0);

  left_wheel_joint_->SetParam("vel", 0, v_left);
  right_wheel_joint_->SetParam("vel", 0, v_right);
}

GZ_REGISTER_MODEL_PLUGIN(DiffDrivePlugin)
}
