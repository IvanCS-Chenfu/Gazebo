#ifndef DIFF_DRIVE_PLUGIN_HPP
#define DIFF_DRIVE_PLUGIN_HPP

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace gazebo
{
class DiffDrivePlugin : public ModelPlugin
{
public:
  DiffDrivePlugin();
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);

  gazebo::physics::ModelPtr model_;
  gazebo::physics::JointPtr left_wheel_joint_;
  gazebo::physics::JointPtr right_wheel_joint_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  double wheel_radius_;
  double wheel_separation_;
};
}

#endif
