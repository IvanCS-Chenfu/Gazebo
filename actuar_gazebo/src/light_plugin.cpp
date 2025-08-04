#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace gazebo
{
class LedColorPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    this->model_ = model;

    auto led_link_name = sdf->Get<std::string>("led_link", "led_link").first;
    link_ = model_->GetLink(led_link_name);

    if (!link_)
    {
      gzerr << "Link '" << led_link_name << "' not found.\n";
      return;
    }

    // Get Gazebo ROS node
    ros_node_ = gazebo_ros::Node::Get(sdf);

    if (!ros_node_)
    {
    std::cerr << "[LedColorPlugin] ERROR: Failed to get gazebo_ros node.\n";
    return;
    }

    // Get the visual name from link
    rendering::ScenePtr scene = rendering::get_scene();
    if (!scene || !scene->Initialized())
    {
      gzdbg << "Waiting for rendering scene to initialize...\n";
      this->update_connection_ = event::Events::ConnectPreRender(
        std::bind(&LedColorPlugin::TryInitVisual, this));
    }
    else
    {
      TryInitVisual();
    }

    // ROS2 subscriber
    color_sub_ = ros_node_->create_subscription<std_msgs::msg::ColorRGBA>(
      "/led_color", 10,
      std::bind(&LedColorPlugin::OnColorMsg, this, std::placeholders::_1));

    RCLCPP_INFO(ros_node_->get_logger(), "LedColorPlugin loaded.");
  }

private:
  void TryInitVisual()
    {
    if (visual_) return;  // Ya fue inicializado

    auto scene = rendering::get_scene();
    if (!scene || !scene->Initialized())
        return;

    std::string full_name = model_->GetName() + "::" + link_->GetName();

    visual_ = scene->GetVisual(full_name);
    if (!visual_)
    {
        gzdbg << "Visual " << full_name << " not yet available.\n";
        return;
    }

    gzdbg << "Visual " << full_name << " initialized.\n";
    }


  void OnColorMsg(const std_msgs::msg::ColorRGBA::SharedPtr msg)
  {
    if (!visual_)
      return;

    ignition::math::Color c(msg->r, msg->g, msg->b, msg->a);
    visual_->SetDiffuse(c);
    visual_->SetAmbient(c);
  }

  physics::ModelPtr model_;
  physics::LinkPtr link_;
  rendering::VisualPtr visual_;

  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr color_sub_;

  event::ConnectionPtr update_connection_;
};

GZ_REGISTER_MODEL_PLUGIN(LedColorPlugin)
}
