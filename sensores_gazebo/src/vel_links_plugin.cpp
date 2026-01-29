#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <gazebo/common/UpdateInfo.hh>
#include <builtin_interfaces/msg/time.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <memory>
#include <string>

namespace gazebo {

class VelLinksPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_    = model;
    ros_node_ = gazebo_ros::Node::Get(sdf);

    // Leer parámetros
    link_names_[0] = sdf->Get<std::string>("link_name1");
    link_names_[1] = sdf->Get<std::string>("link_name2");
    link_names_[2] = sdf->Get<std::string>("link_name3");

    topic_names_[0] = sdf->Get<std::string>("topic1");
    topic_names_[1] = sdf->Get<std::string>("topic2");
    topic_names_[2] = sdf->Get<std::string>("topic3");


    if (sdf->HasElement("publish_rate"))
    {
        publish_rate_ = sdf->Get<double>("publish_rate");
    }
    else
    {
        publish_rate_ = 50.0;
    }

    if (sdf->HasElement("frame_id"))
    {
        frame_id_ = sdf->Get<std::string>("frame_id");
    }
    else
    {
        frame_id_ = "world";
    }
    if (sdf->HasElement("frame_propio"))
    {
        frame_propio_ = sdf->Get<bool>("frame_propio");
    }
    else
    {
        frame_propio_ = true;
    }

    // Obtener links + crear publishers
    for (int i = 0; i < 3; ++i)
    {
      links_[i] = model_->GetLink(link_names_[i]);

      if (!links_[i])
      {
        RCLCPP_ERROR(ros_node_->get_logger(),"VelLinksPlugin: no encuentro link %s",link_names_[i].c_str());
      }
      else
      {
        pubs_[i] = ros_node_->create_publisher<geometry_msgs::msg::TwistStamped>(topic_names_[i], 10);

        RCLCPP_INFO(ros_node_->get_logger(),"Publicando velocidad de [%s] en [%s] (rate=%.1f Hz)", link_names_[i].c_str(), topic_names_[i].c_str(), publish_rate_);
      }
    }

    last_sim_time_ = gazebo::common::Time(0, 0);

    // Conectar al bucle de simulación
    update_conn_ = event::Events::ConnectWorldUpdateBegin(std::bind(&VelLinksPlugin::OnUpdate, this, std::placeholders::_1));
  }

private:
  void OnUpdate(const gazebo::common::UpdateInfo& info)
  {  
    // Publicar segun el "publish_rate" deseado
    const gazebo::common::Time sim_time = info.simTime;
    bool enviar = true;
    
    if (publish_rate_ > 0.0)
    {
      enviar = false;
      const double dt = (sim_time - last_sim_time_).Double();
      if (dt >= (1.0 / publish_rate_))
      {
          enviar = true;
          last_sim_time_ = sim_time;
      }
    }
    



    if (enviar)
    {
      // Para pasar al header del mensaje
      builtin_interfaces::msg::Time stamp;
      stamp.sec = static_cast<int32_t>(sim_time.sec);
      stamp.nanosec = static_cast<uint32_t>(sim_time.nsec);

      for (int i = 0; i < 3; ++i)
      {
        if (links_[i] && pubs_[i])
        {   
          auto v_lin = links_[i]->WorldLinearVel();   // ignition::math::Vector3d
          auto v_ang = links_[i]->WorldAngularVel();  // ignition::math::Vector3d
          
          
          if (frame_propio_)
          {
            const auto pose = links_[i]->WorldPose();
            const auto R = pose.Rot();  // world -> link

            v_lin  = R.RotateVectorReverse(v_lin);
            v_ang  = R.RotateVectorReverse(v_ang);

            frame_id_ = link_names_[i];
          }

          geometry_msgs::msg::TwistStamped msg;
          msg.header.stamp = stamp;
          msg.header.frame_id = frame_id_;

          msg.twist.linear.x  = v_lin.X();
          msg.twist.linear.y  = v_lin.Y();
          msg.twist.linear.z  = v_lin.Z();

          msg.twist.angular.x = v_ang.X();
          msg.twist.angular.y = v_ang.Y();
          msg.twist.angular.z = v_ang.Z();

          pubs_[i]->publish(msg);
        }
      }
    }
  }

  physics::ModelPtr model_;
  std::shared_ptr<rclcpp::Node> ros_node_;
  event::ConnectionPtr update_conn_;

  std::string link_names_[3];
  std::string topic_names_[3];
  physics::LinkPtr links_[3];

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pubs_[3];

  double publish_rate_{50.0};
  std::string frame_id_{"world"};
  bool frame_propio_{true};

  gazebo::common::Time last_sim_time_;
};

GZ_REGISTER_MODEL_PLUGIN(VelLinksPlugin)

}  // namespace gazebo
