#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <gazebo/common/UpdateInfo.hh>
#include <builtin_interfaces/msg/time.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <memory>
#include <string>

namespace gazebo {

class AccJointsPlugin : public ModelPlugin {
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

    if (sdf->HasElement("publish_rate"))
    {
      publish_rate_ = sdf->Get<double>("publish_rate");
    }
    else
    {
      publish_rate_ = 50.0;
    }


    // --- Obtener punteros a links y crear suscripciones ---
    for (int i = 0; i < 3; ++i)
    {
      joints_[i] = model_->GetJoint(joint_names_[i]);

      if (!joints_[i]) 
      {
        RCLCPP_ERROR(ros_node_->get_logger(),"AccJointsPlugin: no encuentro joint %s",joint_names_[i].c_str());
      }
      else
      {
        pubs_[i] = ros_node_->create_publisher<std_msgs::msg::Float64>(topic_names_[i], 10);
        prev_vel_[i] = joints_[i]->GetVelocity(0);
        has_prev_[i] = false;  // publicaremos la 2ª vez con derivada válida

        RCLCPP_INFO(ros_node_->get_logger(), "Publicando aceleración de joint [%s] en [%s] (rate=%.1f Hz)", joint_names_[i].c_str(), topic_names_[i].c_str(), publish_rate_);
      }
    }

    last_sim_time_ = gazebo::common::Time(0, 0);
    prev_time_     = gazebo::common::Time(0, 0);
    
    // Conectar al bucle de simulación
    update_conn_ = event::Events::ConnectWorldUpdateBegin(std::bind(&AccJointsPlugin::OnUpdate, this, std::placeholders::_1));
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
      // Tiempo para derivada (desde última publicación)
      const double dt = (sim_time - prev_time_).Double();
      
      for (int i = 0; i < 3; ++i)
      {
        if (joints_[i] && pubs_[i])
        {   
          const double vel = joints_[i]->GetVelocity(0);  // rad/s o m/s

          // Si es la primera vez (o dt muy pequeño), no publicamos derivada
          if (has_prev_[i] && dt > 1e-9)
          {
            const double acc = (vel - prev_vel_[i]) / dt;  // rad/s^2 o m/s^2

            prev_vel_[i] = vel;

            std_msgs::msg::Float64 msg;
            msg.data = acc;
            pubs_[i]->publish(msg);
          }
          else
          {
            prev_vel_[i] = vel;
            has_prev_[i] = true;
          }
        }
      }
      prev_time_ = sim_time;
    }
  }

  physics::ModelPtr model_;
  std::shared_ptr<rclcpp::Node> ros_node_;
  event::ConnectionPtr update_conn_;

  std::string joint_names_[3];
  std::string topic_names_[3];
  physics::JointPtr joints_[3];

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubs_[3];

  double publish_rate_{50.0};

  gazebo::common::Time last_sim_time_;
  gazebo::common::Time prev_time_;

  double prev_vel_[3]{0.0, 0.0, 0.0};
  bool has_prev_[3]{false, false, false};
};

GZ_REGISTER_MODEL_PLUGIN(AccJointsPlugin)

} // namespace gazebo
