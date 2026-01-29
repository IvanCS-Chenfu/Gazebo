#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <gazebo/common/UpdateInfo.hh>
#include <builtin_interfaces/msg/time.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>

#include <memory>
#include <string>

namespace gazebo {

class AccLinksPlugin : public ModelPlugin {
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
        RCLCPP_ERROR(ros_node_->get_logger(),"AccLinksPlugin: no encuentro link %s",link_names_[i].c_str());
      }
      else
      {
        pubs_[i] = ros_node_->create_publisher<geometry_msgs::msg::AccelStamped>(topic_names_[i], 10);

        // Inicializar prev con velocidad actual para evitar “picos” al arrancar
        prev_v_lin_[i] = links_[i]->WorldLinearVel();
        prev_v_ang_[i] = links_[i]->WorldAngularVel();
        has_prev_[i]   = false;  // publicaremos la 2ª vez con derivada válida

        RCLCPP_INFO(ros_node_->get_logger(),"Publicando aceleración de [%s] en [%s] (rate=%.1f Hz)", link_names_[i].c_str(), topic_names_[i].c_str(), publish_rate_);
      }
    }

    last_sim_time_ = gazebo::common::Time(0, 0);
    prev_time_     = gazebo::common::Time(0, 0);

    // Conectar al bucle de simulación
    update_conn_ = event::Events::ConnectWorldUpdateBegin(std::bind(&AccLinksPlugin::OnUpdate, this, std::placeholders::_1));
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
      const double dt_env = (sim_time - last_sim_time_).Double();
      if (dt_env >= (1.0 / publish_rate_))
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
        if (links_[i] && pubs_[i])
        {   
          auto v_lin = links_[i]->WorldLinearVel();   // ignition::math::Vector3d
          auto v_ang = links_[i]->WorldAngularVel();  // ignition::math::Vector3d
          
          // Si es la primera vez (o dt muy pequeño), no publicamos derivada
          if (has_prev_[i] && dt >1e-9)
          {
            // Aceleración en world (derivada numérica)
            ignition::math::Vector3d a_lin = (v_lin - prev_v_lin_[i]) / dt;
            ignition::math::Vector3d a_ang = (v_ang - prev_v_ang_[i]) / dt;

            // Actualizar prev para la próxima
            prev_v_lin_[i] = v_lin;
            prev_v_ang_[i] = v_ang;

            if (frame_propio_)
            {
              const auto pose = links_[i]->WorldPose();
              const auto R = pose.Rot();  // world -> link

              a_lin  = R.RotateVectorReverse(a_lin);
              a_ang  = R.RotateVectorReverse(a_ang);

              frame_id_ = link_names_[i];
            }

            geometry_msgs::msg::AccelStamped msg;
            msg.header.stamp = stamp;
            msg.header.frame_id = frame_id_;

            msg.accel.linear.x  = a_lin.X();
            msg.accel.linear.y  = a_lin.Y();
            msg.accel.linear.z  = a_lin.Z();

            msg.accel.angular.x = a_ang.X();
            msg.accel.angular.y = a_ang.Y();
            msg.accel.angular.z = a_ang.Z();

            pubs_[i]->publish(msg);

          }
          else
          {
            prev_v_lin_[i] = v_lin;
            prev_v_ang_[i] = v_ang;
            has_prev_[i]   = true;
          }
        }
      }
      prev_time_ = sim_time;
    }
  }

  physics::ModelPtr model_;
  std::shared_ptr<rclcpp::Node> ros_node_;
  event::ConnectionPtr update_conn_;

  std::string link_names_[3];
  std::string topic_names_[3];
  physics::LinkPtr links_[3];

  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr pubs_[3];

  double publish_rate_{50.0};
  std::string frame_id_{"world"};
  bool frame_propio_{true};

  gazebo::common::Time last_sim_time_;
  gazebo::common::Time prev_time_;

  ignition::math::Vector3d prev_v_lin_[3]{{0,0,0},{0,0,0},{0,0,0}};
  ignition::math::Vector3d prev_v_ang_[3]{{0,0,0},{0,0,0},{0,0,0}};
  bool has_prev_[3]{false,false,false};
};

GZ_REGISTER_MODEL_PLUGIN(AccLinksPlugin)

}  // namespace gazebo
