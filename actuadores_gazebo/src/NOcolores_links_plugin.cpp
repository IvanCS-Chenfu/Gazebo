#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/rendering.hh>
#include <ignition/math/Color.hh>

#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <memory>
#include <string>


/* LO HE DADO POR PERDIDO. NO SE COMO HACER QUE CAMBIE DE COLOR EL LINK */
namespace gazebo {

class ColoresLinksPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_    = model;
    ros_node_ = gazebo_ros::Node::Get(sdf);

    // --- Leer tres pares <link_name, topic_name> del SDF ---
    link_names_[0]  = sdf->Get<std::string>("link_name1");
    link_names_[1]  = sdf->Get<std::string>("link_name2");
    link_names_[2]  = sdf->Get<std::string>("link_name3");

    topic_names_[0] = sdf->Get<std::string>("topic1");
    topic_names_[1] = sdf->Get<std::string>("topic2");
    topic_names_[2] = sdf->Get<std::string>("topic3");

    // --- Obtener punteros a links y crear suscripciones ---
    for (int i = 0; i < 3; ++i)
    {
      links_[i] = model_->GetLink(link_names_[i]);

      if (!links_[i])
      {
        RCLCPP_ERROR(ros_node_->get_logger(), "ColoresLinksPlugin: no encuentro link %s", link_names_[i].c_str());
      }
      else
      {
        // Crear suscripción a cada tópico
        subs_[i] = ros_node_->create_subscription<std_msgs::msg::String>
        (topic_names_[i], 10, [this, i](const std_msgs::msg::String::SharedPtr msg)
        {
          this->OnColor(i, msg);
        });

        RCLCPP_INFO(ros_node_->get_logger(), "Subscrito a [%s] para cambiar color en [%s]", topic_names_[i].c_str(), link_names_[i].c_str());
      }
    }

    // Conectar al bucle de simulación
    prerender_conn_ = event::Events::ConnectPreRender(std::bind(&ColoresLinksPlugin::OnPreRender, this));
  }

private:
  // Callback parametrizado por índice
  void OnColor(int idx, const std_msgs::msg::String::SharedPtr msg)
  {
    std::string in = msg->data;

    // quitar espacios/saltos típicos
    while (!in.empty() && (in.back()=='\n' || in.back()=='\r' || in.back()==' ' || in.back()=='\t'))
      in.pop_back();

    // Mapeo simple
    if (in == "Red")
      last_rgba_[idx] = ignition::math::Color(1,0,0,1);
    else if (in == "Blue")
      last_rgba_[idx] = ignition::math::Color(0,0,1,1);
    else if (in == "Green")
      last_rgba_[idx] = ignition::math::Color(0,1,0,1);
    else
      last_rgba_[idx] = ignition::math::Color(0.5,0.5,0.5,1);
    if (in == "Red")      last_rgba_[idx] = ignition::math::Color(1,0,0,1);
    else if (in == "Green") last_rgba_[idx] = ignition::math::Color(0,1,0,1);
    else if (in == "Blue")  last_rgba_[idx] = ignition::math::Color(0,0,1,1);
    else if (in == "Black") last_rgba_[idx] = ignition::math::Color(0,0,0,1);
    else if (in == "White") last_rgba_[idx] = ignition::math::Color(1,1,1,1);
    else if (in == "Gray" || in == "Grey") last_rgba_[idx] = ignition::math::Color(0.5,0.5,0.5,1);
    else if (in == "Yellow") last_rgba_[idx] = ignition::math::Color(1,1,0,1);
    else {
      // si no reconocemos, gris
      last_rgba_[idx] = ignition::math::Color(0.5,0.5,0.5,1);
    }

    cambiar_color[idx] = true;

    RCLCPP_INFO(ros_node_->get_logger(),
      "ColoresLinksPlugin: pedido color [%s] -> rgba(%.2f,%.2f,%.2f,%.2f) para link [%s]",
      in.c_str(),
      last_rgba_[idx].R(), last_rgba_[idx].G(), last_rgba_[idx].B(), last_rgba_[idx].A(),
      link_names_[idx].c_str());
  }

  void OnPreRender()
  {
    auto scene = gazebo::rendering::get_scene();
    if (!scene || !scene->Initialized()) return;

    const std::string model_prefix = model_->GetName() + "::";

    for (int i = 0; i < 3; ++i)
    {
      if (!links_[i]) continue;
      if (!cambiar_color[i]) continue;

      const std::string model_name = model_->GetName();

      // Visual raíz del modelo
      auto model_visual = scene->GetVisual(model_->GetScopedName());
      if (!model_visual) {
        model_visual = scene->GetVisual(model_->GetName());
      }
      if (!model_visual) return;

      bool applied = false;

      // Recorremos todos los hijos y aplicamos a los que pertenecen a ese link
      ApplyColorRecursive(model_visual, model_name, link_names_[i], last_rgba_[i], applied);

      if (applied)
      {
        cambiar_color[i] = false;
        RCLCPP_INFO(ros_node_->get_logger(),
          "ColorLinksPlugin: aplicado [%s] a visuals de link [%s]",
          last_color_[i].c_str(), link_names_[i].c_str());
      }
      else
      {
        // Seguimos intentando en frames futuros
        RCLCPP_WARN(ros_node_->get_logger(),
          "No aplicado. ModelName='%s' Scoped='%s' Link='%s'. Prueba a inspeccionar nombres de Visuals.",
          model_->GetName().c_str(), model_->GetScopedName().c_str(), link_names_[i].c_str());
      }
    }

    /*
    auto scene = gazebo::rendering::get_scene();

    // La escena puede no estar lista justo al inicio
    if (scene && scene->Initialized())
    {
      const std::string model_name = model_->GetName();

      for (int i = 0; i < 3; ++i)
      {
        if (links_[i] && cambiar_color[i])
        {
          // Nombre típico del Visual del link en Gazebo Classic: "<model>::<link>"
          const std::string visual_name = model_name + "::" + link_names_[i];

          auto visual = scene->GetVisual(visual_name);
          if (visual) 
          {
            visual->SetMaterial(last_color_[i], true);
            cambiar_color[i] = false;
            RCLCPP_WARN(ros_node_->get_logger(), "ColorLinksPlugin: NO encuentro visual [%s]", visual_name.c_str());
          }
          else
          {
            RCLCPP_INFO(ros_node_->get_logger(), "ColoresLinksPlugin: no se ha encontrado el visual");
          }
        }
      }
    }
    */
  }

 bool BelongsToLink(const std::string &visual_name,
                    const std::string &model_name,
                    const std::string &link_name)
  {
    // Cubre casos: model::link, model::link::visual, model::link::visual::...
    const std::string key1 = model_name + "::" + link_name;
    return visual_name.find(key1) != std::string::npos;
  }

  void ApplyColorRecursive(const gazebo::rendering::VisualPtr &v,
                          const std::string &model_name,
                          const std::string &link_name,
                          const ignition::math::Color &c,
                          bool &applied)
  {
    if (!v) return;

    const std::string vn = v->Name();

    if (BelongsToLink(vn, model_name, link_name))
    {
      v->SetDiffuse(c);
      v->SetAmbient(c);
      applied = true;
    }

    for (unsigned int k = 0; k < v->GetChildCount(); ++k)
    {
      auto child = v->GetChild(k);
      auto child_visual = std::dynamic_pointer_cast<gazebo::rendering::Visual>(child);
      if (child_visual)
        ApplyColorRecursive(child_visual, model_name, link_name, c, applied);
    }
  }

  // Miembros
  physics::ModelPtr model_;
  std::shared_ptr<rclcpp::Node> ros_node_;

  std::string link_names_[3];
  std::string topic_names_[3];
  physics::LinkPtr links_[3];

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subs_[3];

  // Estado (3 materiales)
  std::string last_color_[3];
  bool cambiar_color[3]{false, false, false};

  // Evento de render
  event::ConnectionPtr prerender_conn_;


  ignition::math::Color last_rgba_[3]{
    ignition::math::Color(0.5,0.5,0.5,1),
    ignition::math::Color(0.5,0.5,0.5,1),
    ignition::math::Color(0.5,0.5,0.5,1)
  };
};

GZ_REGISTER_MODEL_PLUGIN(ColoresLinksPlugin)

} // namespace gazebo
