#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace gazebo
{
  class ToggleLightPlugin : public ModelPlugin
  {
    private: physics::ModelPtr model;
    private: sdf::ElementPtr sdf;
    private: transport::NodePtr node;
    private: transport::PublisherPtr lightPub;
    private: rclcpp::Node::SharedPtr rosNode;
    private: rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub;
    private: bool lightOn;
    private: std::string lightName;

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      this->model = _model;
      this->sdf = _sdf;

      gzdbg << "ToggleLightPlugin loading..." << std::endl;

      if (this->sdf->HasElement("light_name"))
      {
        this->lightName = this->sdf->Get<std::string>("light_name");
      }
      else
      {
        gzerr << "ToggleLightPlugin missing <light_name> parameter, defaulting to 'user_spot_light'.\n";
        this->lightName = "user_spot_light";
      }

      gzdbg << "Light name: " << this->lightName << std::endl;

      // Initialize ROS 2 context
      if (!rclcpp::ok())
      {
        rclcpp::init(0, nullptr);
      }

      // Initialize ROS 2 node
      this->rosNode = rclcpp::Node::make_shared("toggle_light_plugin");
      gzdbg << "ROS 2 node initialized." << std::endl;

      // Subscribe to the ROS 2 topic
      this->sub = this->rosNode->create_subscription<std_msgs::msg::Bool>(
        "/sRobotTurnoffUVC", 10, std::bind(&ToggleLightPlugin::OnRosMsg, this, std::placeholders::_1));

      gzdbg << "Subscribed to ROS 2 topic /sRobotTurnoffUVC." << std::endl;

      // Create the node and connect to the Gazebo topic
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(this->model->GetWorld()->Name());

      this->lightPub = this->node->Advertise<msgs::Light>("~/light/modify");
      gzdbg << "Advertising on Gazebo transport topic '~/light/modify'." << std::endl;

      this->lightOn = true;  // Assuming the light is on initially

      // Set initial light state
      this->ToggleLight(this->lightName, this->lightOn);

      // Start the ROS 2 spinning thread
      auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor->add_node(this->rosNode);
      std::thread([executor]() { executor->spin(); }).detach();

      gzdbg << "ToggleLightPlugin loaded successfully." << std::endl;
    }

    public: void OnRosMsg(const std_msgs::msg::Bool::SharedPtr msg)
    {
      gzdbg << "Received ROS message: " << msg->data << std::endl;
      // Invert the logic
      this->lightOn = !msg->data;
      this->ToggleLight(this->lightName, this->lightOn);
    }

    private: void ToggleLight(const std::string &lightName, bool turnOn)
    {
      gzdbg << "Toggling light: " << lightName << " to " << (turnOn ? "on" : "off") << std::endl;

      msgs::Light lightMsg;
      lightMsg.set_name(lightName);
      lightMsg.set_range(30);
      lightMsg.set_attenuation_linear(0.01);
      lightMsg.set_attenuation_quadratic(0.001);
      lightMsg.set_cast_shadows(false);

      if (turnOn)
      {
        auto diffuse = lightMsg.mutable_diffuse();
        diffuse->set_r(0.5);
        diffuse->set_g(1.5);
        diffuse->set_b(0.5);
        diffuse->set_a(1.0);

        auto specular = lightMsg.mutable_specular();
        specular->set_r(0.1);
        specular->set_g(0.1);
        specular->set_b(0.1);
        specular->set_a(1.0);
      }
      else
      {
        auto diffuse = lightMsg.mutable_diffuse();
        diffuse->set_r(0.0);
        diffuse->set_g(0.0);
        diffuse->set_b(0.0);
        diffuse->set_a(0.0);

        auto specular = lightMsg.mutable_specular();
        specular->set_r(0.0);
        specular->set_g(0.0);
        specular->set_b(0.0);
        specular->set_a(0.0);
      }

      gzdbg << "Publishing light modification message." << std::endl;
      this->lightPub->Publish(lightMsg);
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(ToggleLightPlugin)
}
