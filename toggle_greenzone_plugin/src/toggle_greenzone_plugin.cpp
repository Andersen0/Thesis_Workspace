#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/common/Events.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>

namespace gazebo
{
  class ToggleGreenZoneModel : public ModelPlugin
  {
    private:
      physics::ModelPtr model;
      rclcpp::Node::SharedPtr rosNode;
      rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub;
      event::ConnectionPtr updateConnection;
      bool layerVisible = false;  // Initialize visibility
      std::string modelName;
      int visualLayer;  // Layer ID assigned to the visual

    public:
      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
      {
        this->model = _model;
        this->visualLayer = 1;  // Set this to the layer ID your visual is assigned to

        if (_sdf->HasElement("model_name")) {
          this->modelName = _sdf->Get<std::string>("model_name");
          gzdbg << "Model name set to: " << this->modelName << std::endl;
        } else {
          this->modelName = "green_zone_model";  // Default model name
          gzdbg << "Model name not provided, defaulting to 'green_zone_model'." << std::endl;
        }

        if (!rclcpp::ok())
          rclcpp::init(0, nullptr);

        this->rosNode = rclcpp::Node::make_shared("toggle_greenzone_plugin");
        this->sub = this->rosNode->create_subscription<std_msgs::msg::Int64>(
          "/scan", 10, std::bind(&ToggleGreenZoneModel::OnRosMsg, this, std::placeholders::_1));
        gzdbg << "Subscribed to ROS 2 topic /scan." << std::endl;

        auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor->add_node(this->rosNode);
        std::thread([executor]() { executor->spin(); }).detach();
        gzdbg << "ROS node spinning started in a separate thread." << std::endl;

        // Connect to the world update event
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ToggleGreenZoneModel::OnUpdate, this));
      }

      void OnUpdate()
      {
        rendering::ScenePtr scene = rendering::get_scene();
        if (!scene || !scene->Initialized())
          return;

        // Set the visibility of the visual layer
        auto visual = scene->GetVisual(this->modelName);
        if (visual) {
          visual->SetVisible(this->layerVisible);
        }
      }

      void OnRosMsg(const std_msgs::msg::Int64::SharedPtr msg)
      {
        if (msg->data >= 3 && msg->data <= 7) {
          this->layerVisible = true;
          gzdbg << "Layer visibility set to visible." << std::endl;
        } else {
          this->layerVisible = false;
          gzdbg << "Layer visibility set to hidden." << std::endl;
        }
      }
  };

  GZ_REGISTER_MODEL_PLUGIN(ToggleGreenZoneModel)
}
