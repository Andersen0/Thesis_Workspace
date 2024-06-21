#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <algorithm>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>

class FollowRobot : public gazebo::ModelPlugin {
public:
    FollowRobot() : scanValue_(0.0) {}

    ~FollowRobot() {
        if (ros_node_) {
            rclcpp::shutdown();
        }
    }

    void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) override {
        model = _parent; // Assign the robot model
        auto world = model->GetWorld();

        // Initialize ROS node
        ros_node_ = std::make_shared<rclcpp::Node>("follow_robot_plugin");

        // Retrieve other models by name
        spotLight = world->LightByName("user_spot_light");
        greenZoneModel = world->ModelByName("green_zone_model");
        yellowZoneModel = world->ModelByName("yellow_zone_model");
        redZoneModel = world->ModelByName("red_zone_model");
        blackCylinderModel = world->ModelByName("black_cylinder_model");

        // Setup ROS communications
        sub_ = ros_node_->create_subscription<std_msgs::msg::Float64>(
            "/distance_float", 10, [this](std_msgs::msg::Float64::SharedPtr msg) {
                OnScanMsg(msg);
            });

        updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
            [this](const gazebo::common::UpdateInfo & /*info*/) {
                OnUpdate();
            });
    }

private:
    void OnUpdate() {
        if (!model) return;

        auto robotPose = model->WorldPose();

        // Update the positions of the models to follow the robot
        UpdateModelPose(robotPose, greenZoneModel);
        UpdateModelPose(robotPose, yellowZoneModel);
        UpdateModelPose(robotPose, redZoneModel);

        // Update the black cylinder's position based on the /distance_float value
        UpdateCylinderPose(robotPose);

        // Update the Spot Light's position to follow the robot's entire pose (x, y, z, roll, pitch, yaw)
        UpdateLightPose(robotPose);

        rclcpp::spin_some(ros_node_);
    }

    void UpdateModelPose(const ignition::math::Pose3d& robotPose, const gazebo::physics::ModelPtr& model) {
        if (!model) return;
        auto currentModelPose = model->WorldPose();
        ignition::math::Pose3d newPose(
            robotPose.Pos().X(), robotPose.Pos().Y(), currentModelPose.Pos().Z(),
            currentModelPose.Rot().Roll(), currentModelPose.Rot().Pitch(), currentModelPose.Rot().Yaw());
        model->SetWorldPose(newPose);
    }

    void UpdateCylinderPose(const ignition::math::Pose3d& robotPose) {
        if (!blackCylinderModel) return;
        auto currentCylinderPose = blackCylinderModel->WorldPose();
        double xPositionOffset;

        // Check if the scan value is 0.0 and set the x position far away
        if (scanValue_ == 0.0) {
            xPositionOffset = 1000.0; // You can adjust this value to be "far away" as needed
        } else {
            // Add 0.2 to the scan value to adjust the cylinder's position
            xPositionOffset = static_cast<double>(scanValue_) + 0.2;
        }

        ignition::math::Pose3d newPose(
            robotPose.Pos().X() + xPositionOffset, // Adjust X position based on xPositionOffset
            robotPose.Pos().Y(), 
            currentCylinderPose.Pos().Z(),
            currentCylinderPose.Rot().Roll(), 
            currentCylinderPose.Rot().Pitch(), 
            currentCylinderPose.Rot().Yaw());
        blackCylinderModel->SetWorldPose(newPose);
    }

    void UpdateLightPose(const ignition::math::Pose3d& robotPose) {
        if (!spotLight) return;
        auto currentLightPose = spotLight->WorldPose();
        ignition::math::Pose3d newPose(
            robotPose.Pos().X(), robotPose.Pos().Y(), robotPose.Pos().Z() + 1.125, // Adjust Z position
            currentLightPose.Rot().Roll(), currentLightPose.Rot().Pitch(), currentLightPose.Rot().Yaw());
        spotLight->SetWorldPose(newPose);
    }

    void OnScanMsg(const std_msgs::msg::Float64::SharedPtr msg) {
        scanValue_ = msg->data; // Update the scan value used to adjust the cylinder's position
    }

    gazebo::physics::ModelPtr model;
    gazebo::physics::ModelPtr greenZoneModel;
    gazebo::physics::ModelPtr yellowZoneModel;
    gazebo::physics::ModelPtr redZoneModel;
    gazebo::physics::ModelPtr blackCylinderModel;
    gazebo::physics::LightPtr spotLight;
    gazebo::event::ConnectionPtr updateConnection;
    std::shared_ptr<rclcpp::Node> ros_node_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
    double scanValue_;
};

GZ_REGISTER_MODEL_PLUGIN(FollowRobot)
