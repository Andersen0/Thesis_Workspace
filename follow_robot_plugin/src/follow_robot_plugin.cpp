#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <algorithm>
#include <string>

class FollowRobot : public gazebo::ModelPlugin
{
public:
    void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) override
    {
        this->model = _parent;  // Assign the model

        // Get the world and all lights in it
        auto world = this->model->GetWorld();
        std::vector<gazebo::physics::LightPtr> lights = world->Lights();

        // Find the specific light by name
        auto it = std::find_if(lights.begin(), lights.end(), [](const gazebo::physics::LightPtr& light) {
            return light->GetName() == "user_spot_light";
        });

        if (it != lights.end()) {
            this->light = *it;
        } else {
            gzerr << "Light named 'user_spot_light' not found!" << std::endl;
            return;
        }

        // Connect OnUpdate to the simulation update event
        this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&FollowRobot::OnUpdate, this));
    }

    void OnUpdate()
    {
        if (this->light) {
            // Get the current robot pose
            auto robotPose = this->model->WorldPose();

            // Debug output
            gzdbg << "Current Robot Pose: " << robotPose << std::endl;

            // Update the light's position
            robotPose.Pos().Z() += 1.0;  // Adjust height as necessary
            this->light->SetWorldPose(robotPose);

            // More debug output
            gzdbg << "Updated Light Pose: " << robotPose << std::endl;
        } else {
            gzerr << "Light reference not available." << std::endl;
        }
    }

private:
    gazebo::physics::ModelPtr model;
    gazebo::physics::LightPtr light;
    gazebo::event::ConnectionPtr updateConnection;
};

GZ_REGISTER_MODEL_PLUGIN(FollowRobot)
