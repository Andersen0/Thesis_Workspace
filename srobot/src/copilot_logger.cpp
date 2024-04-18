#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/empty.hpp"

using std::placeholders::_1;

class CopilotLogger : public rclcpp::Node {
  public:
    CopilotLogger() : Node("copilotlogger") {
      handlerpropOperationalstate_0_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "copilot/handlerpropOperationalstate_0", 10,
        std::bind(&CopilotLogger::handlerpropOperationalstate_0_callback, this, _1));

      handlerpropOperationalstate_1_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "copilot/handlerpropOperationalstate_1", 10,
        std::bind(&CopilotLogger::handlerpropOperationalstate_1_callback, this, _1));

      handlerpropOperationalstate_2_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "copilot/handlerpropOperationalstate_2", 10,
        std::bind(&CopilotLogger::handlerpropOperationalstate_2_callback, this, _1));

      handlerpropOperationalstate_3_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "copilot/handlerpropOperationalstate_3", 10,
        std::bind(&CopilotLogger::handlerpropOperationalstate_3_callback, this, _1));

      handlerpropDtt_assumption_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "copilot/handlerpropDtt_assumption", 10,
        std::bind(&CopilotLogger::handlerpropDtt_assumption_callback, this, _1));

      handlerpropState_req103_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "copilot/handlerpropState_req103", 10,
        std::bind(&CopilotLogger::handlerpropState_req103_callback, this, _1));

      handlerpropClassifier_assumption_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "copilot/handlerpropClassifier_assumption", 10,
        std::bind(&CopilotLogger::handlerpropClassifier_assumption_callback, this, _1));

      handlerpropState_req203_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "copilot/handlerpropState_req203", 10,
        std::bind(&CopilotLogger::handlerpropState_req203_callback, this, _1));

      handlerpropState_req101_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "copilot/handlerpropState_req101", 10,
        std::bind(&CopilotLogger::handlerpropState_req101_callback, this, _1));

      handlerpropState_req201_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "copilot/handlerpropState_req201", 10,
        std::bind(&CopilotLogger::handlerpropState_req201_callback, this, _1));

      handlerpropState_req102_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "copilot/handlerpropState_req102", 10,
        std::bind(&CopilotLogger::handlerpropState_req102_callback, this, _1));

      handlerpropState_req202_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "copilot/handlerpropState_req202", 10,
        std::bind(&CopilotLogger::handlerpropState_req202_callback, this, _1));

      handlerpropState_req000_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "copilot/handlerpropState_req000", 10,
        std::bind(&CopilotLogger::handlerpropState_req000_callback, this, _1));

    }

  private:
    void handlerpropOperationalstate_0_callback(const std_msgs::msg::Empty::SharedPtr msg) const {
      RCLCPP_INFO(this->get_logger(), "Copilot monitor violation: handlerpropOperationalstate_0");
    }

    void handlerpropOperationalstate_1_callback(const std_msgs::msg::Empty::SharedPtr msg) const {
      RCLCPP_INFO(this->get_logger(), "Copilot monitor violation: handlerpropOperationalstate_1");
    }

    void handlerpropOperationalstate_2_callback(const std_msgs::msg::Empty::SharedPtr msg) const {
      RCLCPP_INFO(this->get_logger(), "Copilot monitor violation: handlerpropOperationalstate_2");
    }

    void handlerpropOperationalstate_3_callback(const std_msgs::msg::Empty::SharedPtr msg) const {
      RCLCPP_INFO(this->get_logger(), "Copilot monitor violation: handlerpropOperationalstate_3");
    }

    void handlerpropDtt_assumption_callback(const std_msgs::msg::Empty::SharedPtr msg) const {
      RCLCPP_INFO(this->get_logger(), "Copilot monitor violation: handlerpropDtt_assumption");
    }

    void handlerpropState_req103_callback(const std_msgs::msg::Empty::SharedPtr msg) const {
      RCLCPP_INFO(this->get_logger(), "Copilot monitor violation: handlerpropState_req103");
    }

    void handlerpropClassifier_assumption_callback(const std_msgs::msg::Empty::SharedPtr msg) const {
      RCLCPP_INFO(this->get_logger(), "Copilot monitor violation: handlerpropClassifier_assumption");
    }

    void handlerpropState_req203_callback(const std_msgs::msg::Empty::SharedPtr msg) const {
      RCLCPP_INFO(this->get_logger(), "Copilot monitor violation: handlerpropState_req203");
    }

    void handlerpropState_req101_callback(const std_msgs::msg::Empty::SharedPtr msg) const {
      RCLCPP_INFO(this->get_logger(), "Copilot monitor violation: handlerpropState_req101");
    }

    void handlerpropState_req201_callback(const std_msgs::msg::Empty::SharedPtr msg) const {
      RCLCPP_INFO(this->get_logger(), "Copilot monitor violation: handlerpropState_req201");
    }

    void handlerpropState_req102_callback(const std_msgs::msg::Empty::SharedPtr msg) const {
      RCLCPP_INFO(this->get_logger(), "Copilot monitor violation: handlerpropState_req102");
    }

    void handlerpropState_req202_callback(const std_msgs::msg::Empty::SharedPtr msg) const {
      RCLCPP_INFO(this->get_logger(), "Copilot monitor violation: handlerpropState_req202");
    }

    void handlerpropState_req000_callback(const std_msgs::msg::Empty::SharedPtr msg) const {
      RCLCPP_INFO(this->get_logger(), "Copilot monitor violation: handlerpropState_req000");
    }

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr handlerpropOperationalstate_0_subscription_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr handlerpropOperationalstate_1_subscription_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr handlerpropOperationalstate_2_subscription_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr handlerpropOperationalstate_3_subscription_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr handlerpropDtt_assumption_subscription_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr handlerpropState_req103_subscription_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr handlerpropClassifier_assumption_subscription_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr handlerpropState_req203_subscription_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr handlerpropState_req101_subscription_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr handlerpropState_req201_subscription_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr handlerpropState_req102_subscription_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr handlerpropState_req202_subscription_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr handlerpropState_req000_subscription_;

};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CopilotLogger>());
  rclcpp::shutdown();
  return 0;
}
