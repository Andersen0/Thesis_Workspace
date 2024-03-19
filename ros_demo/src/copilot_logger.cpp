#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/empty.hpp"

using std::placeholders::_1;

class CopilotLogger : public rclcpp::Node {
  public:
    CopilotLogger() : Node("copilotlogger") {
      handlerstate_req101_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "copilot/handlerstate_req101", 10,
        std::bind(&CopilotLogger::handlerstate_req101_callback, this, _1));

      handlerdtt_assumption_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "copilot/handlerDtt_assumption", 10,
        std::bind(&CopilotLogger::handlerdtt_assumption_callback, this, _1));

      handlerstate_req201_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "copilot/handlerstate_req201", 10,
        std::bind(&CopilotLogger::handlerstate_req201_callback, this, _1));

      handlerstate_req202_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "copilot/handlerstate_req202", 10,
        std::bind(&CopilotLogger::handlerstate_req202_callback, this, _1));

      handlerstate_req203_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "copilot/handlerstate_req203", 10,
        std::bind(&CopilotLogger::handlerstate_req203_callback, this, _1));

      handlerstate_req102_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "copilot/handlerstate_req102", 10,
        std::bind(&CopilotLogger::handlerstate_req102_callback, this, _1));

      handlerclassifier_empty_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "copilot/handlerclassifier_empty", 10,
        std::bind(&CopilotLogger::handlerclassifier_empty_callback, this, _1));

      handleroperationalstate_3_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "copilot/handleroperationalstate_3", 10,
        std::bind(&CopilotLogger::handleroperationalstate_3_callback, this, _1));

      handleroperationalstate_0_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "copilot/handleroperationalstate_0", 10,
        std::bind(&CopilotLogger::handleroperationalstate_0_callback, this, _1));

      handleroperationalstate_1_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "copilot/handleroperationalstate_1", 10,
        std::bind(&CopilotLogger::handleroperationalstate_1_callback, this, _1));

      handlerclassifier_assumption_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "copilot/handlerclassifier_assumption", 10,
        std::bind(&CopilotLogger::handlerclassifier_assumption_callback, this, _1));

      handlerstate_req103_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "copilot/handlerstate_req103", 10,
        std::bind(&CopilotLogger::handlerstate_req103_callback, this, _1));

      handleroperationalstate_2_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "copilot/handleroperationalstate_2", 10,
        std::bind(&CopilotLogger::handleroperationalstate_2_callback, this, _1));

    }

  private:
    void handlerstate_req101_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/) const {
      RCLCPP_INFO(this->get_logger(), "Copilot monitor violation: handlerstate_req101");
    }

    void handlerdtt_assumption_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/) const {
      RCLCPP_INFO(this->get_logger(), "Copilot monitor violation: handlerdtt_assumption");
    }

    void handlerstate_req201_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/) const {
      RCLCPP_INFO(this->get_logger(), "Copilot monitor violation: handlerstate_req201");
    }

    void handlerstate_req202_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/) const {
      RCLCPP_INFO(this->get_logger(), "Copilot monitor violation: handlerstate_req202");
    }

    void handlerstate_req203_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/) const {
      RCLCPP_INFO(this->get_logger(), "Copilot monitor violation: handlerstate_req203");
    }

    void handlerstate_req102_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/) const {
      RCLCPP_INFO(this->get_logger(), "Copilot monitor violation: handlerstate_req102");
    }

    void handlerclassifier_empty_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/) const {
      RCLCPP_INFO(this->get_logger(), "Copilot monitor violation: handlerclassifier_empty");
    }

    void handleroperationalstate_3_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/) const {
      RCLCPP_INFO(this->get_logger(), "Copilot monitor violation: handleroperationalstate_3");
    }

    void handleroperationalstate_0_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/) const {
      RCLCPP_INFO(this->get_logger(), "Copilot monitor violation: handleroperationalstate_0");
    }

    void handleroperationalstate_1_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/) const {
      RCLCPP_INFO(this->get_logger(), "Copilot monitor violation: handleroperationalstate_1");
    }

    void handlerclassifier_assumption_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/) const {
      RCLCPP_INFO(this->get_logger(), "Copilot monitor violation: handlerclassifier_assumption");
    }

    void handlerstate_req103_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/) const {
      RCLCPP_INFO(this->get_logger(), "Copilot monitor violation: handlerstate_req103");
    }

    void handleroperationalstate_2_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/) const {
      RCLCPP_INFO(this->get_logger(), "Copilot monitor violation: handleroperationalstate_2");
    }

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr handlerstate_req101_subscription_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr handlerdtt_assumption_subscription_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr handlerstate_req201_subscription_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr handlerstate_req202_subscription_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr handlerstate_req203_subscription_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr handlerstate_req102_subscription_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr handlerclassifier_empty_subscription_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr handleroperationalstate_3_subscription_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr handleroperationalstate_0_subscription_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr handleroperationalstate_1_subscription_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr handlerclassifier_assumption_subscription_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr handlerstate_req103_subscription_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr handleroperationalstate_2_subscription_;

};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CopilotLogger>());
  rclcpp::shutdown();
  return 0;
}
