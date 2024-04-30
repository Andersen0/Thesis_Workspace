#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cstdint>

#include "monitor.h"
#include "monitor.c"

using std::placeholders::_1;

std::int64_t classifier;
std::int64_t distance_to_target;
std::int64_t OpState;
bool alert;
bool halt;
bool slowdown;
bool turnoffUVC;
std::int64_t step_count = 0;

class CopilotRV : public rclcpp::Node {
  public:
    CopilotRV() : Node("copilotrv") {
      classifier_subscription_ = this->create_subscription<std_msgs::msg::Int64>(
        "/sRobotClassifier", 10,
        std::bind(&CopilotRV::classifier_callback, this, _1));

      distance_to_target_subscription_ = this->create_subscription<std_msgs::msg::Int64>(
        "/scan", 10,
        std::bind(&CopilotRV::distance_to_target_callback, this, _1));

      alert_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "/sRobotAlert", 10,
        std::bind(&CopilotRV::alert_callback, this, _1));

      halt_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "/sRobotHalt", 10,
        std::bind(&CopilotRV::halt_callback, this, _1));

      slowdown_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "/sRobotSlowdown", 10,
        std::bind(&CopilotRV::slowdown_callback, this, _1));

      opstate_subscription_ = this->create_subscription<std_msgs::msg::Int64>(
        "/sRobotState", 10,
        std::bind(&CopilotRV::opstate_callback, this, _1));

      turnoffUVC_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "/sRobotTurnoffUVC", 10,
        std::bind(&CopilotRV::turnoffUVC_callback, this, _1));

      handleroperationalstate_0_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "copilot/handleroperationalstate_0", 10);

      handleroperationalstate_1_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "copilot/handleroperationalstate_1", 10);

      handleroperationalstate_2_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "copilot/handleroperationalstate_2", 10);

      handleroperationalstate_3_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "copilot/handleroperationalstate_3", 10);

      handlerdtt_assumption_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "copilot/handlerdtt_assumption", 10);

      handlerstate_req103_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "copilot/handlerstate_req103", 10);

      handlerclassifier_assumption_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "copilot/handlerclassifier_assumption", 10);

      handlerstate_req203_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "copilot/handlerstate_req203", 10);

      handlerstate_req101_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "copilot/handlerstate_req101", 10);

      handlerstate_req201_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "copilot/handlerstate_req201", 10);

      handlerstate_req102_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "copilot/handlerstate_req102", 10);

      handlerstate_req202_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "copilot/handlerstate_req202", 10);

      handlerstate_req000_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "copilot/handlerstate_req000", 10);

    }

    // Report (publish) monitor violations.
    void handleroperationalstate_0() {
      auto output = std_msgs::msg::Empty();
      handleroperationalstate_0_publisher_->publish(output);
    }

    // Report (publish) monitor violations.
    void handleroperationalstate_1() {
      auto output = std_msgs::msg::Empty();
      handleroperationalstate_1_publisher_->publish(output);
    }

    // Report (publish) monitor violations.
    void handleroperationalstate_2() {
      auto output = std_msgs::msg::Empty();
      handleroperationalstate_2_publisher_->publish(output);
    }

    // Report (publish) monitor violations.
    void handleroperationalstate_3() {
      auto output = std_msgs::msg::Empty();
      handleroperationalstate_3_publisher_->publish(output);
    }

    // Report (publish) monitor violations.
    void handlerdtt_assumption() {
      auto output = std_msgs::msg::Empty();
      handlerdtt_assumption_publisher_->publish(output);
    }

    // Report (publish) monitor violations.
    void handlerstate_req103() {
      auto output = std_msgs::msg::Empty();
      handlerstate_req103_publisher_->publish(output);
    }

    // Report (publish) monitor violations.
    void handlerclassifier_assumption() {
      auto output = std_msgs::msg::Empty();
      handlerclassifier_assumption_publisher_->publish(output);
    }

    // Report (publish) monitor violations.
    void handlerstate_req203() {
      auto output = std_msgs::msg::Empty();
      handlerstate_req203_publisher_->publish(output);
    }

    // Report (publish) monitor violations.
    void handlerstate_req101() {
      auto output = std_msgs::msg::Empty();
      handlerstate_req101_publisher_->publish(output);
    }

    // Report (publish) monitor violations.
    void handlerstate_req201() {
      auto output = std_msgs::msg::Empty();
      handlerstate_req201_publisher_->publish(output);
    }

    // Report (publish) monitor violations.
    void handlerstate_req102() {
      auto output = std_msgs::msg::Empty();
      handlerstate_req102_publisher_->publish(output);
    }

    // Report (publish) monitor violations.
    void handlerstate_req202() {
      auto output = std_msgs::msg::Empty();
      handlerstate_req202_publisher_->publish(output);
    }

    // Report (publish) monitor violations.
    void handlerstate_req000() {
      auto output = std_msgs::msg::Empty();
      handlerstate_req000_publisher_->publish(output);
    }

    // Needed so we can report messages to the log.
    static CopilotRV& getInstance() {
      static CopilotRV instance;
      return instance;
    }

  private:
    void classifier_callback(const std_msgs::msg::Int64::SharedPtr msg) const {
      classifier = msg->data;
    }

    void distance_to_target_callback(const std_msgs::msg::Int64::SharedPtr msg) const {
      distance_to_target = msg->data;
    }

    void alert_callback(const std_msgs::msg::Bool::SharedPtr msg) const {
      alert = msg->data;
    }

    void halt_callback(const std_msgs::msg::Bool::SharedPtr msg) const {
      halt = msg->data;
    }

    void slowdown_callback(const std_msgs::msg::Bool::SharedPtr msg) const {
      slowdown = msg->data;
    }

    void opstate_callback(const std_msgs::msg::Int64::SharedPtr msg) const {
      OpState = msg->data;
    }

    void turnoffUVC_callback(const std_msgs::msg::Bool::SharedPtr msg) const {
      turnoffUVC = msg->data;
      step();
      step_count++; // Ensure this line is executed.
      // RCLCPP_INFO(this->get_logger(), "Step: %ld", step_count); // Use format specifier for long.
    }


    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr classifier_subscription_;

    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr distance_to_target_subscription_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alert_subscription_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr halt_subscription_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr slowdown_subscription_;

    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr opstate_subscription_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr turnoffUVC_subscription_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr handleroperationalstate_0_publisher_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr handleroperationalstate_1_publisher_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr handleroperationalstate_2_publisher_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr handleroperationalstate_3_publisher_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr handlerdtt_assumption_publisher_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr handlerstate_req103_publisher_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr handlerclassifier_assumption_publisher_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr handlerstate_req203_publisher_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr handlerstate_req101_publisher_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr handlerstate_req201_publisher_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr handlerstate_req102_publisher_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr handlerstate_req202_publisher_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr handlerstate_req000_publisher_;

};

// Pass monitor violations to the actual class, which has ways to
// communicate with other applications.
void handleroperationalstate_0() {
  CopilotRV::getInstance().handleroperationalstate_0();
}

// Pass monitor violations to the actual class, which has ways to
// communicate with other applications.
void handleroperationalstate_1() {
  CopilotRV::getInstance().handleroperationalstate_1();
}

// Pass monitor violations to the actual class, which has ways to
// communicate with other applications.
void handleroperationalstate_2() {
  CopilotRV::getInstance().handleroperationalstate_2();
}

// Pass monitor violations to the actual class, which has ways to
// communicate with other applications.
void handleroperationalstate_3() {
  CopilotRV::getInstance().handleroperationalstate_3();
}

// Pass monitor violations to the actual class, which has ways to
// communicate with other applications.
void handlerdtt_assumption() {
  CopilotRV::getInstance().handlerdtt_assumption();
}

// Pass monitor violations to the actual class, which has ways to
// communicate with other applications.
void handlerstate_req103() {
  CopilotRV::getInstance().handlerstate_req103();
}

// Pass monitor violations to the actual class, which has ways to
// communicate with other applications.
void handlerclassifier_assumption() {
  CopilotRV::getInstance().handlerclassifier_assumption();
}

// Pass monitor violations to the actual class, which has ways to
// communicate with other applications.
void handlerstate_req203() {
  CopilotRV::getInstance().handlerstate_req203();
}

// Pass monitor violations to the actual class, which has ways to
// communicate with other applications.
void handlerstate_req101() {
  CopilotRV::getInstance().handlerstate_req101();
}

// Pass monitor violations to the actual class, which has ways to
// communicate with other applications.
void handlerstate_req201() {
  CopilotRV::getInstance().handlerstate_req201();
}

// Pass monitor violations to the actual class, which has ways to
// communicate with other applications.
void handlerstate_req102() {
  CopilotRV::getInstance().handlerstate_req102();
}

// Pass monitor violations to the actual class, which has ways to
// communicate with other applications.
void handlerstate_req202() {
  CopilotRV::getInstance().handlerstate_req202();
}

// Pass monitor violations to the actual class, which has ways to
// communicate with other applications.
void handlerstate_req000() {
  CopilotRV::getInstance().handlerstate_req000();
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CopilotRV>());
  rclcpp::shutdown();
  return 0;
}
