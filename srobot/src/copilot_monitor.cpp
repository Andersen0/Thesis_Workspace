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
bool alert;
bool halt;
bool slowdown;
bool turnoffUVC;

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

      turnoffUVC_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "/sRobotTurnoffUVC", 10,
        std::bind(&CopilotRV::turnoffUVC_callback, this, _1));

      handlerpropOperationalstate_0_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "copilot/handlerpropOperationalstate_0", 10);

      handlerpropOperationalstate_1_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "copilot/handlerpropOperationalstate_1", 10);

      handlerpropOperationalstate_2_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "copilot/handlerpropOperationalstate_2", 10);

      handlerpropOperationalstate_3_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "copilot/handlerpropOperationalstate_3", 10);

      handlerpropDtt_assumption_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "copilot/handlerpropDtt_assumption", 10);

      handlerpropState_req103_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "copilot/handlerpropState_req103", 10);

      handlerpropClassifier_assumption_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "copilot/handlerpropClassifier_assumption", 10);

      handlerpropState_req203_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "copilot/handlerpropState_req203", 10);

      handlerpropState_req101_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "copilot/handlerpropState_req101", 10);

      handlerpropState_req201_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "copilot/handlerpropState_req201", 10);

      handlerpropState_req102_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "copilot/handlerpropState_req102", 10);

      handlerpropState_req202_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "copilot/handlerpropState_req202", 10);

      handlerpropState_req000_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "copilot/handlerpropState_req000", 10);

    }

    // Report (publish) monitor violations.
    void handlerpropOperationalstate_0() {
      auto output = std_msgs::msg::Empty();
      handlerpropOperationalstate_0_publisher_->publish(output);
    }

    // Report (publish) monitor violations.
    void handlerpropOperationalstate_1() {
      auto output = std_msgs::msg::Empty();
      handlerpropOperationalstate_1_publisher_->publish(output);
    }

    // Report (publish) monitor violations.
    void handlerpropOperationalstate_2() {
      auto output = std_msgs::msg::Empty();
      handlerpropOperationalstate_2_publisher_->publish(output);
    }

    // Report (publish) monitor violations.
    void handlerpropOperationalstate_3() {
      auto output = std_msgs::msg::Empty();
      handlerpropOperationalstate_3_publisher_->publish(output);
    }

    // Report (publish) monitor violations.
    void handlerpropDtt_assumption() {
      auto output = std_msgs::msg::Empty();
      handlerpropDtt_assumption_publisher_->publish(output);
    }

    // Report (publish) monitor violations.
    void handlerpropState_req103() {
      auto output = std_msgs::msg::Empty();
      handlerpropState_req103_publisher_->publish(output);
    }

    // Report (publish) monitor violations.
    void handlerpropClassifier_assumption() {
      auto output = std_msgs::msg::Empty();
      handlerpropClassifier_assumption_publisher_->publish(output);
    }

    // Report (publish) monitor violations.
    void handlerpropState_req203() {
      auto output = std_msgs::msg::Empty();
      handlerpropState_req203_publisher_->publish(output);
    }

    // Report (publish) monitor violations.
    void handlerpropState_req101() {
      auto output = std_msgs::msg::Empty();
      handlerpropState_req101_publisher_->publish(output);
    }

    // Report (publish) monitor violations.
    void handlerpropState_req201() {
      auto output = std_msgs::msg::Empty();
      handlerpropState_req201_publisher_->publish(output);
    }

    // Report (publish) monitor violations.
    void handlerpropState_req102() {
      auto output = std_msgs::msg::Empty();
      handlerpropState_req102_publisher_->publish(output);
    }

    // Report (publish) monitor violations.
    void handlerpropState_req202() {
      auto output = std_msgs::msg::Empty();
      handlerpropState_req202_publisher_->publish(output);
    }

    // Report (publish) monitor violations.
    void handlerpropState_req000() {
      auto output = std_msgs::msg::Empty();
      handlerpropState_req000_publisher_->publish(output);
    }

    // Needed so we can report messages to the log.
    static CopilotRV& getInstance() {
      static CopilotRV instance;
      return instance;
    }

  private:
    void classifier_callback(const std_msgs::msg::Int64::SharedPtr msg) const {
      classifier = msg->data;
      step();
    }

    void distance_to_target_callback(const std_msgs::msg::Int64::SharedPtr msg) const {
      distance_to_target = msg->data;
      step();
    }

    void alert_callback(const std_msgs::msg::Bool::SharedPtr msg) const {
      alert = msg->data;
      step();
    }

    void halt_callback(const std_msgs::msg::Bool::SharedPtr msg) const {
      halt = msg->data;
      step();
    }

    void slowdown_callback(const std_msgs::msg::Bool::SharedPtr msg) const {
      slowdown = msg->data;
      step();
    }

    void turnoffUVC_callback(const std_msgs::msg::Bool::SharedPtr msg) const {
      turnoffUVC = msg->data;
      step();
    }

    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr classifier_subscription_;

    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr distance_to_target_subscription_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alert_subscription_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr halt_subscription_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr slowdown_subscription_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr turnoffUVC_subscription_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr handlerpropOperationalstate_0_publisher_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr handlerpropOperationalstate_1_publisher_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr handlerpropOperationalstate_2_publisher_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr handlerpropOperationalstate_3_publisher_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr handlerpropDtt_assumption_publisher_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr handlerpropState_req103_publisher_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr handlerpropClassifier_assumption_publisher_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr handlerpropState_req203_publisher_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr handlerpropState_req101_publisher_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr handlerpropState_req201_publisher_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr handlerpropState_req102_publisher_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr handlerpropState_req202_publisher_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr handlerpropState_req000_publisher_;

};

// Pass monitor violations to the actual class, which has ways to
// communicate with other applications.
void handlerpropOperationalstate_0() {
  CopilotRV::getInstance().handlerpropOperationalstate_0();
}

// Pass monitor violations to the actual class, which has ways to
// communicate with other applications.
void handlerpropOperationalstate_1() {
  CopilotRV::getInstance().handlerpropOperationalstate_1();
}

// Pass monitor violations to the actual class, which has ways to
// communicate with other applications.
void handlerpropOperationalstate_2() {
  CopilotRV::getInstance().handlerpropOperationalstate_2();
}

// Pass monitor violations to the actual class, which has ways to
// communicate with other applications.
void handlerpropOperationalstate_3() {
  CopilotRV::getInstance().handlerpropOperationalstate_3();
}

// Pass monitor violations to the actual class, which has ways to
// communicate with other applications.
void handlerpropDtt_assumption() {
  CopilotRV::getInstance().handlerpropDtt_assumption();
}

// Pass monitor violations to the actual class, which has ways to
// communicate with other applications.
void handlerpropState_req103() {
  CopilotRV::getInstance().handlerpropState_req103();
}

// Pass monitor violations to the actual class, which has ways to
// communicate with other applications.
void handlerpropClassifier_assumption() {
  CopilotRV::getInstance().handlerpropClassifier_assumption();
}

// Pass monitor violations to the actual class, which has ways to
// communicate with other applications.
void handlerpropState_req203() {
  CopilotRV::getInstance().handlerpropState_req203();
}

// Pass monitor violations to the actual class, which has ways to
// communicate with other applications.
void handlerpropState_req101() {
  CopilotRV::getInstance().handlerpropState_req101();
}

// Pass monitor violations to the actual class, which has ways to
// communicate with other applications.
void handlerpropState_req201() {
  CopilotRV::getInstance().handlerpropState_req201();
}

// Pass monitor violations to the actual class, which has ways to
// communicate with other applications.
void handlerpropState_req102() {
  CopilotRV::getInstance().handlerpropState_req102();
}

// Pass monitor violations to the actual class, which has ways to
// communicate with other applications.
void handlerpropState_req202() {
  CopilotRV::getInstance().handlerpropState_req202();
}

// Pass monitor violations to the actual class, which has ways to
// communicate with other applications.
void handlerpropState_req000() {
  CopilotRV::getInstance().handlerpropState_req000();
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CopilotRV>());
  rclcpp::shutdown();
  return 0;
}
