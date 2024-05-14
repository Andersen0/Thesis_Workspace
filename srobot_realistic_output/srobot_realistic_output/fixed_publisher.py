import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int64
from rclpy.qos import QoSPresetProfiles
import datetime

class FixedPublisher(Node):
    def __init__(self):
        super().__init__('fixed_publisher_node')
        qos_profile = QoSPresetProfiles.SYSTEM_DEFAULT.value

        self.state_publisher = self.create_publisher(Int64, '/sRobotState', qos_profile)
        self.slowdown_publisher = self.create_publisher(Bool, '/sRobotSlowdown', qos_profile)
        self.halt_publisher = self.create_publisher(Bool, '/sRobotHalt', qos_profile)
        self.alert_publisher = self.create_publisher(Bool, '/sRobotAlert', qos_profile)
        self.turnoff_uvc_publisher = self.create_publisher(Bool, '/sRobotTurnoffUVC', qos_profile)
        self.classifier_publisher = self.create_publisher(Int64, '/sRobotClassifier', qos_profile)
        self.distance_publisher = self.create_publisher(Int64, '/scan', qos_profile)

        self.timer_frequency = 714 # Hz (you can adjust this based on your required publishing frequency)
        # 714 for 5000Hz, 1428 for 10000Hz, 1786 for 12500hz, 2143 for 15000hz, 2500 for 17500hz, 2857 for 20000hz due to seven topics being published per increment
        self.timer_period = 1/self.timer_frequency  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_messages)
        # self.increment = 0

    def publish_messages(self):
        classifier_msg = Int64(data=0)
        self.classifier_publisher.publish(classifier_msg)

        distance_msg = Int64(data=0)
        self.distance_publisher.publish(distance_msg)

        state_msg = Int64(data=0)
        self.state_publisher.publish(state_msg)

        slowdown_msg = Bool(data=False)
        self.slowdown_publisher.publish(slowdown_msg)

        halt_msg = Bool(data=True)
        self.halt_publisher.publish(halt_msg)

        alert_msg = Bool(data=False)
        self.alert_publisher.publish(alert_msg)

        turnoff_uvc_msg = Bool(data=False)
        self.turnoff_uvc_publisher.publish(turnoff_uvc_msg)

        """
        # Get timepoint for finished publishing of all messages
        now = self.get_clock().now()
        now_time = now.to_msg()
        dt = datetime.datetime.fromtimestamp(now_time.sec + now_time.nanosec / 1e9)
        formatted_time = dt.strftime('"%Y-%m-%d %H:%M:%S.%f",')
        print(formatted_time)
        """

        # self.increment += 1

        # Print the messages in one line for debugging
        # self.get_logger().info(f"Classifier={classifier_msg.data}, distance={distance_msg.data}, state={state_msg.data} ,published messages: slowdown={slowdown_msg.data}, halt={halt_msg.data}, alert={alert_msg.data}, turnoff_uvc={turnoff_uvc_msg.data}, increment={self.increment}")

def main(args=None):
    rclpy.init(args=args)
    fixed_publisher = FixedPublisher()
    rclpy.spin(fixed_publisher)
    fixed_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
