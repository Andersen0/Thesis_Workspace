#!/usr/bin/env python3

from geometry_msgs.msg import Twist
import rclpy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from rclpy.node import Node
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, DurabilityPolicy

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(Joy, 'gamepad', self.joy_callback, 10)
        self.service = self.create_client(Trigger, '~/button1_trigger')
        self.twist = Twist()
        self.last_button = False
        self.halt_value = False
        self.slowdown_value = False
        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)
        self.halt_subber = self.create_subscription(Bool, '/sRobotHalt', self.halt_callback, qos_profile)
        self.slowdown_subber = self.create_subscription(Bool, '/sRobotSlowdown', self.slowdown_callback, qos_profile)
        
    def slowdown_callback(self, msg):
        self.slowdown_value = msg.data

    def halt_callback(self, msg):
        self.halt_value = msg.data

    def joy_callback(self, msg):

        if self.halt_value:
            self.twist.linear.x = 0.0 * msg.axes[0]
            self.twist.angular.z = -0.0 * msg.axes[3]
        elif self.slowdown_value:
            self.twist.linear.x = 0.35/2 * msg.axes[0]
            self.twist.angular.z = -0.45/2 * msg.axes[3]
        else:
            self.twist.linear.x = (0.7/2) * msg.axes[0]
            self.twist.angular.z = (-0.9/2) * msg.axes[3]

        if msg.buttons[0]:
            if not self.last_button:
                self.service.call_async(Trigger.Request())
        self.last_button = msg.buttons[0]

        self.pub.publish(self.twist)

def main():
    rclpy.init()
    teleop = Teleop()
    rclpy.spin(teleop)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
