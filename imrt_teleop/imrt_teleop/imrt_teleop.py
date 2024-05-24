#!/usr/bin/env python3

from geometry_msgs.msg import Twist
import rclpy
from sensor_msgs.msg import Joy
from rclpy.node import Node

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(Joy, 'gamepad', self.joy_callback, 10)
        self.twist = Twist()

    def joy_callback(self, msg):
        self.twist.linear.x = 0.7*msg.axes[0]
        self.twist.angular.z = -0.9*msg.axes[3]

        self.pub.publish(self.twist)



def main():
    rclpy.init()
    teleop = Teleop()
    rclpy.spin(teleop)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
