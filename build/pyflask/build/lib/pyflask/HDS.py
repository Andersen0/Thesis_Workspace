#!usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String

class HumanDetectionSystem(Node):
    def __init__(self):
        super().__init__('human_detection_system')
        self.class_detection_sub = self.create_subscription(String, '/class_detection', self.class_detection_callback, 10)

    def class_detection_callback(self, msg):
        self.latest_detection = msg.data

def main(args=None):
    rclpy.init(args=args)
    human_detection_system = HumanDetectionSystem()
    rclpy.spin(human_detection_system)
    human_detection_system.destroy_node()
    rclpy.shutdown()