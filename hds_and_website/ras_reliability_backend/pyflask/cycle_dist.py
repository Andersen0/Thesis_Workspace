#!usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class CycleDist(Node):
    def __init__(self):
        super().__init__('cycle_dist')
        self.zone_pub = self.create_publisher(String, 'detection_zone', 10)
        state_period = 3
        self.state = self.create_timer(state_period, self.state_callback)
         # Randomly change the robot state   
        self.walk_list = ['none', 'green', 'yellow', 'red', 'yellow', 'green']
        self.i = 0
        
    def state_callback(self):

        walk_len = len(self.walk_list)


        msg = String()

        # Prepare a string message for the 'status' topic       
        msg.data = self.walk_list[self.i % walk_len]
        self.i += 1

        # Publish the message
        self.zone_pub.publish(msg)

        # Log the message
        self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args) # initialize ROS2

    # Create an instance of the MinimalRobotPublisher class
    zone_cycle = CycleDist()

    # Spin the node so the callback function is called.
    rclpy.spin(zone_cycle)
    
    # Destroy the node
    # clean up and terminate the ROS2 program
    zone_cycle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
