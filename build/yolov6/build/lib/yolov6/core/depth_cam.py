#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32

class DepthCam(Node):
    def __init__(self):
        super().__init__('depth_cam')
        self.depth_im_sub = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.depth_pub = self.create_publisher(Float32, 'depth', 10)
        self.depth_im = None


    def listener_callback(self, msg):
        self.depth_im = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.depth_im = np.array(self.depth_im, dtype=np.float32)
        depth = Float32() 
        depth.data = float(np.mean(self.depth_im))
        self.depth_pub.publish(depth)


def main():
    rclpy.init()
    depth_cam = DepthCam()
    rclpy.spin(depth_cam)
    depth_cam.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
