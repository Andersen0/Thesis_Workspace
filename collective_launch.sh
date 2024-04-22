#!/bin/bash

# Run camera and inferer

ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true

ros2 run yolov6 inferer

# Run copilot

ros2 run copilot copilot

# Run subscriber