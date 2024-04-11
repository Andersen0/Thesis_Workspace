#!/bin/bash

# Define your nodes and workspace
COPILOT_NODE="copilot"
ROS2_WS_PATH="$PWD"

printf "ROS2_WS_PATH: $ROS2_WS_PATH\n"

# Source your ROS2 workspace
source $ROS2_WS_PATH/install/setup.bash

# Stop the nodes
killall $COPILOT_NODE &

# Wait a bit for shutdown to complete
# sleep 0.5

# Restart the nodes
ros2 run copilot copilot
