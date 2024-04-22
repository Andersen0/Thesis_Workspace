#!/bin/bash

# Define your nodes and workspace
COPILOT_NODE="copilot"
ROS2_WS_PATH="$PWD"

echo "ROS2_WS_PATH: $ROS2_WS_PATH"

# Source your ROS2 workspace
source $ROS2_WS_PATH/install/setup.bash

# Get time before killing nodes

before_kill_time=$(date +%s.%N)

# Stop any running instances of the copilot node
killall $COPILOT_NODE

start_time=$(date +%s.%N)

# Start the copilot node
ros2 run copilot copilot &

# Loop until the copilotrv node is detected
while ! ros2 node list | grep -q 'copilotrv'; do
  sleep 0.01
done

# Get end time after copilotrv appears
end_time=$(date +%s.%N)

# Calculate the total duration
total_duration=$(echo "$end_time - $start_time" | bc)

echo "Time until copilotrv is ready: $total_duration seconds"
