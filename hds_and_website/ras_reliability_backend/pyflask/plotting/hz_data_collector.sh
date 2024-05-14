#!/bin/bash

# Start the srobot_realistic_output node using ros2 run
echo "Starting fixed_publisher node..."
ros2 run srobot_realistic_output fixed_publisher &
fixed_publisher_pid=$!
sleep 4  # Wait 3 seconds for initialization

# Run ROS2 topic command to monitor standalone rates for 5 minutes (300 seconds)
echo "Collecting standalone rates..."
timeout 180s ros2 topic hz /sRobotTurnoffUVC > standalone_rates_5000hz.txt
sleep 2  # Wait 3 seconds before starting the next node

# Start the copilot node using ros2 run
echo "Starting copilot node..."
ros2 run copilot copilot &
copilot_pid=$!
sleep 5  # Wait 3 seconds for initialization

# Run ROS2 topic command to monitor rates with monitor for 5 minutes (300 seconds)
echo "Collecting monitor rates..."
timeout 180s ros2 topic hz /sRobotTurnoffUVC > monitor_rates_5000hz.txt


# Kill the two running nodes by their process IDs
# echo "Stopping fixed_publisher and copilot nodes..."
# killall fixed_publisher
# killall copilot

echo "Data collection complete. Please stop nodes manually. "
