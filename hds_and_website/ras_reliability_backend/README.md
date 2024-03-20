# ras_reliability_backend

If this is your first time using this package remember to do the following:

$ sudo apt install ros-<ROS_DISTRO>-rosbridge-server

$ source /opt/ros/<ROS_DISTRO>/setup.bash

To run a test of the code do the following:

1. Build and source the workspace

cd "your workspace"

colcon build --packages-select pyflask

source install/local_setup.bash

2a. Launch the server file by running 

$ ros2 run pyflask server

2b. Launch the node file by running

$ ros2 run pyflask talker

and

$ ros2 run pyflask robot

3_alt. Alternativly run the launch which runs all the nodes by running

$ ros2 launch pyflask my_launch_file.launch.py

4. Open the html site at the given address, http://10.42.80."fill inn":5000/. 

Voila!
