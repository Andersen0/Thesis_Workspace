#!/bin/bash

# Launch rosnode_launch.sh
ros2 launch rosnode_launch.py &

# Export FLASK_APP
export FLASK_APP=flasknoreload.py

# Run flask run with --no-reload
flask run --no-reload