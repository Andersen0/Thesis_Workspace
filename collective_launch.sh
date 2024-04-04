#!/bin/bash

# Launch rosnode_launch.sh in the background
ros2 launch copilot rosnode_launch.py &

# Save the current directory
CURRENT_DIR=$(pwd)

# Navigate to the directory containing the Flask application
cd hds_and_website/ras_reliability_backend/pyflask/

# Export FLASK_APP with the path to flasknoreload.py
export FLASK_APP=$(pwd)/flasknoreload.py

# Run Flask application
flask run --no-reload

# Optionally, navigate back to the original directory
cd "$CURRENT_DIR"
