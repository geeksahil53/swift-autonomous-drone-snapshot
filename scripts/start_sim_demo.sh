#!/bin/bash
# Demo launch script for Swift Autonomous Drone
# This script launches a simplified simulation using placeholder nodes

echo "Starting Swift Autonomous Drone Demo..."

# Source ROS 2 setup
source /opt/ros/humble/setup.bash 2>/dev/null || \
source /opt/ros/foxy/setup.bash 2>/dev/null || \
echo "Warning: ROS 2 setup not found. Please source manually."

# Source workspace if built
if [ -f install/setup.bash ]; then
    source install/setup.bash
fi

# Launch demo nodes
echo "Launching demo nodes..."
echo "Note: This is a simplified demo. Full simulation requires Gazebo and WhyCon setup."

# Example launch command (adjust based on actual launch files)
# ros2 launch swift_pico task_1b.launch.py

echo "Demo started. Use Ctrl+C to stop."

