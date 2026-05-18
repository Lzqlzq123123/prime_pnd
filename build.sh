#!/bin/bash
set -e
# Prioritize system Python over uv-managed Python for ROS2 build
export PATH="/usr/bin:$PATH"
source /opt/ros/jazzy/setup.bash
colcon build --packages-skip pteleop_bridge tests_bag