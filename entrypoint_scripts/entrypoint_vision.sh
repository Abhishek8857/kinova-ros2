#!/bin/bash

# Source the ROS2 installation
source /opt/ros/humble/setup.bash

# Source the ROS2 colcon workspace
source /colcon_ws/install/setup.bash

# Source the ROS2 overlay workspace
source /overlay_ws/install/setup.bash

# Run additional commands
ros2 launch kinova_vision kinova_vision.launch.py \
    depth_registration:=true \
    launch_depth:=true \
    max_color_pub_rate:=1.0 \
    max_depth_pub_rate:=5.0
