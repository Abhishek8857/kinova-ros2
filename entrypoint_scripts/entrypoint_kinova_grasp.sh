#!/bin/bash

# Source the ROS2 installation
source /opt/ros/humble/setup.bash

# Source the ROS2 colcon workspace
source /colcon_ws/install/setup.bash

# Source the ROS2 overlay workspace
source /overlay_ws/install/setup.bash

# Run additional commands
ros2 launch kinova_python kinova_grasp.launch.py \
  robot_ip:=192.168.1.10 \
  use_fake_hardware:=false
  # robot_ip:=yyy.yyy.yyy.yyy \
  # use_fake_hardware:=true

