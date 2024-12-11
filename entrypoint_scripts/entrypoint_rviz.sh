#!/bin/bash

# Source the ROS2 installation
source /opt/ros/humble/setup.bash

# Source the ROS2 colcon workspace
source /colcon_ws/install/setup.bash

# Source the ROS2 overlay workspace
source /overlay_ws/install/setup.bash

# Run additional commands
rviz2 -d ${RVIZ_CONFIG}
