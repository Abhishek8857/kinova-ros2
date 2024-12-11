# Use the specified ROS 2 Humble desktop image as the base
FROM osrf/ros:humble-desktop

# Source the workspace
RUN echo "source /colcon_ws/install/setup.bash" >> ~/.bashrc

# Replace /bin/sh with /bin/bash
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Set the ROS Domain ID
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Add the Ignition Gazebo repository and install ignition-gazebo6
# RUN apt-get update && apt-get install -y \
#     wget \
#     lsb-release \
#     gnupg \
#     build-essential \
#     cmake 

# Install required dependencies
RUN apt-get update && apt-get install -y \
    wget \
    lsb-release \
    gnupg \
    build-essential \
    cmake \
    ament-cmake \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-kortex-bringup \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-ros-gz-bridge \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-rviz2 \
    software-properties-common \
    libignition-transport11-dev \
    libgflags-dev \
    ros-humble-kinematics-interface-kdl \
    ros-humble-test-msgs \
    libcap-dev \
    ros-humble-kinova-gen3-7dof-robotiq-2f-85-moveit-config \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install ignotion gazebo
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
RUN apt-get update && apt-get install -y libignition-gazebo6 libignition-gazebo6-dev

RUN apt-get update --fix-missing -y

# Install additional required binaries for the 7-DoF robot
RUN apt-get install -y ros-humble-kinova-gen3-7dof-robotiq-2f-85-moveit-config

# Fix missing update
RUN apt-get update --fix-missing -y

# Copy the entire colcon_ws  and overlay_ws directory with the submodule into the Docker image
COPY colcon_ws/ /colcon_ws
# COPY overlay_ws/ /overlay_ws/

WORKDIR /colcon_ws/src/

# Update package lists and import MoveIt repositories based on the specified ROS distribution
RUN apt-get update && \
    for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do \
        vcs import < "$repo"; \
    done && \
    rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

WORKDIR /colcon_ws/

# Add RMW implementation to the bashrc file for persistent sourcing
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# Import additional repositories using vcs
RUN rosdep update
RUN sudo apt-get update
RUN rosdep install --from-paths src --ignore-src -r -y

# RUN vcs import src --skip-existing --input https://raw.githubusercontent.com/ros-controls/ros2_control_ci/master/ros_controls.$ROS_DISTRO.repos
RUN vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex.humble.repos
RUN vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex-not-released.humble.repos
RUN vcs import src --skip-existing --input src/ros2_kortex/simulation.humble.repos

# Build the workspace with resource management
RUN source /opt/ros/humble/setup.bash && \
    MAKEFLAGS="-j4 -l2" colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 3 --symlink-install --executor sequential

# Copy cyclonedds config files
COPY cyclonedds/config.xml /config.xml
COPY cyclonedds/10-cyclone-max.conf /etc/sysctl.d/10-cyclone-max.conf

# Copy entrypoint scripts and make them executable
COPY entrypoint_scripts/ /entrypoint_scripts/
RUN chmod +x /entrypoint_scripts/*.sh

# Copy contents in overlay ws
COPY overlay_ws/ /overlay_ws/
WORKDIR /overlay_ws/

RUN apt-get update && apt-get upgrade -y
RUN rosdep install --from-paths src --ignore-src -r -y

RUN source /colcon_ws/install/setup.bash && \
    colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
# Set the entrypoint script (modify the entrypoint script as needed)
# ENTRYPOINT ["/entrypoint_scripts/entrypoint_base_node.sh"]
