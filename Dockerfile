#################### DOCKER FILE WITHOUT MOVEIT ######################
# # Use the specified ROS 2 Humble desktop image as the base
# FROM osrf/ros:humble-desktop

# # Replace /bin/sh with /bin/bash
# RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# # Add the Ignition Gazebo repository and install ignition-gazebo6
# RUN apt-get update && apt-get install -y \
#     wget \
#     lsb-release \
#     gnupg
# RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-stable.list'
# RUN wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
# RUN apt-get update && apt-get install -y libignition-gazebo6 libignition-gazebo6-dev


# # Install dependencies with apt
# RUN apt-get update && apt-get install -y ament-cmake
# RUN apt-get install -y python3-pip
# RUN apt-get install -y python3-colcon-common-extensions
# RUN apt-get install -y python3-vcstool
# RUN apt-get install -y ros-humble-joint-state-publisher-gui
# RUN apt-get install -y ros-humble-rmw-cyclonedds-cpp
# RUN apt-get install -y ros-humble-kortex-bringup
# RUN apt-get install -y ros-humble-ros2-control
# RUN apt-get install -y ros-humble-ros2-controllers
# RUN apt-get install -y ros-humble-moveit
# RUN apt-get install -y ros-humble-test-msgs
# RUN apt-get install -y python3-vcstool
# RUN apt-get install -y ros-humble-xacro
# RUN apt-get install -y ros-humble-ros-gz-bridge
# RUN apt-get install -y ros-humble-gazebo-ros-pkgs
# RUN apt-get install -y ros-humble-gazebo-ros2-control
# RUN apt-get install -y ros-humble-rviz2
# RUN apt-get install -y software-properties-common
# RUN apt-get install -y libignition-transport11-dev
# RUN apt-get install -y libgflags-dev
# RUN apt-get install -y ros-humble-kinematics-interface-kdl


# # Install additional required binaries for the 7-DoF robot
# RUN apt-get install -y ros-humble-kinova-gen3-7dof-robotiq-2f-85-moveit-config

# # Fix missing update
# RUN apt-get update --fix-missing

# # Fix rviz2 black screen issue
# #RUN add-apt-repository ppa:kisak/kisak-mesa && \
# #    apt-get -y update && \
# #    apt-get -y upgrade

# # Copy the entire colcon_ws directory with the submodule into the Docker image
# COPY colcon_ws/ /colcon_ws/

# # Install module dependencies for colcon_ws
# WORKDIR /colcon_ws/
# RUN rosdep update
# RUN rosdep install --from-paths src --ignore-src -r -y

# # Import additional repositories using vcs
# RUN vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex.humble.repos
# RUN vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex-not-released.humble.repos
# RUN vcs import src --skip-existing --input src/ros2_kortex/simulation.humble.repos

# # Build the workspace with resource management
# RUN source /opt/ros/humble/setup.bash && \
#     MAKEFLAGS="-j4 -l2" colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 3 --symlink-install

# # Source the workspace (Optional)
# RUN echo "source /colcon_ws/install/setup.bash" >> ~/.bashrc

# # Source ROS 2 setup script and build colcon_ws
# #RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

# # Copy cyclonedds config files
# COPY cyclonedds/config.xml /config.xml
# COPY cyclonedds/10-cyclone-max.conf /etc/sysctl.d/10-cyclone-max.conf

# # Copy entrypoint scripts and make them executable
# COPY entrypoint_scripts/ /entrypoint_scripts/
# RUN chmod +x /entrypoint_scripts/*.sh

# # Set the entrypoint script (modify the entrypoint script as needed)
# #ENTRYPOINT ["/entrypoint_scripts/entrypoint_base_node.sh"]




###################### DOCKER FILE WITH MOVEIT ######################



# # Use the specified ROS 2 Humble desktop image as the base
# FROM osrf/ros:humble-desktop

# # Source the workspace
# RUN echo "source /colcon_ws/install/setup.bash" >> ~/.bashrc

# # Replace /bin/sh with /bin/bash
# RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# # Set the ROS Domain ID
# ENV ROS_DOMAIN_ID=0
# ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# # Install required dependencies
# RUN apt-get update && apt-get install -y \
#     wget \
#     lsb-release \
#     gnupg \
#     build-essential \
#     cmake \
#     ament-cmake \
#     python3-pip \
#     python3-colcon-common-extensions \
#     python3-vcstool \
#     ros-humble-joint-state-publisher-gui \
#     ros-humble-rmw-cyclonedds-cpp \
#     ros-humble-kortex-bringup \
#     ros-humble-ros2-control \
#     ros-humble-ros2-controllers \
#     ros-humble-xacro \
#     ros-humble-ros-gz-bridge \
#     ros-humble-gazebo-ros-pkgs \
#     ros-humble-gazebo-ros2-control \
#     ros-humble-rviz2 \
#     software-properties-common \
#     libignition-transport11-dev \
#     libgflags-dev \
#     ros-humble-kinematics-interface-kdl \
#     ros-humble-test-msgs \
#     libcap-dev \
#     ros-humble-kinova-gen3-7dof-robotiq-2f-85-moveit-config \
#     ros-humble-kinova-gen3-6dof-robotiq-2f-85-moveit-config \
#     && apt-get clean && rm -rf /var/lib/apt/lists/*
          

# # Install igntion gazebo
# RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-stable.list'
# RUN wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
# RUN apt-get update && apt-get install -y libignition-gazebo6 libignition-gazebo6-dev

# RUN apt-get update --fix-missing -y


# # Fix missing update
# RUN apt-get update --fix-missing -y

# # Copy the entire colcon_ws  and overlay_ws directory with the submodule into the Docker image
# COPY colcon_ws/ /colcon_ws
# # COPY overlay_ws/ /overlay_ws/

# WORKDIR /colcon_ws/src/

# # Update package lists and import MoveIt repositories based on the specified ROS distribution
# RUN apt-get update && \
#     for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do \
#         vcs import < "$repo"; \
#     done && \
#     rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

# WORKDIR /colcon_ws/

# # Add RMW implementation to the bashrc file for persistent sourcing
# RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# # Import additional repositories using vcs
# RUN rosdep update
# RUN sudo apt-get update
# RUN rosdep install --from-paths . src --ignore-src -r -y

# # RUN vcs import src --skip-existing --input https://raw.githubusercontent.com/ros-controls/ros2_control_ci/master/ros_controls.$ROS_DISTRO.repos
# RUN vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex.humble.repos
# RUN vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex-not-released.humble.repos
# RUN vcs import src --skip-existing --input src/ros2_kortex/simulation.humble.repos

# # Build the workspace with resource management
# RUN source /opt/ros/humble/setup.bash && \
#     MAKEFLAGS="-j4 -l2" colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 3 --symlink-install --executor sequential

# # Copy cyclonedds config files
# COPY cyclonedds/config.xml /config.xml
# COPY cyclonedds/10-cyclone-max.conf /etc/sysctl.d/10-cyclone-max.conf

# # Copy entrypoint scripts and make them executable
# COPY entrypoint_scripts/ /entrypoint_scripts/
# RUN chmod +x /entrypoint_scripts/*.sh
# RUN apt-get update && apt-get upgrade -y

# # Copy contents in overlay ws
# COPY overlay_ws/ /overlay_ws/
# WORKDIR /overlay_ws/

# RUN rosdep install --from-paths src --ignore-src -r -y

# RUN source /colcon_ws/install/setup.bash && \
#     colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release

# Set the entrypoint script (modify the entrypoint script as needed)
# ENTRYPOINT ["/entrypoint_scripts/entrypoint_base_node.sh"]



####################### DOCKER FILE WITH MOVEIT AND KORTEX VISION ######################



# Use the specified ROS 2 Humble desktop image as the base
ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop

# Source the workspace
RUN echo "source /colcon_ws/install/setup.bash" >> ~/.bashrc

# Replace /bin/sh with /bin/bash
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Set the ROS Domain ID
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

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
    ros-humble-depth-image-proc \
    gstreamer1.0-tools \
    gstreamer1.0-libav \    
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-base \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-good1.0-dev \
    software-properties-common \
    libignition-transport11-dev \
    libgflags-dev \
    ros-humble-kinematics-interface-kdl \
    ros-humble-test-msgs \
    libcap-dev \
    ros-humble-kinova-gen3-7dof-robotiq-2f-85-moveit-config \
    ros-humble-kinova-gen3-6dof-robotiq-2f-85-moveit-config \
    && apt-get clean && rm -rf /var/lib/apt/lists/*
          
    
# Install igntion gazebo
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
RUN apt-get update && apt-get install -y libignition-gazebo6 libignition-gazebo6-dev

# Fix missing update
RUN apt-get update --fix-missing -y

# Copy the entire colcon_ws  and overlay_ws directory with the submodule into the Docker image
COPY colcon_ws/ /colcon_ws/
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


# RUN vcs import src --skip-existing --input https://raw.githubusercontent.com/ros-controls/ros2_control_ci/master/ros_controls.humble.repos
# RUN vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex.humble.repos
# RUN vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex-not-released.humble.repos
# RUN vcs import src --skip-existing --input src/ros2_kortex/simulation.humble.repos

RUN vcs import src --skip-existing --input src/required.repos

RUN rosdep update
RUN sudo apt-get update
RUN rosdep install --from-paths . src --ignore-src -r -y

# Build the workspace with resource management
RUN source /opt/ros/humble/setup.bash && \
    MAKEFLAGS="-j4 -l2" colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 3 --symlink-install --executor sequential

# Copy cyclonedds config files
COPY cyclonedds/config.xml /config.xml
COPY cyclonedds/10-cyclone-max.conf /etc/sysctl.d/10-cyclone-max.conf

# Copy entrypoint scripts and make them executable
COPY entrypoint_scripts/ /entrypoint_scripts/
RUN chmod +x /entrypoint_scripts/*.sh
RUN apt-get update && apt-get upgrade -y

# Copy contents in overlay ws
COPY overlay_ws/ /overlay_ws/
WORKDIR /overlay_ws/

RUN rosdep install --from-paths src --ignore-src -r -y

RUN source /colcon_ws/install/setup.bash && \
    colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release

# Set the entrypoint script (modify the entrypoint script as needed)
# ENTRYPOINT ["/entrypoint_scripts/entrypoint_base_node.sh"]
