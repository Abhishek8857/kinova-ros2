ARG ROS_DISTRO=humble 
FROM osrf/ros:${ROS_DISTRO}-desktop

# Source the workspace
RUN echo "source /colcon_ws/install/setup.bash" >> ~/.bashrc

# Use bash for all RUN commands after this line
SHELL ["/bin/bash", "-c"]

# Set the ROS Domain ID and Middleware
ENV ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 

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
    ros-dev-tools \
    # Dependancies for ros2_kortex
    ros-${ROS_DISTRO}-kortex-bringup \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-kinova-gen3-7dof-robotiq-2f-85-moveit-config \
    ros-${ROS_DISTRO}-topic-based-ros2-control \
    # Dependancies for ros2_kortex_vision
    gstreamer1.0-tools\
    gstreamer1.0-libav \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-good1.0-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-base \
    # clear all the cache and index files
    && apt-get clean && rm -rf /var/lib/apt/lists/*
          
# Fix missing update
RUN apt-get update --fix-missing -y

# Copy the entire colcon_ws  and overlay_ws directory with the submodule into the Docker image
COPY colcon_ws/ /colcon_ws/
WORKDIR /colcon_ws/

# Import additional required Repositories
RUN vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex.${ROS_DISTRO}.repos
RUN vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex-not-released.${ROS_DISTRO}.repos
RUN vcs import src --skip-existing --input src/ros2_kortex/simulation.humble.repos

# Install Gazebo fortress
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \
    apt-get update && sudo apt-get install -y ignition-fortress

# Update package lists and import MoveIt repositories based on the specified ROS distribution
RUN apt-get update && \
    for repo in src/moveit2/moveit2.repos $(f="moveit2/moveit2_${ROS_DISTRO}.repos"; test -r $f && echo $f); do \
        vcs import < "$repo"; \
    done 

# Install package dependancies
RUN rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}  -r -y

# Args for build
ARG LOW_MEMORY=false
ARG MAKEFLAGS="-j4 -l3"
ARG PARALLEL_WORKERS=3

# Build the workspace with resource management
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    if [ "${LOW_MEMORY}" = true ]; then \
        echo "Low RAM build "; \
        export MAKEFLAGS=${MAKEFLAGS} && \
        colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install --parallel-workers ${PARALLEL_WORKERS}; \
    else \
        echo "Normal build"; \
        colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install; \
    fi


# Copy contents in overlay ws
COPY overlay_ws/ /overlay_ws/
WORKDIR /overlay_ws/

# RUN rosdep install --from-paths src --ignore-src -r -y

RUN source /colcon_ws/install/setup.bash && \
    colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install

# Set the entrypoint script (modify the entrypoint script as needed)
# ENTRYPOINT ["/entrypoint_scripts/entrypoint_base_node.sh"]
