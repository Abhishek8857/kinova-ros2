# ROS 2 Kortex Dockerized Workspace

This repository provides a Dockerized environment for working with the ROS 2 Kortex package. The workspace is set up to include the ros2_kortex repository as a Git submodule inside a ROS 2 workspace (colcon_ws), which is then built and run within a Docker container.

## Prerequisites

- **Docker**: Ensure Docker is installed on your system. You can download it from [here](https://www.docker.com/get-started).

## Setup Instructions

<!-- ### 0. Network Setup -->
<!-- For network setup, please follow the instructions from the  [kinova ROS 1](https://git-ce.rwth-aachen.de/wzl-mq-ms/docker-ros/ros/kinova-ros) -->

### 1. Clone This Repository

Clone this repository along with its submodules to your local machine (make sure you're cloning from the right branch):

```sh
git clone --recurse-submodules https://github.com/Abhishek8857/kinova-ros2.git
cd /kinova-ros2
```
If you’ve already cloned the repository without submodules, you can initialize and update the submodules like this:

```sh
git submodule update --init --recursive
```

### 2. Build the Docker Image

Build the Docker image using the provided Dockerfile. This command must be run from the root of the repository where the Dockerfile is located:

```sh
bash docker_build.sh
```
This will create a Docker image named ros2-kortex:latest, which includes the ROS 2 environment and the ros2_kortex package.

**The build process may take some time. On systems with an RTX 3050, the process may freeze when building Gazebo dependencies. Grabbing a coffee is recommended if the process stalls.**


### 3. Run the Docker Container

Once the Docker image is built, you can run the container interactively using:

```sh
cd docker_run/
bash docker_vision.sh
```

This will start a new container from the ros2-kortex:latest image, allowing you to interact with the ROS 2 environment.

### 4. Running ROS 2 Nodes

After entering the container, you can source the ROS 2 and workspace setup files and run ROS 2 nodes or launch files:

```sh
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Source the workspace overlay
source /colcon_ws/install/setup.bash

```

## Usage

To launch and view any of the robot's URDF run:
```sh
ros2 launch kortex_description view_robot.launch.py
```
The accepted arguments are:

- `robot_type` : Your robot model. Possible values are either `gen3` or `gen3_lite`, the default is `gen3`.

- `gripper` : Gripper to use. Possible values for the Gen3 are either `robotiq_2f_85` or `robotiq_2f_140`. For the Gen3 Lite, the only option is `gen3_lite_2f`. Default value is an empty string, which will display the arm without a gripper.

- `dof` : Degrees of freedom of the arm. Possible values for the Gen3 are either `6` or `7`. For the Gen3 Lite, the only option is `6`. Default value is `7`.

### Gen 3 Robots

The `gen3.launch.py` launch file is designed to be used for Gen3 arms. The typical use case to bringup and visualize the 7 DoF Kinova Gen3 robot arm (default) with mock hardware on Rviz:
```sh
ros2 launch kortex_bringup gen3.launch.py \
  robot_ip:=yyy.yyy.yyy.yyy \
  use_fake_hardware:=true
```

Alternatively, for a physical robot:
```sh
ros2 launch kortex_bringup gen3.launch.py \
  robot_ip:=192.168.1.10 \
```

You can specify additional arguments if you wish to change your arm configuration: see details at the section of same name [ROS 2 Kortex](https://github.com/Kinovarobotics/ros2_kortex?tab=readme-ov-file#gen-3-robots)

## Simulation

The `kortex_sim_control.launch.py` launch file is designed to simulate all of our arm models, you just need to specify your configuration through the arguments. By default, the Gen3 7 dof configuration is used :

```sh
ros2 launch kortex_bringup kortex_sim_control.launch.py \
  use_sim_time:=true \
  launch_rviz:=false
```

For arguments, see details at the section of same name [ROS 2 Kortex](https://github.com/Kinovarobotics/ros2_kortex?tab=readme-ov-file#simulation)

# MoveIt2

* Remember to open a new terminal and:
  ```sh
  cd /kinova-ros2/docker_run/
  bash docker_exec.sh
  ```
  Then repeat step 4 [Running ROS 2 Nodes](https://git-ce.rwth-aachen.de/wzl-mq-ms/docker-ros/ros2/kinova-ros2/-/edit/devel_lmanasses/README.md?ref_type=heads#4-running-ros-2-nodes)


To generate motion plans and execute them with a simulated 7 DoF Kinova Gen3 arm with mock hardware:

```sh
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py \
  robot_ip:=yyy.yyy.yyy.yyy \
  use_fake_hardware:=true
```

**To generate motion plans and execute them with an ignition simulated 7 DoF Kinova Gen3 arm (previously launched with the command at the simulation section):**

```sh
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config sim.launch.py \
  use_sim_time:=true
```

To work with a physical robot and generate/execute paths with MoveIt run the following:

For Gen3:
```sh
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py \
  robot_ip:=192.168.1.10
```
For Gen3-Lite:
```sh
ros2 launch kinova_gen3_lite_moveit_config robot.launch.py \
  robot_ip:=192.168.1.10
```

### Exit the Container

To exit the container, simply type exit and press Enter.


### Notes

This setup is based on the ROS 2 Humble distribution. If you need to use a different ROS 2 distribution, modify the Dockerfile and the setup commands accordingly.

The Docker container is configured to use CycloneDDS as the default RMW (ROS Middleware). This can be changed by modifying the Dockerfile and related configuration files.
