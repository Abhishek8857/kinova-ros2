#!/bin/bash

SCRIPT_DIR="$(dirname $(readlink -f $0))"
REPO_DIR="$(realpath "${SCRIPT_DIR}/..")"	
PARENT_DIR="$(realpath "${REPO_DIR}/..")"
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
IMAGE_NAME=$(cat "${REPO_DIR}/image_name.cfg")


xhost +
docker run \
		-it \
		--rm \
		--net=host \
		--pid=host \
		--ipc=host \
		--privileged \
        --gpus all \
        --runtime=nvidia \
		-v /dev:/dev \
		-v $HOME/.ros/log:/.ros/log \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		--env RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} \
		--env DISPLAY=$DISPLAY \
        --name robot_launch \
        -v "$REPO_DIR:/kinova-ros2:rw" \
        -v $PARENT_DIR:/root/workspaces/:rw \
        -w /kinova-ros2 \
    	$IMAGE_NAME \
        /kinova-ros2/entrypoint_scripts/entrypoint_fake_robot_launch.sh
