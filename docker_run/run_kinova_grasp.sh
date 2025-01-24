#!/bin/bash

SCRIPT_DIR="$(dirname $(readlink -f $0))"
REPO_DIR="$(realpath "${SCRIPT_DIR}/..")"	
PARENT_DIR="$(realpath "${REPO_DIR}/..")"


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
        --name kinova_grasp \
        -v "$REPO_DIR:/kinova-ros2:rw" \
        -v $PARENT_DIR:/root/ws/kinova_repos:rw \
        -w /kinova-ros2 \
        ros2-kortex-vision-moveit-temp:latest \
        /kinova-ros2/entrypoint_scripts/entrypoint_kinova_grasp.sh
