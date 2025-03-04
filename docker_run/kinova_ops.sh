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
		--env RMW_IMPLEMENTATION=${rmw_fastrtps_cpp} \
		--env DISPLAY=$DISPLAY \
        --name kinova_ops \
        -v "$REPO_DIR:/kinova-humble:rw" \
        -v $PARENT_DIR:/root/ws/kinova_repos:rw \
        -w /kinova-humble \
        kinova_gen3_7dof_humble:main \
        /kinova-humble/entrypoint_scripts/entrypoint_kinova_ops.sh
