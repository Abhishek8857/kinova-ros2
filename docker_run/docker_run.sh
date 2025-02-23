#!/bin/bash

# Run rviz
xhost +local:root

cd ../..
docker run \
    -it \
    --rm \
    --privileged \
    --net=host \
    --ipc=host \
    --pid=host \
    --env DISPLAY=$DISPLAY \
    --volume ${HOME}/.Xauthority:/root/.Xauthority:rw \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume /etc/localtime:/etc/localtime:ro \
    --volume /etc/timezone:/etc/timezone:ro \
    --name test \
    ros2-kortex-vision-moveit:latest \
    bash



