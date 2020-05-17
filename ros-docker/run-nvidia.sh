#!/bin/bash

# go to directory of this script
THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$THIS_DIR"

IMAGE_NAME=ros-docker
WORKSPACE_DIR="$THIS_DIR/../ros-robot-control"

xhost local:root

DOCKER_X11_ARGS="--gpus all --env=NVIDIA_VISIBLE_DEVICES=all --env=NVIDIA_DRIVER_CAPABILITIES=all --env=DISPLAY --env=QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw"

# run container (--rm flag will remove container after you exit)
docker run --rm -it --net=host --privileged $DOCKER_X11_ARGS --volume="$WORKSPACE_DIR":/root/workspace --name="$IMAGE_NAME" $IMAGE_NAME terminator
