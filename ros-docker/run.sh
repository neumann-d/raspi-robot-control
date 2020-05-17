#!/bin/bash

# go to directory of this script
THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$THIS_DIR"

IMAGE_NAME=ros-docker
WORKSPACE_DIR="$THIS_DIR/../ros-robot-control"

# run container (--rm flag will remove container after you exit)
docker run --rm -it --net=host --privileged --volume="$WORKSPACE_DIR":/root/workspace --name="$IMAGE_NAME" $IMAGE_NAME terminator
