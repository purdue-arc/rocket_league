#!/bin/bash

WS_DIR=$(cd $(pwd -P)/$(dirname $0)/../../../ && echo $(pwd -P))
REPO_NAME="purduearc/rocket-league"
CONTAINER_NAME="${CONTAINER_NAME:-arc-rocket-league-dev}"
echo "mounting host directory $WS_DIR as container directory /home/$USER/catkin_ws"

docker run --rm -it \
    -e USER \
    -e DISPLAY \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v $XAUTHORITY:/home/$USER/.Xauthority:ro \
    -v $WS_DIR:/home/$USER/catkin_ws \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/.gitconfig:/home/$USER/.gitconfig:ro \
    -v ~/.ssh:/home/$USER/.ssh:ro \
    --name $CONTAINER_NAME \
    $@ \
    $REPO_NAME:local \
    $DOCKER_CMD
