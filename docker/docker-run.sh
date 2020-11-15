#!/bin/bash

WS_DIR="$(readlink -f $(dirname $0)/../../../)"
echo "mounting host directory $WS_DIR as container directory /home/$USER/catkin_ws"

docker run --rm -it \
    -e USER \
    -e DISPLAY \
    -v $XAUTHORITY:/home/$USER/.Xauthority \
    -v $WS_DIR:/home/$USER/catkin_ws \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --hostname arc-rocket-league-dev \
    --name arc-rocket-league-dev \
    --privileged \
    arc-rocket-league-dev \
    /bin/zsh
