#!/bin/bash

WS_DIR=$(readlink -f $(dirname $0)/../../../)
REPO_NAME="purduearc/rocket-league"
echo "mounting host directory $WS_DIR as container directory /home/$USER/catkin_ws"

CMD="/bin/zsh"
for ARGS in "$@"; do
shift
    case "$ARGS" in
        "--use-gpu") NVIDIA_ARGS="
                        --gpus all \
                        -e NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all} \
                        -e NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}compute,graphics" ;;
        *) CMD=$ARGS
    esac
done

docker run --rm -it \
    -e USER \
    -e DISPLAY \
    -v $XAUTHORITY:/home/$USER/.Xauthority \
    -v $WS_DIR:/home/$USER/catkin_ws \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --hostname arc-rocket-league-dev \
    --name arc-rocket-league-dev \
    --privileged \
    $NVIDIA_ARGS \
    $REPO_NAME:local \
    $CMD
