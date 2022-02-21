#!/bin/bash

WS_DIR=$(realpath $(dirname $0)/../../../)
REPO_NAME="purduearc/rocket-league"
CONTAINER_NAME="${CONTAINER_NAME:-arc-rocket-league-dev}"
echo "mounting host directory $WS_DIR as container directory $HOME/catkin_ws"

# tty-specific options
if [ -t 0 -a -t 1 ]
then
    TTY_OPTS="-it"
fi

docker run --rm \
    $TTY_OPTS \
    -e USER \
    -e DISPLAY \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v $XAUTHORITY:$HOME/.Xauthority:ro \
    -v $WS_DIR:$HOME/catkin_ws \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    `[ -f ~/.gitconfig ] && echo "-v $HOME/.gitconfig:$HOME/.gitconfig:ro"` \
    -v ~/.ssh:$HOME/.ssh:ro \
    --name $CONTAINER_NAME \
    $@ \
    $REPO_NAME:local \
    ${DOCKER_CMD:+/bin/zsh -c "$DOCKER_CMD"}
