#!/bin/bash

WS_DIR=$(realpath $(dirname $0)/../../../)
REPO_NAME="ghcr.io/purduearc/rocket-league"
CONTAINER_NAME="${CONTAINER_NAME:-$USER-arc-rocket-league-dev}"
MY_DISPLAY=$([ $(uname -s) == 'Darwin' ] && echo "host.docker.internal:0" || echo "$DISPLAY")
echo "Using display $MY_DISPLAY"
echo "mounting host directory $WS_DIR as container directory /home/$USER/catkin_ws"

# tty-specific options
if [ -t 0 -a -t 1 ]
then
    TTY_OPTS="-it"
fi

docker run --rm \
    $TTY_OPTS \
    -e USER \
    -e DISPLAY=$MY_DISPLAY \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v $XAUTHORITY:/home/$USER/.Xauthority:ro \
    -v $WS_DIR:/home/$USER/catkin_ws \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    `[ -f ~/.gitconfig ] && echo "-v $HOME/.gitconfig:/home/$USER/.gitconfig:ro"` \
    -v ~/.ssh:/home/$USER/.ssh:ro \
    --name $CONTAINER_NAME \
    $@ \
    $REPO_NAME:local-$USER \
    ${DOCKER_CMD:+/bin/zsh -c "$DOCKER_CMD"}
