#!/bin/bash

WS_DIR=$(realpath $(dirname $0)/../../../)
REPO_NAME="purduearc/rocket-league"
CONTAINER_NAME="${CONTAINER_NAME:-arc-rocket-league-dev}"
echo "mounting host directory $WS_DIR as container directory /home/$USER/catkin_ws"

# tty-specific options
if [ -t 0 -a -t 1 ]
then
    TTY_OPTS="-it"
fi

$WS_DIR/src/rocket_league/docker/docker-build.sh

docker run --rm \
    $TTY_OPTS \
    -e USER \
    -e DISPLAY \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v $XAUTHORITY:/home/$USER/.Xauthority:ro \
    -v $WS_DIR:/home/$USER/catkin_ws \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    `[ -f ~/.gitconfig ] && echo "-v $HOME/.gitconfig:/home/$USER/.gitconfig:ro"` \
    -v $HOME/.Xauthority:/home/$USER/.Xauthority \
    -v ~/.ssh:/home/$USER/.ssh:ro \
    --net host \
    --name $USER-$CONTAINER_NAME \
    $@ \
    $REPO_NAME:local-$USER \
    ${DOCKER_CMD:+/bin/zsh -c "$DOCKER_CMD"}

docker kill $REPO_NAME:local-$USER
docker rmi $REPO_NAME:local-$USER
