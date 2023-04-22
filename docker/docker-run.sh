#!/bin/bash

WS_DIR=$(realpath $(dirname $0)/../../../)
REPO_NAME="purduearc/rocket-league"
CONTAINER_NAME="${CONTAINER_NAME:-$USER-arc-rocket-league-dev}"
MY_DISPLAY=$([ $(uname -s) == 'Darwin' ] && echo "host.docker.internal:0" || echo "$DISPLAY")
echo "Using display $MY_DISPLAY"
echo "mounting host directory $WS_DIR as container directory /home/$USER/catkin_ws"

# tty-specific options
if [ -t 0 -a -t 1 ]; then
    XTRA_OPTS="-it"
fi

# gitconfig
if [[ -f "$HOME/.gitconfig" ]]; then
    XTRA_OPTS="$XTRA_OPTS -v $HOME/.gitconfig:/home/$USER/.gitconfig:ro"
fi

# Create histroy file and vscode-server files if they don't exist
if [[ ! -f "$WS_DIR/.zsh_history" ]]; then
    touch "$WS_DIR/.zsh_history"
fi

if [[ ! -d "$WS_DIR/.vscode-server" ]]; then
    mkdir "$WS_DIR/.vscode-server"
fi

docker run --rm \
    $XTRA_OPTS \
    -e USER \
    -e DISPLAY=$MY_DISPLAY \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v $XAUTHORITY:/home/$USER/.Xauthority:ro \
    -v $WS_DIR:/home/$USER/catkin_ws \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $WS_DIR/.zsh_history:/home/$USER/.zsh_history \
    -v $WS_DIR/.vscode-server:/home/$USER/.vscode-server \
    -v ~/.ssh:/home/$USER/.ssh:ro \
    --name $CONTAINER_NAME \
    $@ \
    $REPO_NAME:local-$USER \
    ${DOCKER_CMD:+/bin/zsh -c "$DOCKER_CMD"}
