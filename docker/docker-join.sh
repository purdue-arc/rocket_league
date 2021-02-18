#!/bin/bash

CONTAINER_NAME="arc-rocket-league-dev"
DOCKER_CMD=${DOCKER_CMD:+source /home/$USER/.zshrc && $DOCKER_CMD}

echo /bin/zsh $DOCKER_CMD

docker exec -it \
    -e DISPLAY \
    -e LIBGL_ALWAYS_INDIRECT \
    $@ \
    $CONTAINER_NAME \
    $DOCKER_CMD
