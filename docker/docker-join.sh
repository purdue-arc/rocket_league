#!/bin/bash

CONTAINER_NAME="${CONTAINER_NAME:-$USER-arc-rocket-league-dev}"

docker exec -it \
    -e DISPLAY \
    -e LIBGL_ALWAYS_INDIRECT \
    $@ \
    $CONTAINER_NAME \
    ${DOCKER_CMD:-/bin/zsh}
