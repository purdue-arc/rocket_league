#!/bin/bash

CONTAINER_NAME="arc-rocket-league-dev"

docker exec -it \
    -e DISPLAY \
    -e LIBGL_ALWAYS_INDIRECT \
    $@ \
    $CONTAINER_NAME
