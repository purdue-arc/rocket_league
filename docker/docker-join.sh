#!/bin/bash

IMAGE_NAME="arc-rocket-league-dev"

docker exec -it \
    -e DISPLAY \
    -e LIBGL_ALWAYS_INDIRECT \
    $IMAGE_NAME \
    /bin/zsh
