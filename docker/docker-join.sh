#!/bin/bash

REPO_NAME="purduearc/rocket-league"

docker exec -it \
    -e DISPLAY \
    -e LIBGL_ALWAYS_INDIRECT \
    $REPO_NAME:local \
    /bin/zsh
