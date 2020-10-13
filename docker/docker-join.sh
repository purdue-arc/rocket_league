#!/bin/bash

nvidia-docker exec -it \
    -e DISPLAY \
    -e LIBGL_ALWAYS_INDIRECT \
    arc-rocket-league-dev \
    /bin/zsh
