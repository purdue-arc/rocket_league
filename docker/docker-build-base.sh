#!/bin/bash

DOCKER_DIR=$(realpath $(dirname $0))
REPO_NAME="purduearc/rocket-league"

docker build \
    --tag $REPO_NAME \
    --cache-from=type=registry,ref=${REPO_NAME}:cache \
    $@ \
    $DOCKER_DIR
