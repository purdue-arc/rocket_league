#!/bin/bash

DOCKER_DIR=$(readlink -f $(dirname $0))
REPO_NAME="purduearc/rocket-league"

docker build -t $REPO_NAME $DOCKER_DIR

echo "
ARC Rocket League development image built as '$REPO_NAME'
To push, run 'docker push $REPO_NAME:latest'"
