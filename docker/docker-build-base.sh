#!/bin/bash

DOCKER_DIR=$(readlink -f $(dirname $0))
REPO_NAME="purduearc/rocket-league"

docker build -t $REPO_NAME $@ $DOCKER_DIR

+if [ $? -eq 0 ]; then
echo "
ARC Rocket League development image built as '$REPO_NAME'
To push, run 'docker push $REPO_NAME:latest'"
else
echo "
Error building image, see above messages" >&2
fi
