#!/bin/bash

DOCKER_DIR=$(readlink -f $(dirname $0))
REPO_NAME="purduearc/rocket-league"

docker build --build-arg USER=$USER \
             --build-arg PW="robot" \
             --build-arg UID=$(id -u) \
             --build-arg GID=$(id -g) \
             --build-arg ROS_DEPS="$ROS_DEPS" \
             -t $REPO_NAME:local \
             -f $DOCKER_DIR/Dockerfile.local \
             $1 \
             $DOCKER_DIR

echo "
ARC Rocket League development image built as '$REPO_NAME:local'
sudo password in container is 'robot'.
Run 'sudo passwd' inside the container to change it,
then run 'docker commit <container-id>' in a new terminal to make persistent"

