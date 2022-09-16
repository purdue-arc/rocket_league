#!/bin/bash

DOCKER_DIR=$(realpath $(dirname $0))
REPO_NAME="purduearc/rocket-league"
M1_ARGS=$([ $(uname -p) == 'arm' ] && echo "--platform \"linux/arm64/v8\"")

docker build --build-arg USER=$USER \
             --build-arg PW="robot" \
             --build-arg UID=$(id -u) \
             --build-arg GID=$(id -g) \
             --build-arg ROS_DEPS="$ROS_DEPS" \
             $M1_ARGS \
             -t $REPO_NAME:local-$USER \
             -f $DOCKER_DIR/Dockerfile.local \
             $@ \
             $DOCKER_DIR

if [ $? -eq 0 ]; then
echo "
ARC Rocket League development image built as '$REPO_NAME:local'
sudo password in container is 'robot'.
Run 'sudo passwd' inside the container to change it,
then run 'docker commit <container-id>' in a new terminal to make persistent"
else
echo "
Error building image, see above messages" >&2
exit 1
fi
