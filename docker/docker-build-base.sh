#!/bin/bash

DOCKER_DIR=$(readlink -f $(dirname $0))
REPO_NAME="purduearc/rocket-league-test-repository-please-ignore"

ARCH=${ARCH:-"amd64, arm64"}
PLATFORM_ARG=`printf '%s ' '--platform'; for var in $(echo $ARCH | sed "s/,/ /g"); do printf 'linux/%s,' "$var"; done | sed 's/,*$//g'`

docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

docker buildx build \
    ${PLATFORM_ARG} \
    --tag $REPO_NAME \
    --cache-from=type=registry,ref=${REPO_NAME} \
    --cache-to=type=registry,ref=${REPO_NAME},mode=max \
    --push \
    $@ \
    $DOCKER_DIR
