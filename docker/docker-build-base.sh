#!/bin/bash

DOCKER_DIR=$(readlink -f $(dirname $0))
REPO_NAME="purduearc/rocket-league-test-repository-please-ignore"

ARCH=${ARCH:-"amd64, arm64"}
PLATFORM_ARG=`printf '%s ' '--platform'; for var in $(echo $ARCH | sed "s/,/ /g"); do printf 'linux/%s,' "$var"; done | sed 's/,*$//g'`

docker buildx build \
    ${PLATFORM_ARG} \
    --tag $REPO_NAME \
    $@ \
    $DOCKER_DIR
