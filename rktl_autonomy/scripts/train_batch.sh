#!/bin/bash

# Script to automate running several training containers all at once
# Usage:

set -e

# modify this if you want a different script to be executed
TRAIN_CMD=" source catkin_ws/devel/setup.zsh  &&
    catkin_ws/src/rocket_league/rktl_autonomy/scripts/train_rocket_league.py"

# paths
WS_DIR=$(realpath $(dirname $0)/../../../../)   # workspace dir on host
DOCKER_DIR="src/rocket_league/docker"           # docker dir in workspace
LOG_DIR="data/rocket_league/batch_logs"         # log dir in workspace

# generate unique UUID for this script execution
UUID=$(uuidgen)
echo "logging to $WS_DIR/$LOG_DIR/$UUID"
mkdir -p $WS_DIR/$LOG_DIR/$UUID

echo "running a batch of $# experiments"
for commit in "$@"; do
    echo "" # newline

    # checkout code
    echo "checking out git commit $commit"
    # git checkout $commit

    # create log file
    echo "logging to $WS_DIR/$LOG_DIR/$UUID/$commit.log"
    touch $WS_DIR/$LOG_DIR/$UUID/$commit.log

    # set arguments for container
    DOCKER_CMD="$TRAIN_CMD &> catkin_ws/$LOG_DIR/$UUID/$commit.log"
    export DOCKER_CMD
    CONTAINER_NAME="$UUID""_$commit"
    export CONTAINER_NAME

    # launch container
    echo "launching container $CONTAINER_NAME"
    $WS_DIR/$DOCKER_DIR/docker-run.sh --gpus all -d > /dev/null

    # wait until done launching
    until egrep "training on [0-9]+ steps" $WS_DIR/$LOG_DIR/$UUID/$commit.log; do
        sleep 1
    done

    # print RUN ID
    echo "training successfully started"
    grep "RUN ID: " $WS_DIR/$LOG_DIR/$UUID/$commit.log
done
