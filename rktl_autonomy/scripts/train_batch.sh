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
LOG_DIR="data/rocket_league/batch_logs"         # our log dir in workspace
TRAIN_LOG_DIR="data/rocket_league"              # training script's log dir in workspace

# generate unique UUID for this script execution
UUID=$(uuidgen)
echo "logging to $WS_DIR/$LOG_DIR/$UUID"
mkdir -p $WS_DIR/$LOG_DIR/$UUID

echo "running a batch of $# experiments"
for commit in "$@"; do
    echo "" # newline

    # checkout code
    echo "checking out git commit $commit"
    git checkout $commit
    git show -s HEAD

    # create log file
    LOG_FILE="$WS_DIR/$LOG_DIR/$UUID/$commit.log"
    touch $LOG_FILE
    echo "logging to $LOG_FILE"

    # set arguments for container
    DOCKER_CMD="$TRAIN_CMD &> catkin_ws/$LOG_DIR/$UUID/$commit.log"
    export DOCKER_CMD
    CONTAINER_NAME="$UUID""_$commit"
    export CONTAINER_NAME

    # launch container
    echo "launching container $CONTAINER_NAME"
    $WS_DIR/$DOCKER_DIR/docker-run.sh --gpus all -d > /dev/null

    # get run id
    until grep "RUN ID:" $LOG_FILE; do
        sleep 1
    done
    RUN_ID=$(grep "RUN ID: " $LOG_FILE | cut -d' ' -f 3)

    # wait until done launching
    until [ -d $WS_DIR/$TRAIN_LOG_DIR/$RUN_ID ]; do
        sleep 1
    done

    # print RUN ID
    echo "training successfully started"
done
