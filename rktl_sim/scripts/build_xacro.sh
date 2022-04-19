#!/bin/bash

FIELD_LENGTH=$(rosparam get /field/length)
if [ "$FIELD_LENGTH" == "ERROR:"* ]; then
  echo "$FIELD_LENGTH"
fi

FIELD_WIDTH=$(rosparam get /field/width)
if [ "$FIELD_WIDTH" == "ERROR:"* ]; then
  echo "$FIELD_WIDTH"
fi

GOAL_SIZE=$(rosparam get /field/goal/width)
if [ "$GOAL_SIZE" == "ERROR:"* ]; then
  echo "$GOAL_SIZE"
fi

URDF_DIR=$(realpath $(dirname $0)/../urdf/)

rm $URDF_DIR/walls.urdf
xacro "$URDF_DIR/walls.urdf.xacro" "field_length:=$FIELD_LENGTH" "field_width:=$FIELD_WIDTH" "goal_size:=$GOAL_SIZE" -o "$URDF_DIR/walls.urdf"