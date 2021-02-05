#!/bin/bash

DOCKER_DIR=$(readlink -f $(dirname $0))
REPO_NAME="purduearc/rocket-league"

# Add any local dependencies here, pipe delimited
# e.g. my_dep1|mydep_2|mydep_3
ROS_DEP_IGNORE="camera_aravis"

# Tags to search for in the package.xml files
TAGS=(depend build_depend build_export_depend exec_depend test_depend buildtool_depend doc_depend)

# Find all ROS dependencies
#   - Find all files named package.xml
#   - For each package.xml, get the contents of dependency tags (contents of $TAGS) from XPath
#   - Remove all duplicates
#   - Remove all local dependencies
#   - Reduce to one line for passing to docker build
ROS_DEPS=$(find "$DOCKER_DIR/.." -name package.xml \
          | xargs xmllint --xpath "$(printf '/package/%s/text()|' "${TAGS[@]}" | sed 's/|$//')" \
          | sort \
          | uniq \
          | sed -E "s/$ROS_DEP_IGNORE//g" \
          | tr '\n' ' ')

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