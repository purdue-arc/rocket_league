cmake_minimum_required(VERSION 3.0.2)
project(local_planner)

# find catkin so we can use our macros
find_package(catkin REQUIRED)

# install python module(s)
# catkin_python_setup()

# generate variables for installation
catkin_package()

#############
## Install ##
#############

# Mark executable python scripts and nodes for installation
catkin_install_python(PROGRAMS
  nodes/trajectory_planner_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)