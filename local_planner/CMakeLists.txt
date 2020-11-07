cmake_minimum_required(VERSION 3.0.2)
project(local_planner)

# find catkin so we can use our macros
find_package(catkin REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

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