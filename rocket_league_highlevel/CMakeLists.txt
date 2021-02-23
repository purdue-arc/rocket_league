cmake_minimum_required(VERSION 3.0.2)
project(rocket_league_highlevel)

# find catkin so we can use our macros
find_package(catkin REQUIRED COMPONENTS
    rocket_league_msgs
)

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
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)