cmake_minimum_required(VERSION 3.0.2)
project(rocket_league_test)

# find catkin so we can use our macros
find_package(catkin REQUIRED COMPONENTS
  rocket_league_msgs
  )

# generate variables for installation
catkin_package()

#############
## Install ##
#############

# Mark executable python scripts and nodes for installation
catkin_install_python(PROGRAMS
  nodes/test_simulation
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

