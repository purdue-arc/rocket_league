cmake_minimum_required(VERSION 3.0.2)
project(racersim)

# find catkin so we can use our macros
find_package(catkin REQUIRED)

# install python module(s)
catkin_python_setup()

# generate variables for installation
catkin_package()

#############
## Install ##
#############

# Mark executable python scripts and nodes for installation
catkin_install_python(PROGRAMS
  nodes/sim_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_snakesim.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)