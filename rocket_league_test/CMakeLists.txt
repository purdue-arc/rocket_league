cmake_minimum_required(VERSION 3.0.2)
project(rocket_league_test)

# find catkin so we can use our macros
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rocket_league_msgs
  std_msgs
  )

# generate variables for installation
catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############

# Mark launch files for installation
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(my_unit_test test/simulation_test.launch test/simulation_test.cpp)
  target_link_libraries(my_unit_test ${catkin_LIBRARIES})
endif()
