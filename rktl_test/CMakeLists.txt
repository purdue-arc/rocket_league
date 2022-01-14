cmake_minimum_required(VERSION 3.0.2)
project(rktl_test)

# find catkin so we can use our macros
find_package(catkin REQUIRED COMPONENTS
  rospy
  rktl_msgs
  std_msgs
  rostest
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
  add_rostest(test/test_simulation.test)
endif()
