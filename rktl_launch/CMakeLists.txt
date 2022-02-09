cmake_minimum_required(VERSION 3.0.2)
project(rktl_launch)

# find catkin so we can use our macros
find_package(catkin REQUIRED)

# generate variables for installation
catkin_package()

# Tests
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
