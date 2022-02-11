cmake_minimum_required(VERSION 3.0.2)
project(rktl_control)

# Find catkin macros
find_package(catkin REQUIRED)

# install python module(s)
catkin_python_setup()

# generates cmake config files and set variables for installation
catkin_package()

# Tests
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
  find_package(rostest REQUIRED)
  add_rostest(test/test_sync.test)
  add_rostest(test/test_mean_filter.test)
endif()
