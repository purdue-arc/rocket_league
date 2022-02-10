cmake_minimum_required(VERSION 3.0.2)
project(rktl_sim)

# find catkin so we can use our macros
find_package(catkin REQUIRED)

# install python module(s)
catkin_python_setup()

# generate variables for installation
catkin_package()

# tests
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
  find_package(rostest REQUIRED)
  add_rostest(test/test_car.test)
endif()
