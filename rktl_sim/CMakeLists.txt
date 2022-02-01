cmake_minimum_required(VERSION 3.0.2)
project(rktl_sim)

# find catkin so we can use our macros
find_package(catkin REQUIRED COMPONENTS
  rktl_msgs
)

# install python module(s)
catkin_python_setup()

# generate variables for installation
catkin_package()

# Tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test_car.test)
endif()
