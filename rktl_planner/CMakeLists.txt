cmake_minimum_required(VERSION 3.0.2)
project(rktl_planner)

# find catkin so we can use our macros
find_package(catkin REQUIRED COMPONENTS
  rospy
  rktl_msgs
  std_srvs
  std_msgs
  geometry_msgs
  message_generation
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

# msg and srv
add_service_files(
  FILES
  CreateBezierPath.srv
)

generate_messages(
  DEPENDENCIES
  rktl_msgs
  std_srvs
  std_msgs
  geometry_msgs
)

# generate variables for installation
catkin_package()
