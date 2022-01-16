cmake_minimum_required(VERSION 3.0.2)
project(rktl_msgs)

# find catkin so we can use our macros
find_package(catkin REQUIRED COMPONENTS
   std_msgs
   geometry_msgs
   message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MatchStatus.msg
  Path.msg
  PathError.msg
  Target.msg
  Waypoint.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  rktl_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rktl_msgs
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)
