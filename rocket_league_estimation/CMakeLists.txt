cmake_minimum_required(VERSION 3.0.2)
project(rocket_league_estimation)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Find catkin macros and libraries
# if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  apriltag_ros
  geometry_msgs
  tf2
  tf2_geometry_msgs
  image_transport
  cv_bridge
)

find_package(OpenCV REQUIRED
  core
  highgui
)

###################################
## catkin specific configuration ##
###################################
# The catkin_package macro generates cmake config files for your package
# Declare things to be passed to dependent projects
# INCLUDE_DIRS: uncomment this if your package contains header files
# LIBRARIES: libraries you create in this project that dependent projects also need
# CATKIN_DEPENDS: catkin_packages dependent projects also need
# DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Declare a C++ executable
# With catkin_make all packages are built within a single CMake context
# The recommended prefix ensures that target names across packages don't collide
add_executable(detection_to_pose
  src/detection_to_pose.cpp
  src/DetectionToPose.cpp
)

# Add cmake target dependencies of the executable
add_dependencies(detection_to_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(detection_to_pose
  ${catkin_LIBRARIES}
)

add_executable(ball_detection
  src/ball_detection.cpp
  src/BallDetection.cpp
)

add_dependencies(ball_detection
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${OpenCV_EXPORTED_TARGETS}
)

# Specify libraries to link a library or executable target against
target_link_libraries(ball_detection
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

# Mark Python executables for installation
catkin_install_python(PROGRAMS
  nodes/pose_estimator
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp executables for installation
install(TARGETS
  detection_to_pose
  ball_detection
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
