cmake_minimum_required(VERSION 3.0.2)
project(rktl_perception)

## Compile as C++11
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp  
  pointgrey_camera_driver
  image_proc
  tf2
  tf2_ros
  image_transport
  cv_bridge
  apriltag_ros
  geometry_msgs
  image_geometry
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED
  core
  highgui
)
find_package(Eigen3 REQUIRED)

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
  INCLUDE_DIRS include
#  LIBRARIES camera_tracking
  CATKIN_DEPENDS 
    roscpp
    pointgrey_camera_driver
    image_proc
    tf2
    tf2_ros
    image_transport
    cv_bridge
    apriltag_ros
  DEPENDS OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_tf_rebroadcaster_node
  src/tf_rebroadcaster_node.cpp
  src/tf_rebroadcaster.cpp
)
add_executable(${PROJECT_NAME}_camera_origin_node
  src/camera_origin_node.cpp
)
add_executable(${PROJECT_NAME}_tf_parser_node
  src/tf_parser_node.cpp
)
add_executable(${PROJECT_NAME}_pose_extractor_node
  src/pose_extractor_node.cpp
)
add_executable(${PROJECT_NAME}_chromatag_preprocess_node
  src/chromatag_preprocess_node.cpp
)
add_executable(${PROJECT_NAME}_localizer_node
  src/localizer_node.cpp
  src/localizer.cpp
)
add_executable(ball_detection
  src/ball_detection.cpp
  src/ball_detection_node.cpp
)
add_executable(detection_to_pose
  src/detection_to_pose.cpp
  src/DetectionToPose.cpp
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(${PROJECT_NAME}_tf_rebroadcaster_node PROPERTIES
  OUTPUT_NAME tf_rebroadcaster
  PREFIX ""
)
set_target_properties(${PROJECT_NAME}_camera_origin_node PROPERTIES
  OUTPUT_NAME camera_origin
  PREFIX ""
)
set_target_properties(${PROJECT_NAME}_tf_parser_node PROPERTIES
  OUTPUT_NAME tf_parser
  PREFIX ""
)
set_target_properties(${PROJECT_NAME}_pose_extractor_node PROPERTIES
  OUTPUT_NAME pose_extractor
  PREFIX ""
)
set_target_properties(${PROJECT_NAME}_localizer_node PROPERTIES
  OUTPUT_NAME localizer
  PREFIX ""
)
set_target_properties(${PROJECT_NAME}_chromatag_preprocess_node PROPERTIES
  OUTPUT_NAME chromatag_preprocess
  PREFIX ""
)
set_target_properties(${PROJECT_NAME}_localizer_node PROPERTIES
  OUTPUT_NAME localizer
  PREFIX ""
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_tf_rebroadcaster_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(${PROJECT_NAME}_camera_origin_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(${PROJECT_NAME}_tf_parser_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(${PROJECT_NAME}_pose_extractor_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(${PROJECT_NAME}_chromatag_preprocess_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(${PROJECT_NAME}_localizer_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(ball_detection
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${OpenCV_EXPORTED_TARGETS}
)
add_dependencies(detection_to_pose
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_tf_rebroadcaster_node
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_camera_origin_node
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_tf_parser_node
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_pose_extractor_node
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_chromatag_preprocess_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_localizer_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)
target_link_libraries(ball_detection
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(detection_to_pose
  ${catkin_LIBRARIES}
)
