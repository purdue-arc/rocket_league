cmake_minimum_required(VERSION 3.0.2)
project(rktl_perception)

## Compile as C++11
add_compile_options(-std=c++11)

## Find catkin macros and libraries
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

## System dependencies
find_package(OpenCV REQUIRED
  core
  highgui
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES rktl_perception
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

# Add include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Build chromatag_preprocess node
add_executable(${PROJECT_NAME}_chromatag_preprocess_node
  src/chromatag_preprocess_node.cpp
)
set_target_properties(${PROJECT_NAME}_chromatag_preprocess_node PROPERTIES
  OUTPUT_NAME chromatag_preprocess
  PREFIX ""
)
add_dependencies(${PROJECT_NAME}_chromatag_preprocess_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_chromatag_preprocess_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Build localizer node
add_executable(${PROJECT_NAME}_localizer_node
  src/localizer_node.cpp
  src/localizer.cpp
)
set_target_properties(${PROJECT_NAME}_localizer_node PROPERTIES
  OUTPUT_NAME localizer
  PREFIX ""
)
add_dependencies(${PROJECT_NAME}_localizer_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_localizer_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

# Build ball_detection node
add_executable(${PROJECT_NAME}_ball_detection_node
  src/ball_detection.cpp
  src/ball_detection_node.cpp
)
set_target_properties(${PROJECT_NAME}_ball_detection_node PROPERTIES
  OUTPUT_NAME ball_detection
  PREFIX ""
)
add_dependencies(${PROJECT_NAME}_ball_detection_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${OpenCV_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_ball_detection_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# vis node
add_executable(${PROJECT_NAME}_focus_vis_node
  src/focus_vis.cpp
  src/focus_vis_node.cpp
)
set_target_properties(${PROJECT_NAME}_focus_vis_node PROPERTIES
  OUTPUT_NAME focus_vis
  PREFIX ""
)
add_dependencies(${PROJECT_NAME}_focus_vis_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${OpenCV_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_focus_vis_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Tests
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()