cmake_minimum_required(VERSION 3.8)
project(rktl_perception2)

## Enable C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# find dependencies added from previous
find_package(pointgrey_camera_driver REQUIRED)
find_package(image_proc REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(apriltag_ros REQUIRED)
find_package(nodelet REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# System dependencies
find_package(OpenCV REQUIRED COMPONENTS core highgui)
find_package(Eigen3 REQUIRED)

# Add include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
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
  )
target_link_libraries(${PROJECT_NAME}_localizer_node
  Eigen3::Eigen
  )
ament_target_dependencies(${PROJECT_NAME}_localizer_node
  rclcpp
  std_msgs
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
  ${OpenCV_EXPORTED_TARGETS}
  )
target_link_libraries(${PROJECT_NAME}_ball_detection_node
  ${OpenCV_LIBRARIES}
  )
ament_target_dependencies(${PROJECT_NAME}_ball_detection_node
  rclcpp
  std_msgs
  )

# Build ball_color node
add_executable(${PROJECT_NAME}_ball_color_node
  src/ball_color.cpp
  )
set_target_properties(${PROJECT_NAME}_ball_color_node PROPERTIES
  OUTPUT_NAME ball_color
  PREFIX ""
  )
add_dependencies(${PROJECT_NAME}_ball_color_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${OpenCV_EXPORTED_TARGETS}
  )
  target_link_libraries(${PROJECT_NAME}_ball_color_node
  ${OpenCV_LIBRARIES}
  )

ament_target_dependencies(${PROJECT_NAME}_ball_color_node
  rclcpp
  std_msgs
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
  ${OpenCV_EXPORTED_TARGETS}
  )
target_link_libraries(${PROJECT_NAME}_focus_vis_node
  ${OpenCV_LIBRARIES}
  )
ament_target_dependencies(${PROJECT_NAME}_focus_vis_node
  rclcpp
  std_msgs
  )

  if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
    endif()
  
ament_export_dependencies(rclcpp
  pointgrey_camera_driver
  image_proc
  tf2
  tf2_ros
  image_transport
  cv_bridge
  apriltag_ros
  OpenCV
  )
  
ament_export_include_directories(include)
ament_package() 