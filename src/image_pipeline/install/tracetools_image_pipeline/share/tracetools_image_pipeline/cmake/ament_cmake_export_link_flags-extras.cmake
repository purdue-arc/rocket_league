# generated from ament_cmake_export_link_flags/cmake/ament_cmake_export_link_flags-extras.cmake.in

set(_exported_link_flags "-rdynamic")

# append link_flags to tracetools_image_pipeline_LINK_FLAGS
if(NOT "${_exported_link_flags} " STREQUAL " ")
  foreach(_link_flag ${_exported_link_flags})
    list(APPEND tracetools_image_pipeline_LINK_FLAGS "${_link_flag}")
  endforeach()
endif()
