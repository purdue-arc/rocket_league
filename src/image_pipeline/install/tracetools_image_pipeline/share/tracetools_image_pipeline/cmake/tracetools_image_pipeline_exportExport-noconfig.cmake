#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "tracetools_image_pipeline::tracetools_image_pipeline" for configuration ""
set_property(TARGET tracetools_image_pipeline::tracetools_image_pipeline APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(tracetools_image_pipeline::tracetools_image_pipeline PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libtracetools_image_pipeline.so"
  IMPORTED_SONAME_NOCONFIG "libtracetools_image_pipeline.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS tracetools_image_pipeline::tracetools_image_pipeline )
list(APPEND _IMPORT_CHECK_FILES_FOR_tracetools_image_pipeline::tracetools_image_pipeline "${_IMPORT_PREFIX}/lib/libtracetools_image_pipeline.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
