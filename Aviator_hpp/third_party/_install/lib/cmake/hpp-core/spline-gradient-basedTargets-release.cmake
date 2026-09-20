#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "hpp-core::spline-gradient-based" for configuration "Release"
set_property(TARGET hpp-core::spline-gradient-based APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(hpp-core::spline-gradient-based PROPERTIES
  IMPORTED_COMMON_LANGUAGE_RUNTIME_RELEASE ""
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/hppPlugins/spline-gradient-based.so"
  IMPORTED_NO_SONAME_RELEASE "TRUE"
  )

list(APPEND _IMPORT_CHECK_TARGETS hpp-core::spline-gradient-based )
list(APPEND _IMPORT_CHECK_FILES_FOR_hpp-core::spline-gradient-based "${_IMPORT_PREFIX}/lib/hppPlugins/spline-gradient-based.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
