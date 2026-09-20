#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "hpp-constraints::hpp-constraints" for configuration "Release"
set_property(TARGET hpp-constraints::hpp-constraints APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(hpp-constraints::hpp-constraints PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libhpp-constraints.so"
  IMPORTED_SONAME_RELEASE "libhpp-constraints.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS hpp-constraints::hpp-constraints )
list(APPEND _IMPORT_CHECK_FILES_FOR_hpp-constraints::hpp-constraints "${_IMPORT_PREFIX}/lib/libhpp-constraints.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
