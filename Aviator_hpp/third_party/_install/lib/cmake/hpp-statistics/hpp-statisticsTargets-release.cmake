#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "hpp-statistics::hpp-statistics" for configuration "Release"
set_property(TARGET hpp-statistics::hpp-statistics APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(hpp-statistics::hpp-statistics PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libhpp-statistics.so"
  IMPORTED_SONAME_RELEASE "libhpp-statistics.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS hpp-statistics::hpp-statistics )
list(APPEND _IMPORT_CHECK_FILES_FOR_hpp-statistics::hpp-statistics "${_IMPORT_PREFIX}/lib/libhpp-statistics.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
