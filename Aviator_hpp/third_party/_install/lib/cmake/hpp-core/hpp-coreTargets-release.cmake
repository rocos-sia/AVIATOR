#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "hpp-core::hpp-core" for configuration "Release"
set_property(TARGET hpp-core::hpp-core APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(hpp-core::hpp-core PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libhpp-core.so"
  IMPORTED_SONAME_RELEASE "libhpp-core.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS hpp-core::hpp-core )
list(APPEND _IMPORT_CHECK_FILES_FOR_hpp-core::hpp-core "${_IMPORT_PREFIX}/lib/libhpp-core.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
