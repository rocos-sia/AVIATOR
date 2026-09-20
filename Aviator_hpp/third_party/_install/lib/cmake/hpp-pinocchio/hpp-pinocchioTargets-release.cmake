#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "hpp-pinocchio::hpp-pinocchio" for configuration "Release"
set_property(TARGET hpp-pinocchio::hpp-pinocchio APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(hpp-pinocchio::hpp-pinocchio PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libhpp-pinocchio.so"
  IMPORTED_SONAME_RELEASE "libhpp-pinocchio.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS hpp-pinocchio::hpp-pinocchio )
list(APPEND _IMPORT_CHECK_FILES_FOR_hpp-pinocchio::hpp-pinocchio "${_IMPORT_PREFIX}/lib/libhpp-pinocchio.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
