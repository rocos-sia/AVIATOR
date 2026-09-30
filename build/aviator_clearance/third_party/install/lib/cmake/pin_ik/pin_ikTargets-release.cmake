#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "pin_ik::pin_ik" for configuration "Release"
set_property(TARGET pin_ik::pin_ik APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(pin_ik::pin_ik PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libpin_ik.so"
  IMPORTED_SONAME_RELEASE "libpin_ik.so"
  )

list(APPEND _cmake_import_check_targets pin_ik::pin_ik )
list(APPEND _cmake_import_check_files_for_pin_ik::pin_ik "${_IMPORT_PREFIX}/lib/libpin_ik.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
