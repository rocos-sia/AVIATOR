
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was Config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################



set(skip_this_file TRUE)
if(NOT hpp-constraints_FOUND)
  set(skip_this_file FALSE)
endif()
if(skip_this_file)
  foreach(component ${hpp-constraints_FIND_COMPONENTS})
    if(NOT "hpp-constraints_${component}_FOUND")
      set(skip_this_file FALSE)
    endif()
  endforeach()
endif()
if(skip_this_file)
  return()
endif()

set("hpp-constraints_INCLUDE_DIRS" "${PACKAGE_PREFIX_DIR}/include")
set("HPP_CONSTRAINTS_INCLUDE_DIRS" "${PACKAGE_PREFIX_DIR}/include")
set("hpp-constraints_DOXYGENDOCDIR" "${PACKAGE_PREFIX_DIR}/share/doc/hpp-constraints/doxygen-html")
set(
  "HPP_CONSTRAINTS_DOXYGENDOCDIR"
  "${PACKAGE_PREFIX_DIR}/share/doc/hpp-constraints/doxygen-html"
)
set("hpp-constraints_DEPENDENCIES" "hpp-pinocchio")
set("hpp-constraints_PKG_CONFIG_DEPENDENCIES" "")

set(
  CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  
)

# Find absolute library paths for all _PKG_CONFIG_LIBS as CMake expects full paths, while pkg-config does not.
set(_PACKAGE_CONFIG_LIBRARIES "")
set("_hpp-constraints_PKG_CONFIG_LIBDIR" "${pcfiledir}/../../lib")
set("_hpp-constraints_PKG_CONFIG_LIBS_LIST" "")
if(_hpp-constraints_PKG_CONFIG_LIBS_LIST)
  string(FIND ${_hpp-constraints_PKG_CONFIG_LIBS_LIST} ", " _is_comma_space)
  while(_is_comma_space GREATER -1)
    string(
      REPLACE ", "
      ","
      _hpp-constraints_PKG_CONFIG_LIBS_LIST
      "${_hpp-constraints_PKG_CONFIG_LIBS_LIST}"
    )
    string(FIND ${_hpp-constraints_PKG_CONFIG_LIBS_LIST} ", " _is_comma_space)
  endwhile()
  string(
    REPLACE " "
    ";"
    _hpp-constraints_PKG_CONFIG_LIBS_LIST
    "${_hpp-constraints_PKG_CONFIG_LIBS_LIST}"
  )
  set(LIBDIR_HINTS ${_hpp-constraints_PKG_CONFIG_LIBDIR})
  foreach(component ${_hpp-constraints_PKG_CONFIG_LIBS_LIST})
    string(STRIP ${component} component)
    # If the component is a link directory ("-L/full/path"), append to LIBDIR_HINTS.
    string(FIND ${component} "-L" _is_library_dir)
    if(${_is_library_dir} EQUAL 0)
      string(REGEX REPLACE "^-L" "" lib_path ${component})
      list(APPEND LIBDIR_HINTS "${lib_path}")
      continue()
    endif()
    # If the component is a library name
    string(FIND ${component} "-l" _is_library_name)
    if(${_is_library_name} EQUAL 0)
      string(REGEX REPLACE "^-l" "" lib ${component})
      find_library(abs_lib_${lib} ${lib} HINTS ${LIBDIR_HINTS})
      if(NOT abs_lib_${lib})
        if(_LIBDIR_HINTS)
          message(STATUS "${lib} searched on ${_LIBDIR_HINTS} not FOUND.")
        else()
          message(STATUS "${lib} not FOUND.")
        endif()
      else()
        if(_LIBDIR_HINTS)
          message(
            STATUS
            "${lib} searched on ${_LIBDIR_HINTS} FOUND. ${lib} at ${abs_lib_${lib}}"
          )
        else()
          message(STATUS "${lib} FOUND. ${lib} at ${abs_lib_${lib}}")
        endif()
        list(APPEND _PACKAGE_CONFIG_LIBRARIES "${abs_lib_${lib}}")
      endif()
      unset(abs_lib_${lib} CACHE)
      continue()
    endif()
    # If the component contains a collection of additional arguments
    string(FIND ${component} "," _is_collection)
    if(${_is_collection} GREATER -1)
      string(REPLACE "," ";" component_list "${component}")
      list(GET component_list -1 lib_info)
      set(options ${component})
      list(REMOVE_AT options -1)
      string(FIND ${lib_info} "-l" _is_library_name)
      if(${_is_library_name} GREATER -1)
        string(REGEX REPLACE "^-l" "" lib ${lib_info})
        find_library(abs_lib_${lib} ${lib} HINTS ${LIBDIR_HINTS})
        if(NOT abs_lib_${lib})
          if(_LIBDIR_HINTS)
            message(STATUS "${lib} searched on ${_LIBDIR_HINTS} not FOUND.")
          else()
            message(STATUS "${lib} not FOUND.")
          endif()
        else()
          if(_LIBDIR_HINTS)
            message(
              STATUS
              "${lib} searched on ${_LIBDIR_HINTS} FOUND. ${lib} at ${abs_lib_${lib}}"
            )
          else()
            message(STATUS "${lib} FOUND. ${lib} at ${abs_lib_${lib}}")
          endif()
          list(APPEND _PACKAGE_CONFIG_LIBRARIES "${abs_lib_${lib}}")
        endif()
        unset(abs_lib_${lib} CACHE)
        continue()
      else() # This is an absolute lib
        list(APPEND _PACKAGE_CONFIG_LIBRARIES "${component}")
      endif()
      continue()
    endif()
    # Else, this is just an absolute lib
    if(EXISTS "${component}")
      list(APPEND _PACKAGE_CONFIG_LIBRARIES "${component}")
    endif()
  endforeach()
endif(_hpp-constraints_PKG_CONFIG_LIBS_LIST)

set("hpp-constraints_LIBRARIES" ${_PACKAGE_CONFIG_LIBRARIES})
set("HPP_CONSTRAINTS_LIBRARIES" ${_PACKAGE_CONFIG_LIBRARIES})


include("${CMAKE_CURRENT_LIST_DIR}/cxx-standard.cmake")



include(CMakeFindDependencyMacro)
if(${CMAKE_VERSION} VERSION_LESS "3.15.0")
  find_package(hpp-pinocchio REQUIRED)
else()
  find_dependency(hpp-pinocchio REQUIRED)
endif()

if(COMMAND ADD_REQUIRED_DEPENDENCY)
  foreach(pkgcfg_dep ${hpp-constraints_PKG_CONFIG_DEPENDENCIES})
    # Avoid duplicated lookup.
    list(FIND $_PKG_CONFIG_REQUIRES "${pkgcfg_dep}" _index)
    if(${_index} EQUAL -1)
      ADD_REQUIRED_DEPENDENCY(${pkgcfg_dep})
    endif()
  endforeach()
endif(COMMAND ADD_REQUIRED_DEPENDENCY)

include("${CMAKE_CURRENT_LIST_DIR}/hpp-constraintsTargets.cmake")

foreach(component ${hpp-constraints_FIND_COMPONENTS})
  set(comp_file "${CMAKE_CURRENT_LIST_DIR}/${component}Config.cmake")
  if(EXISTS ${comp_file})
    include(${comp_file})
  else()
    set(hpp-constraints_${component}_FOUND FALSE)
  endif()
  if(hpp-constraints_${component}_FOUND)
    message(STATUS "hpp-constraints: ${component} found.")
  else()
    if(hpp-constraints_FIND_REQUIRED_${component})
      message(FATAL_ERROR "hpp-constraints: ${component} not found.")
    else()
      message(STATUS "hpp-constraints: ${component} not found.")
    endif()
  endif()
endforeach()
check_required_components("hpp-constraints")


CHECK_MINIMAL_CXX_STANDARD(14 ENFORCE)
