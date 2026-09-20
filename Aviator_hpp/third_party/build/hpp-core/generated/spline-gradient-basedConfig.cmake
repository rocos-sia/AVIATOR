
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was componentConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

####################################################################################

if(NOT hpp-core_FOUND)
  set(hpp-core_spline-gradient-based_FOUND FALSE)
  return()
endif()

# At the moment, components only support targets
#set("hpp-core_spline-gradient-based_INCLUDE_DIRS" "/home/sia/Documents/GitHub/AVIATOR/Aviator_hpp/third_party/_install/include")
#set("hpp-core_spline-gradient-based_LIBRARIES" ${_PACKAGE_CONFIG_LIBRARIES})
set(
  hpp-core_spline-gradient-based_DEPENDENCIES
  ""
)

include(CMakeFindDependencyMacro)
if(${CMAKE_VERSION} VERSION_LESS "3.15.0")
  
else()
  
endif()

include("${CMAKE_CURRENT_LIST_DIR}/spline-gradient-basedTargets.cmake")
set(hpp-core_spline-gradient-based_FOUND TRUE)


