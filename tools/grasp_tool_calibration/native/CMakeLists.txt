if(NOT TARGET aviator_core_control)
  return()
endif()

add_executable(aviator_grasp_tool_state main.cpp)
target_link_libraries(aviator_grasp_tool_state PRIVATE aviator_core_control)

if(AVIATOR_BUILD_XCORE_SDK)
  # The vendor archive has Eigen/KDL symbols that must not interpose on Core's
  # Pinocchio/PIN-IK code. Keep SDK types behind a shared-library boundary.
  add_library(aviator_grasp_tool_sdk SHARED rokae_joint_reader.cpp)
  target_include_directories(aviator_grasp_tool_sdk PRIVATE
    ${AVIATOR_VENDOR_DIR}/xCoreSDK-0.7.1/include)
  target_link_libraries(aviator_grasp_tool_sdk PRIVATE
    Eigen3::Eigen Rokae::Rokae Threads::Threads ${CMAKE_DL_LIBS})
  target_link_options(aviator_grasp_tool_sdk PRIVATE
    "-Wl,--exclude-libs,ALL" "-Wl,-Bsymbolic")
  target_link_libraries(aviator_grasp_tool_state PRIVATE aviator_grasp_tool_sdk)
  target_compile_definitions(aviator_grasp_tool_state PRIVATE AVIATOR_GRASP_HAVE_ROKAE=1)
  install(TARGETS aviator_grasp_tool_sdk LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()

install(TARGETS aviator_grasp_tool_state RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
