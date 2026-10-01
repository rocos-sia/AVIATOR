# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "inspire_hand: 6 messages, 26 services")

set(MSG_I_FLAGS "-Iinspire_hand:/home/cindy/try_ws/src/inspire_hand/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(inspire_hand_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/get_force_act_1.msg" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/msg/get_force_act_1.msg" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/get_angle_act_1.msg" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/msg/get_angle_act_1.msg" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/get_touch_act_1.msg" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/msg/get_touch_act_1.msg" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/set_angle_1.msg" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/msg/set_angle_1.msg" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/set_force_1.msg" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/msg/set_force_1.msg" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/set_speed_1.msg" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/msg/set_speed_1.msg" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_id.srv" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/srv/set_id.srv" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_redu_ratio.srv" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/srv/set_redu_ratio.srv" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_clear_error.srv" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/srv/set_clear_error.srv" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_save_flash.srv" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/srv/set_save_flash.srv" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_reset_para.srv" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/srv/set_reset_para.srv" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_force_clb.srv" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/srv/set_force_clb.srv" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_gesture_no.srv" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/srv/set_gesture_no.srv" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_current_limit.srv" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/srv/set_current_limit.srv" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_default_speed.srv" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/srv/set_default_speed.srv" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_default_force.srv" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/srv/set_default_force.srv" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_user_def_angle.srv" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/srv/set_user_def_angle.srv" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_pos.srv" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/srv/set_pos.srv" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_angle.srv" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/srv/set_angle.srv" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_angle_1.srv" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/srv/set_angle_1.srv" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_force.srv" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/srv/set_force.srv" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_speed.srv" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/srv/set_speed.srv" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_pos_act.srv" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/srv/get_pos_act.srv" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_angle_act.srv" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/srv/get_angle_act.srv" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_force_act.srv" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/srv/get_force_act.srv" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_current.srv" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/srv/get_current.srv" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_error.srv" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/srv/get_error.srv" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_status.srv" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/srv/get_status.srv" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_temp.srv" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/srv/get_temp.srv" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_pos_set.srv" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/srv/get_pos_set.srv" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_angle_set.srv" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/srv/get_angle_set.srv" ""
)

get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_force_set.srv" NAME_WE)
add_custom_target(_inspire_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "inspire_hand" "/home/cindy/try_ws/src/inspire_hand/srv/get_force_set.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/get_force_act_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_msg_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/get_angle_act_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_msg_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/get_touch_act_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_msg_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/set_angle_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_msg_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/set_force_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_msg_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/set_speed_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)

### Generating Services
_generate_srv_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_id.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_srv_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_redu_ratio.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_srv_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_clear_error.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_srv_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_save_flash.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_srv_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_reset_para.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_srv_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_force_clb.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_srv_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_gesture_no.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_srv_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_current_limit.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_srv_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_default_speed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_srv_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_default_force.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_srv_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_user_def_angle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_srv_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_pos.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_srv_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_angle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_srv_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_angle_1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_srv_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_force.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_srv_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_speed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_srv_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_pos_act.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_srv_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_angle_act.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_srv_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_force_act.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_srv_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_current.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_srv_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_error.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_srv_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_status.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_srv_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_temp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_srv_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_pos_set.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_srv_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_angle_set.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)
_generate_srv_cpp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_force_set.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
)

### Generating Module File
_generate_module_cpp(inspire_hand
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(inspire_hand_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(inspire_hand_generate_messages inspire_hand_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/get_force_act_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/get_angle_act_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/get_touch_act_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/set_angle_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/set_force_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/set_speed_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_id.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_redu_ratio.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_clear_error.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_save_flash.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_reset_para.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_force_clb.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_gesture_no.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_current_limit.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_default_speed.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_default_force.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_user_def_angle.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_pos.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_angle.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_angle_1.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_force.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_speed.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_pos_act.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_angle_act.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_force_act.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_current.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_error.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_status.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_temp.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_pos_set.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_angle_set.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_force_set.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_cpp _inspire_hand_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(inspire_hand_gencpp)
add_dependencies(inspire_hand_gencpp inspire_hand_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS inspire_hand_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/get_force_act_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_msg_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/get_angle_act_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_msg_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/get_touch_act_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_msg_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/set_angle_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_msg_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/set_force_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_msg_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/set_speed_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)

### Generating Services
_generate_srv_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_id.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_srv_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_redu_ratio.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_srv_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_clear_error.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_srv_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_save_flash.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_srv_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_reset_para.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_srv_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_force_clb.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_srv_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_gesture_no.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_srv_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_current_limit.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_srv_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_default_speed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_srv_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_default_force.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_srv_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_user_def_angle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_srv_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_pos.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_srv_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_angle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_srv_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_angle_1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_srv_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_force.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_srv_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_speed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_srv_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_pos_act.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_srv_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_angle_act.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_srv_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_force_act.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_srv_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_current.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_srv_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_error.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_srv_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_status.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_srv_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_temp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_srv_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_pos_set.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_srv_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_angle_set.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)
_generate_srv_eus(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_force_set.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
)

### Generating Module File
_generate_module_eus(inspire_hand
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(inspire_hand_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(inspire_hand_generate_messages inspire_hand_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/get_force_act_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/get_angle_act_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/get_touch_act_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/set_angle_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/set_force_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/set_speed_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_id.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_redu_ratio.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_clear_error.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_save_flash.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_reset_para.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_force_clb.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_gesture_no.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_current_limit.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_default_speed.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_default_force.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_user_def_angle.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_pos.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_angle.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_angle_1.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_force.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_speed.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_pos_act.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_angle_act.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_force_act.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_current.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_error.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_status.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_temp.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_pos_set.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_angle_set.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_force_set.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_eus _inspire_hand_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(inspire_hand_geneus)
add_dependencies(inspire_hand_geneus inspire_hand_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS inspire_hand_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/get_force_act_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_msg_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/get_angle_act_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_msg_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/get_touch_act_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_msg_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/set_angle_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_msg_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/set_force_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_msg_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/set_speed_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)

### Generating Services
_generate_srv_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_id.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_srv_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_redu_ratio.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_srv_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_clear_error.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_srv_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_save_flash.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_srv_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_reset_para.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_srv_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_force_clb.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_srv_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_gesture_no.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_srv_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_current_limit.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_srv_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_default_speed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_srv_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_default_force.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_srv_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_user_def_angle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_srv_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_pos.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_srv_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_angle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_srv_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_angle_1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_srv_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_force.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_srv_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_speed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_srv_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_pos_act.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_srv_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_angle_act.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_srv_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_force_act.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_srv_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_current.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_srv_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_error.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_srv_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_status.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_srv_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_temp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_srv_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_pos_set.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_srv_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_angle_set.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)
_generate_srv_lisp(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_force_set.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
)

### Generating Module File
_generate_module_lisp(inspire_hand
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(inspire_hand_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(inspire_hand_generate_messages inspire_hand_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/get_force_act_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/get_angle_act_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/get_touch_act_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/set_angle_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/set_force_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/set_speed_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_id.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_redu_ratio.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_clear_error.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_save_flash.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_reset_para.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_force_clb.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_gesture_no.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_current_limit.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_default_speed.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_default_force.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_user_def_angle.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_pos.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_angle.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_angle_1.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_force.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_speed.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_pos_act.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_angle_act.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_force_act.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_current.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_error.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_status.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_temp.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_pos_set.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_angle_set.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_force_set.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_lisp _inspire_hand_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(inspire_hand_genlisp)
add_dependencies(inspire_hand_genlisp inspire_hand_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS inspire_hand_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/get_force_act_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_msg_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/get_angle_act_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_msg_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/get_touch_act_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_msg_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/set_angle_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_msg_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/set_force_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_msg_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/set_speed_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)

### Generating Services
_generate_srv_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_id.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_srv_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_redu_ratio.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_srv_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_clear_error.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_srv_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_save_flash.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_srv_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_reset_para.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_srv_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_force_clb.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_srv_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_gesture_no.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_srv_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_current_limit.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_srv_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_default_speed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_srv_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_default_force.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_srv_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_user_def_angle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_srv_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_pos.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_srv_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_angle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_srv_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_angle_1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_srv_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_force.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_srv_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_speed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_srv_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_pos_act.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_srv_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_angle_act.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_srv_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_force_act.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_srv_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_current.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_srv_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_error.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_srv_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_status.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_srv_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_temp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_srv_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_pos_set.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_srv_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_angle_set.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)
_generate_srv_nodejs(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_force_set.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
)

### Generating Module File
_generate_module_nodejs(inspire_hand
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(inspire_hand_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(inspire_hand_generate_messages inspire_hand_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/get_force_act_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/get_angle_act_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/get_touch_act_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/set_angle_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/set_force_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/set_speed_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_id.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_redu_ratio.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_clear_error.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_save_flash.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_reset_para.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_force_clb.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_gesture_no.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_current_limit.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_default_speed.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_default_force.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_user_def_angle.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_pos.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_angle.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_angle_1.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_force.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_speed.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_pos_act.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_angle_act.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_force_act.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_current.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_error.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_status.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_temp.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_pos_set.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_angle_set.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_force_set.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_nodejs _inspire_hand_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(inspire_hand_gennodejs)
add_dependencies(inspire_hand_gennodejs inspire_hand_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS inspire_hand_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/get_force_act_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_msg_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/get_angle_act_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_msg_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/get_touch_act_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_msg_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/set_angle_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_msg_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/set_force_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_msg_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/msg/set_speed_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)

### Generating Services
_generate_srv_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_id.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_srv_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_redu_ratio.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_srv_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_clear_error.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_srv_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_save_flash.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_srv_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_reset_para.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_srv_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_force_clb.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_srv_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_gesture_no.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_srv_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_current_limit.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_srv_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_default_speed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_srv_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_default_force.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_srv_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_user_def_angle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_srv_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_pos.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_srv_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_angle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_srv_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_angle_1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_srv_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_force.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_srv_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/set_speed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_srv_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_pos_act.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_srv_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_angle_act.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_srv_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_force_act.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_srv_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_current.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_srv_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_error.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_srv_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_status.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_srv_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_temp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_srv_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_pos_set.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_srv_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_angle_set.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)
_generate_srv_py(inspire_hand
  "/home/cindy/try_ws/src/inspire_hand/srv/get_force_set.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
)

### Generating Module File
_generate_module_py(inspire_hand
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(inspire_hand_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(inspire_hand_generate_messages inspire_hand_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/get_force_act_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/get_angle_act_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/get_touch_act_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/set_angle_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/set_force_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/msg/set_speed_1.msg" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_id.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_redu_ratio.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_clear_error.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_save_flash.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_reset_para.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_force_clb.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_gesture_no.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_current_limit.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_default_speed.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_default_force.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_user_def_angle.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_pos.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_angle.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_angle_1.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_force.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/set_speed.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_pos_act.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_angle_act.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_force_act.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_current.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_error.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_status.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_temp.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_pos_set.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_angle_set.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cindy/try_ws/src/inspire_hand/srv/get_force_set.srv" NAME_WE)
add_dependencies(inspire_hand_generate_messages_py _inspire_hand_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(inspire_hand_genpy)
add_dependencies(inspire_hand_genpy inspire_hand_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS inspire_hand_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/inspire_hand
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(inspire_hand_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/inspire_hand
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(inspire_hand_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/inspire_hand
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(inspire_hand_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/inspire_hand
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(inspire_hand_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/inspire_hand
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(inspire_hand_generate_messages_py std_msgs_generate_messages_py)
endif()
