# Python-only Modbus TCP adapter.  It is independent from the SocketCAN hand node.
find_package(Python3 QUIET COMPONENTS Interpreter)

install(PROGRAMS rh56ftp_node.py fake_rh56ftp_hand.py
  DESTINATION ${CMAKE_INSTALL_DATADIR}/aviator/rh56ftp_hand)
install(FILES README.md
  DESTINATION ${CMAKE_INSTALL_DATADIR}/aviator/rh56ftp_hand)
install(FILES ${PROJECT_SOURCE_DIR}/config/rh56ftp_hand.yaml
  DESTINATION ${CMAKE_INSTALL_DATADIR}/aviator/config)
install(FILES
  ${PROJECT_SOURCE_DIR}/third_party/RH56FTP/python/pendant/__init__.py
  ${PROJECT_SOURCE_DIR}/third_party/RH56FTP/python/pendant/handlink.py
  DESTINATION ${CMAKE_INSTALL_DATADIR}/aviator/rh56ftp_hand/pendant)

if(BUILD_TESTING AND Python3_Interpreter_FOUND)
  add_test(NAME rh56ftp_node_unit
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_rh56ftp_node.py)
  set_tests_properties(rh56ftp_node_unit PROPERTIES TIMEOUT 15)
  add_test(NAME fake_rh56ftp_hand
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_fake_rh56ftp_hand.py)
  set_tests_properties(fake_rh56ftp_hand PROPERTIES TIMEOUT 20)
endif()
