# The main launcher script currently expects the same standard resource layout.
if(NOT CMAKE_INSTALL_BINDIR STREQUAL "bin" OR NOT CMAKE_INSTALL_DATADIR STREQUAL "share")
  message(FATAL_ERROR "AVIATOR desktop launcher requires bin + share layout; use AVIATOR_INSTALL_LAUNCHER=OFF for a custom layout")
endif()
configure_file(aviator_launcher.in ${CMAKE_CURRENT_BINARY_DIR}/aviator_launcher @ONLY)
configure_file(aviator-launcher.desktop.in ${CMAKE_CURRENT_BINARY_DIR}/org.aviator.Launcher.desktop @ONLY)
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/aviator_launcher DESTINATION ${CMAKE_INSTALL_BINDIR})
install(FILES main.py app.py window.py tray.py options.py paths.py settings.py runner.py protocol.py desktop.py README.md
  DESTINATION ${CMAKE_INSTALL_DATADIR}/aviator/tools/aviator_launcher)
install(DIRECTORY assets/ DESTINATION ${CMAKE_INSTALL_DATADIR}/aviator/tools/aviator_launcher/assets)
install(FILES assets/aviator-launcher.svg DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/scalable/apps)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/org.aviator.Launcher.desktop DESTINATION ${CMAKE_INSTALL_DATADIR}/applications)
