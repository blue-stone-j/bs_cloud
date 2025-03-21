# install header files
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include

  # FILES_MATCHING PATTERN *.h
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/README.md
  DESTINATION ${CMAKE_INSTALL_PREFIX}/cmake/${PROJECT_NAME})

# generate and install CMakeconfig files
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${CLOUD_SAMPLE_VERSION}
  COMPATIBILITY SameMajorVersion)

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake
)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_PREFIX}/cmake/${PROJECT_NAME})