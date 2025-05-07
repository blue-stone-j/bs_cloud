
add_library(line_fit SHARED
  src/line.cpp)
target_link_libraries(line_fit
  ${PCL_LIBRARIES}
)
install(TARGETS line_fit
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

if(BUILD_TEST)
  add_executable(line_test test/line_test.cpp src/line.cpp)
  target_link_libraries(line_test ${PCL_LIBRARIES} gtest gtest_main)
endif()