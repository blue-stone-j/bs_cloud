
add_library(plane_fit SHARED
  src/plane.cpp)
target_link_libraries(plane_fit
  ${PCL_LIBRARIES}
)
install(TARGETS plane_fit
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

if(BUILD_TEST)
  add_executable(plane_test test/plane_test.cpp src/plane.cpp)
  target_link_libraries(plane_test ${PCL_LIBRARIES} gtest gtest_main)
endif()