if(BUILD_TEST)
  foreach(CPP_FILE ${CPP_FILES})
    # Get the filename without extension
    get_filename_component(FILENAME_NO_EXT ${CPP_FILE} NAME_WE)

    add_executable(${FILENAME_NO_EXT}_test test/${FILENAME_NO_EXT}_test.cpp src/${FILENAME_NO_EXT}.cpp)
    target_link_libraries(${FILENAME_NO_EXT}_test ${common_lib} gtest gtest_main)
    # Register test
    add_test(NAME ${FILENAME_NO_EXT}Test COMMAND ${FILENAME_NO_EXT}_test)
  endforeach()
endif()