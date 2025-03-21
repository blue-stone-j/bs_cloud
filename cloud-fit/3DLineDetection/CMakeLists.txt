cmake_minimum_required(VERSION 3.10.2)
project(line_detection)

add_compile_options(-std=c++17)
ADD_DEFINITIONS(-DUNICODE)

find_package(OpenCV REQUIRED)


include_directories(
  include
)

# message(FATAL_ERROR ${OpenCV_INCLUDE_DIRS} )

add_executable(${PROJECT_NAME}_node src/main.cpp src/CommonFunctions.cpp src/LineDetection3D.cpp)
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
target_include_directories(${PROJECT_NAME}_node PUBLIC
 ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_node
  ${OpenCV_LIBRARIES}
)


# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_line_detection.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
