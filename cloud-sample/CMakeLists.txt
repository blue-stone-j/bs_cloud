cmake_minimum_required(VERSION 3.10)
project(cloud_sample)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -pthread")

set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT DEFINED CLOUD_SAMPLE_VERSION)
  set(CLOUD_SAMPLE_VERSION_MAJOR 1)
  set(CLOUD_SAMPLE_VERSION_MINOR 0)
  set(CLOUD_SAMPLE_VERSION_PATCH 0)
  set(CLOUD_SAMPLE_VERSION ${CLOUD_SAMPLE_VERSION_MAJOR}.${CLOUD_SAMPLE_VERSION_MINOR}.${CLOUD_SAMPLE_VERSION_PATCH})
  set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install/${PROJECT_NAME}-${CLOUD_SAMPLE_VERSION})
endif()

enable_testing()

include(cmake/global_definition.cmake)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

set(common_lib "")
list(APPEND common_lib ${PCL_LIBRARIES})

include(cmake/build_libraries.cmake)

include(cmake/build_test.cmake)

include(cmake/install_files.cmake)