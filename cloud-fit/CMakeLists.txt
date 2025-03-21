cmake_minimum_required(VERSION 3.16)
project(cloud_fit)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -pthread")

if(NOT DEFINED CLOUD_FIT_VERSION)
  set(CLOUD_FIT_VERSION_MAJOR 1)
  set(CLOUD_FIT_VERSION_MINOR 0)
  set(CLOUD_FIT_VERSION_PATCH 0)
  set(CLOUD_FIT_VERSION ${CLOUD_FIT_VERSION_MAJOR}.${CLOUD_FIT_VERSION_MINOR}.${CLOUD_FIT_VERSION_PATCH})
  set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install/${PROJECT_NAME}-${CLOUD_FIT_VERSION})
endif()

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

include(cmake/cylinder.cmake)

include(cmake/line.cmake)

include(cmake/plane.cmake)

include(cmake/install_files.cmake)
