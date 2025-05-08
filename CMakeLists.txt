cmake_minimum_required(VERSION 3.16.3)
project(bcloud)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -pthread")

set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT DEFINED BCLOUD_VERSION)
  set(BCLOUD_VERSION_MAJOR 1)
  set(BCLOUD_VERSION_MINOR 0)
  set(BCLOUD_VERSION_PATCH 0)
  set(BCLOUD_VERSION ${BCLOUD_VERSION_MAJOR}.${BCLOUD_VERSION_MINOR}.${BCLOUD_VERSION_PATCH})
  set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install/${PROJECT_NAME}-${BCLOUD_VERSION})
endif()

enable_testing()

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

function(get_functions module_directory function_names function_files)

    set(function_names_local "")
    set(function_files_local "")
    file(GLOB children RELATIVE "${module_directory}" "${module_directory}/*")
    set(function_directories "")
    foreach(child ${children})
        if(IS_DIRECTORY "${module_directory}/${child}" AND NOT child STREQUAL "test" AND NOT child STREQUAL "cmake" AND NOT child STREQUAL "common" AND NOT child STREQUAL "docs")
            list(APPEND function_names_local "${child}")
            file(GLOB_RECURSE tmp_cpp_files "${module_directory}/${child}/*.cpp")
            list(APPEND function_files_local ${tmp_cpp_files})
        endif()
    endforeach()

    set(function_names ${function_names_local} PARENT_SCOPE)
    set(function_files ${function_files_local} PARENT_SCOPE)

endfunction()

function(compile_module module_name module_include module_lib)

    set(function_names "")
    set(function_files "")
    get_functions(${CMAKE_CURRENT_SOURCE_DIR} function_names function_files)

    add_library(${module_name} SHARED ${function_files})
    target_link_libraries(${module_name} ${module_lib})
    target_include_directories(${module_name} PUBLIC ./include  ${module_include})
    install(TARGETS ${module_name} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/)


    if(BUILD_TEST)
        foreach(function_name ${function_names})
            add_executable(${module_name}_${function_name}_test test/${function_name}_test.cpp ${function_name}/${function_name}.cpp)
            target_link_libraries(${module_name}_${function_name}_test ${module_lib} gtest gtest_main)
            target_include_directories(${module_name}_${function_name}_test PUBLIC ./include ${module_include})
            # Register test
            add_test(NAME ${function_name}Test COMMAND ${module_name}_${function_name}_test)
        endforeach()
    endif()

endfunction()

file(GLOB sub_elements RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/*")
set(module_directories "")
foreach(sub_element ${sub_elements})
    if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${sub_element}" AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${sub_element}/CMakeLists.txt")        
        list(APPEND module_directories "${CMAKE_CURRENT_SOURCE_DIR}/${sub_element}")
        add_subdirectory(${sub_element})
    endif()
endforeach()

include(cmake/global_definition.cmake)

include(cmake/install_files.cmake)