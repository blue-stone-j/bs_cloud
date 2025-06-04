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

set(modules_directory ${CMAKE_CURRENT_SOURCE_DIR}/modules)

include_directories(
  ${modules_directory}
)

function(get_functions module_directory function_names function_files)

    set(function_names_local "")
    set(function_files_local "")
    file(GLOB functions RELATIVE "${module_directory}" "${module_directory}/*")
    set(function_directories "")
    foreach(function ${functions})
        if(IS_DIRECTORY "${module_directory}/${function}" AND 
           NOT function STREQUAL "test" AND 
           NOT function STREQUAL "common" AND 
           NOT function STREQUAL "docs")
            # message(STATUS "${module_directory}: ${function}")
            list(APPEND function_names_local "${function}")
            file(GLOB_RECURSE temp_cpp_files "${module_directory}/${function}/*.cpp")
            # message(STATUS "${module_directory}/${function} : ${temp_cpp_files}")
            list(APPEND function_files_local ${temp_cpp_files})
        endif()
    endforeach()
    # message(STATUS "${module_directory} : ${function_files_local}")
    set(function_names ${function_names_local} PARENT_SCOPE)
    set(function_files ${function_files_local} PARENT_SCOPE)

endfunction()

function(compile_module module_name module_include module_lib)

    set(function_names "")
    set(function_files "")
    get_functions(${modules_directory}/${module_name} function_names function_files)

    add_library(${module_name} SHARED ${function_files})
    target_link_libraries(${module_name} ${module_lib})
    target_include_directories(${module_name} PUBLIC ./include  ${module_include})
    install(TARGETS ${module_name} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/)


    if(BUILD_TEST)
        foreach(function_name ${function_names})
            # message(WARNING "Compiling test for function: ${function_name} in module: ${module_name}")
            message(STATUS "${modules_directory}/${module_name}/${function_name}")
            file(GLOB_RECURSE function_cpp_files "${modules_directory}/${module_name}/${function_name}/*.cpp")
            add_executable(${module_name}_${function_name}_test
                           ${modules_directory}/${module_name}/test/${function_name}_test.cpp 
                           ${function_cpp_files})
            target_link_libraries(${module_name}_${function_name}_test ${module_lib} gtest gtest_main)
            target_include_directories(${module_name}_${function_name}_test PUBLIC ./include ${module_include})
            # Register test
            add_test(NAME ${function_name}Test COMMAND ${module_name}_${function_name}_test)
        endforeach()
    endif()

endfunction()

file(GLOB sub_elements RELATIVE "${modules_directory}" "${modules_directory}/*")
set(module_directories "")
foreach(sub_element ${sub_elements})
    if(IS_DIRECTORY "${modules_directory}/${sub_element}" AND EXISTS "${modules_directory}/${sub_element}/CMakeLists.txt")        
        list(APPEND module_directories "${modules_directory}/${sub_element}")
        add_subdirectory("${modules_directory}/${sub_element}")
    endif()
endforeach()

include(cmake/global_definition.cmake)

include(cmake/install_files.cmake)