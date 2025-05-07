
# Set the source folder
set(SOURCE_DIR ${CMAKE_SOURCE_DIR}/src)

# Get all .cpp files in the source folder
file(GLOB_RECURSE CPP_FILES ${SOURCE_DIR}/*.cpp)

# Initialize an empty list to store filenames without extensions
set(FILES_NO_EXT)

# Extract filenames without extensions and add them to the list
foreach(CPP_FILE ${CPP_FILES})
    # Get the filename without extension
    get_filename_component(FILENAME_NO_EXT ${CPP_FILE} NAME_WE)

    add_library(${FILENAME_NO_EXT} SHARED ${CPP_FILE})
    target_link_libraries(${FILENAME_NO_EXT} ${common_lib})
    install(TARGETS ${FILENAME_NO_EXT} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
endforeach()