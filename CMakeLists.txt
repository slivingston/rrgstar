cmake_minimum_required (VERSION 2.8.3)
project (rrglib)

add_subdirectory (src)
add_subdirectory (examples/rrg)
add_subdirectory (examples/rrgstar)

find_package (Doxygen)
if (${DOXYGEN_FOUND})
    add_custom_target (doc ${DOXYGEN_EXECUTABLE}
                       ${CMAKE_SOURCE_DIR}/doc_config/rrglib.doxyconf
                       WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
else  ()
    message ("Doxygen not found. The manual will not be built.")
endif ()
