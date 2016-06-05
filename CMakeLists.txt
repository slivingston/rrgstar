cmake_minimum_required (VERSION 2.8.3)
project (rrglib)

install (DIRECTORY src/ DESTINATION include/rrglib
         FILES_MATCHING
         PATTERN "*.h"
         PATTERN "*.hpp")

add_compile_options (-g
    -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE -D__STDC_FORMAT_MACROS
    -Wall
    )

add_subdirectory (src)
add_subdirectory (examples/rrg)
add_subdirectory (examples/rrgstar)

find_package (Doxygen)
if (${DOXYGEN_FOUND})
    add_custom_target (doc ${DOXYGEN_EXECUTABLE}
                       ${CMAKE_SOURCE_DIR}/doc/rrglib.doxyconf
                       WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    set_property (DIRECTORY PROPERTY ADDITIONAL_MAKE_CLEAN_FILES html)
else  ()
    message ("Doxygen not found. The manual will not be built.")
endif ()
