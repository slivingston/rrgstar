add_library (kdtree STATIC kdtree/kdtree.c)
add_library (incmumc STATIC inc_mu_mc/pt.cpp inc_mu_mc/ms.cpp)

install (TARGETS kdtree incmumc EXPORT rrglib ARCHIVE DESTINATION lib/rrglib)
install (EXPORT rrglib DESTINATION lib/rrglib FILE rrglibConfig.cmake)
