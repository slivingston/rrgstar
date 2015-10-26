add_executable (rrg_single_integrator single_integrator.cpp)
target_link_libraries (rrg_single_integrator kdtree incmumc)

add_executable (rrg_double_integrator double_integrator.cpp)
target_link_libraries (rrg_double_integrator kdtree incmumc)

add_executable (rrg_dubins_car dubins_car.cpp)
target_link_libraries (rrg_dubins_car kdtree incmumc)

add_executable (rrg_dubins_double_integrator_airplane dubins_double_integrator_airplane.cpp)
target_link_libraries (rrg_dubins_double_integrator_airplane kdtree incmumc)

include_directories (${CMAKE_SOURCE_DIR}/src)
set_property (TARGET rrg_single_integrator rrg_double_integrator
                     rrg_dubins_car rrg_dubins_double_integrator_airplane
              PROPERTY RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
