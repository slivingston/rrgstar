add_executable (rrgstar_single_integrator single_integrator.cpp)
target_link_libraries (rrgstar_single_integrator kdtree incmumc)

add_executable (rrgstar_double_integrator double_integrator.cpp)
target_link_libraries (rrgstar_double_integrator kdtree incmumc)

add_executable (rrgstar_dubins_car dubins_car.cpp)
target_link_libraries (rrgstar_dubins_car kdtree incmumc)

add_executable (rrgstar_dubins_double_integrator_airplane dubins_double_integrator_airplane.cpp)
target_link_libraries (rrgstar_dubins_double_integrator_airplane kdtree incmumc)

include_directories (${CMAKE_SOURCE_DIR}/src)
set_property (TARGET rrgstar_single_integrator rrgstar_double_integrator
                     rrgstar_dubins_car rrgstar_dubins_double_integrator_airplane
              PROPERTY RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
