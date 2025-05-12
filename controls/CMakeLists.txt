cmake_minimum_required(VERSION 3.10)

project(libfranka-examples CXX)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/../cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT FRANKA_IS_FOUND)
    find_package(Franka REQUIRED)
endif()
find_package(Eigen3 REQUIRED)

add_library(examples_common STATIC
  examples_common.cpp
)

target_link_libraries(examples_common PUBLIC Franka::Franka)

set(EXAMPLES
  cartesian_impedance_control
  white_light
  addmitance
)

foreach(example ${EXAMPLES})
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} Franka::Franka examples_common)
endforeach()

add_subdirectory(net_ft)

target_link_libraries(addmitance ft_interface)

include(GNUInstallDirs)
install(TARGETS ${EXAMPLES} ${ROS2_EXAMPLES}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)