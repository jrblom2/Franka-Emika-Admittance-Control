cmake_minimum_required(VERSION 3.10)
project(data_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(data_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

# build ft sensor
add_subdirectory(src/net_ft)

# set up examples_utils library
add_library(utilities STATIC
  src/utils/examples_common.cpp
  src/utils/traj_simulate.cpp
  src/utils/minimal_publisher.cpp
  src/utils/data_dumper.cpp
)
target_include_directories(utilities PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(utilities
  Franka
  rclcpp
  data_interfaces
)

set(EXAMPLES
  cartesian_impedance_control
  white_light
  admittance
  admittance_fixed
  spring_demo
  spring_demo_seperate
)

foreach(example ${EXAMPLES})
  add_executable(${example} src/${example}.cpp)

  # include directories
  target_include_directories(${example} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

  # link libraries
  target_link_libraries(${example} Franka::Franka utilities ft_interface)

  # c and cpp standards
  target_compile_features(${example} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  # where does it go? It goes in lib
  install(TARGETS ${example}
  DESTINATION lib/${PROJECT_NAME})
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()