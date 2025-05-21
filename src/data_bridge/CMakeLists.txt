cmake_minimum_required(VERSION 3.10)
project(data_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)

# set up examples_utils library
add_library(examples_common STATIC
  src/examples_common.cpp
)
target_include_directories(examples_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(examples_common PUBLIC Franka::Franka)

# build ft sensor
add_subdirectory(src/net_ft)

set(EXAMPLES
  data_node
  cartesian_impedance_control
  white_light
  admitance
)

foreach(example ${EXAMPLES})
  add_executable(${example} src/${example}.cpp)

  # include directories
  target_include_directories(${example} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

  # link libraries
  target_link_libraries(${example} Franka::Franka examples_common ft_interface)

  # c and cpp standards
  target_compile_features(${example} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

  ament_target_dependencies(${example}
    rclcpp
    geometry_msgs
    std_msgs
  )
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