find_package(PkgConfig REQUIRED)
pkg_check_modules(ASIO REQUIRED asio)
pkg_check_modules(CURLPP REQUIRED curlpp)
pkg_check_modules(TINY REQUIRED tinyxml2)

add_library(ft_interface SHARED 
  net_ft_interface.cpp
  ati_ft_interface.cpp
  ati_axia_ft_interface.cpp
  hardware_interface.cpp
)
target_include_directories(ft_interface PUBLIC ${ASIO_INCLUDE_DIRS} ${CURLPP_INCLUDE_DIRS} ${TINY_INCLUDE_DIRS})
target_link_libraries(ft_interface ${ASIO_LDFLAGS} ${CURLPP_LDFLAGS} ${TINY_LDFLAGS} ${CMAKE_THREAD_LIBS_INIT})

add_executable(ft_test test_ft.cpp)
target_link_libraries(ft_test ft_interface)

install(TARGETS ft_interface
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(TARGETS ft_test
  DESTINATION lib/${PROJECT_NAME})