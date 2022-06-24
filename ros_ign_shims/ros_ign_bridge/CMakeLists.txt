cmake_minimum_required(VERSION 3.5)

project(ros_ign_bridge)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(ros_gz_bridge REQUIRED)

message(DEPRECATION "ros_ign is deprecated. Please use ros_gz instead!")

# Symlink libs =================================================================
file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
  ${CMAKE_INSTALL_PREFIX}/lib/libros_gz_bridge_lib.so
  ${PROJECT_BINARY_DIR}/lib/libros_ign_bridge_lib.so)")
install(FILES
  ${PROJECT_BINARY_DIR}/lib/libros_ign_bridge_lib.so
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

# Symlink binaries =============================================================
set(GZ_BINS
  parameter_bridge
  static_bridge
  test_gz_publisher
  test_gz_server
  test_gz_subscriber
  test_ros_client
  test_ros_publisher
  test_ros_subscriber)

file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/lib/ros_ign_bridge)
foreach(gz_bin IN LISTS GZ_BINS)
  string(REPLACE "gz_" "ign_" ign_bin ${gz_bin})

  install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${CMAKE_INSTALL_PREFIX}/lib/ros_gz_bridge/${gz_bin}
    ${PROJECT_BINARY_DIR}/lib/ros_ign_bridge/${ign_bin})")
  install(FILES
    ${PROJECT_BINARY_DIR}/lib/ros_ign_bridge/${ign_bin}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/ros_ign_bridge)
endforeach()

# Symlink includes =============================================================
file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/include)
install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
  ${CMAKE_INSTALL_PREFIX}/include/ros_gz_bridge
  ${PROJECT_BINARY_DIR}/include/ros_ign_bridge)")
install(DIRECTORY
  ${PROJECT_BINARY_DIR}/include/ros_ign_bridge
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include)

# Export
ament_export_include_directories(include)
ament_export_libraries(ros_ign_bridge_lib)
ament_export_dependencies(
  ros_gz_bridge
)

ament_package()
