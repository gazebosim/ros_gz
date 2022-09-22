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
find_package(ament_index_cpp REQUIRED)

add_executable(parameter_bridge src/parameter_bridge_shim.cpp)
ament_target_dependencies(parameter_bridge ament_index_cpp)

add_executable(static_bridge src/static_bridge_shim.cpp)
ament_target_dependencies(static_bridge ament_index_cpp)

ament_export_dependencies(ament_index_cpp ros_gz_bridge)

install(TARGETS
  parameter_bridge
  static_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
