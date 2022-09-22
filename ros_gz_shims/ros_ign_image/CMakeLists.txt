cmake_minimum_required(VERSION 3.5)

project(ros_ign_image)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(image_bridge src/image_bridge_shim.cpp)
ament_target_dependencies(image_bridge ament_index_cpp)

ament_export_dependencies(ament_index_cpp ros_gz_image)

install(TARGETS
  image_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
