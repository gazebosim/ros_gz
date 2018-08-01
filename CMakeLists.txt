cmake_minimum_required(VERSION 3.5)

project(ros1_ign_bridge)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(ignition-transport5 QUIET REQUIRED)
set(IGN_TRANSPORT_VER ${ignition-transport5_VERSION_MAJOR})

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(static_bridge src/static_bridge.cpp)
target_link_libraries(static_bridge
  ${catkin_LIBRARIES}
  ignition-transport${IGN_TRANSPORT_VER}::core
)
