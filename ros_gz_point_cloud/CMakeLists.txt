cmake_minimum_required(VERSION 3.5)

project(ros_ign_point_cloud)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs)

find_package(ignition-gazebo2 2.1 QUIET REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo2_VERSION_MAJOR})

find_package(ignition-rendering2 QUIET REQUIRED)
set(IGN_RENDERING_VER ${ignition-rendering2_VERSION_MAJOR})

find_package(ignition-sensors2 QUIET REQUIRED)
set(IGN_SENSORS_VER ${ignition-sensors2_VERSION_MAJOR})

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(plugin_name RosIgnPointCloud)
add_library(${plugin_name} SHARED
  src/point_cloud.cc
)
target_link_libraries(${plugin_name}
  ignition-gazebo${IGN_GAZEBO_VER}::core
  ignition-rendering${IGN_RENDERING_VER}::core
  ignition-sensors${IGN_SENSORS_VER}::core
  ${catkin_LIBRARIES}
)
install(TARGETS ${plugin_name}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY
    examples/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples
)

