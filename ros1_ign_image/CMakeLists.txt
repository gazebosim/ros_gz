cmake_minimum_required(VERSION 3.5)

project(ros1_ign_image)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(catkin REQUIRED COMPONENTS
  image_transport
  ros1_ign_bridge
  roscpp
  sensor_msgs)

find_package(ignition-msgs4 QUIET REQUIRED)
set(IGN_MSGS_VER ${ignition-msgs4_VERSION_MAJOR})

find_package(ignition-transport7 QUIET REQUIRED)
set(IGN_TRANSPORT_VER ${ignition-transport7_VERSION_MAJOR})

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(executable
  image_bridge
)

add_executable(${executable}
  src/image_bridge.cpp
)
target_link_libraries(${executable}
  ${catkin_LIBRARIES}
  ignition-msgs${IGN_MSGS_VER}::core
  ignition-transport${IGN_TRANSPORT_VER}::core
)
install(TARGETS ${executable}
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

