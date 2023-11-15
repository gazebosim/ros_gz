cmake_minimum_required(VERSION 3.5)
project(ros_ign_gazebo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(create
  src/create_shim.cpp
)
ament_target_dependencies(create ament_index_cpp)

ament_export_dependencies(
  ament_index_cpp
  ros_gz_bridge
)

# TODO(CH3): Deprecated. Remove on tock.
if("$ENV{GZ_VERSION}" STREQUAL "" AND NOT "$ENV{IGNITION_VERSION}" STREQUAL "")
  message(DEPRECATION "Environment variable [IGNITION_VERSION] is deprecated. Use [GZ_VERSION] instead.")
  set(ENV{GZ_VERSION} $ENV{IGNITION_VERSION})
endif()

# Edifice
if("$ENV{GZ_VERSION}" STREQUAL "edifice")
  find_package(ignition-gazebo5 REQUIRED)
  message(STATUS "Compiling against Gazebo Edifice")
# Garden
elseif("$ENV{GZ_VERSION}" STREQUAL "garden")
  find_package(gz-sim7 REQUIRED)
  message(STATUS "Compiling against Gazebo Garden")
elseif("$ENV{GZ_VERSION}" STREQUAL "harmonic")
  find_package(gz-sim8 REQUIRED)
  message(STATUS "Compiling against Gazebo Harmonic")
# Default to Fortress
else()
  find_package(ignition-gazebo6 REQUIRED)
  message(STATUS "Compiling against Gazebo Fortress")
endif()

install(FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/launch/ign_gazebo.launch.py"
  DESTINATION share/${PROJECT_NAME}/launch
)

install(TARGETS
  create
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
