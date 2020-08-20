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
find_package(ignition-math6 REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ignition-transport8 QUIET)
find_package(ignition-transport9 QUIET)
# Citadel
if(ignition-transport8_FOUND)
  set(IGN_TRANSPORT_VER ${ignition-transport8_VERSION_MAJOR})

  find_package(ignition-msgs5 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs5_VERSION_MAJOR})

# Dome
elseif(ignition-transport9_FOUND)
  set(IGN_TRANSPORT_VER ${ignition-transport9_VERSION_MAJOR})

  find_package(ignition-msgs6 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs6_VERSION_MAJOR})

# Blueprint
else()
  find_package(ignition-transport7 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport7_VERSION_MAJOR})

  find_package(ignition-msgs4 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs4_VERSION_MAJOR})
endif()

ign_find_package(gflags
    REQUIRED
    PKGCONFIG gflags)

add_executable(create src/create.cpp)
ament_target_dependencies(create
  rclcpp
  ignition-math6
)
target_link_libraries(create
  gflags
  ignition-msgs${IGN_MSGS_VER}::core
  ignition-transport${IGN_TRANSPORT_VER}::core
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  create
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
