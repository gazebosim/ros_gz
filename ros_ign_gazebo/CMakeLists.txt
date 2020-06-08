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
find_package(ignition-msgs5 REQUIRED)
find_package(ignition-transport8 REQUIRED)
find_package(rclcpp REQUIRED)

ign_find_package(gflags
    REQUIRED
    PKGCONFIG gflags)

add_executable(create src/create.cpp)
ament_target_dependencies(create
  rclcpp
  ignition-math6
  ignition-msgs5
  ignition-transport8
)
target_link_libraries(create
  gflags
)

install(PROGRAMS
  scripts/ign_gazebo
  DESTINATION lib/${PROJECT_NAME}
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
