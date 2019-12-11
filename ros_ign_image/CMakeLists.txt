cmake_minimum_required(VERSION 3.5)

project(ros_ign_image)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(ros_ign_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Citadel
find_package(ignition-transport8 QUIET)
if (ignition-transport8_FOUND)
  set(IGN_TRANSPORT_VER ${ignition-transport8_VERSION_MAJOR})

  find_package(ignition-msgs5 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs5_VERSION_MAJOR})

# Blueprint
else()
  find_package(ignition-transport7 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport7_VERSION_MAJOR})

  find_package(ignition-msgs4 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs4_VERSION_MAJOR})
endif()

include_directories(include)

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

ament_target_dependencies(${executable}
  "image_transport"
  "ros_ign_bridge"
  "rclcpp"
  "sensor_msgs"
)

install(TARGETS ${executable}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
