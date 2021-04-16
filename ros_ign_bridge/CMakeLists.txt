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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Citadel
if("$ENV{IGNITION_VERSION}" STREQUAL "citadel")
  find_package(ignition-transport8 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport8_VERSION_MAJOR})

  find_package(ignition-msgs5 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs5_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Citadel")
# Dome
elseif("$ENV{IGNITION_VERSION}" STREQUAL "dome")
  find_package(ignition-transport9 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport9_VERSION_MAJOR})

  find_package(ignition-msgs6 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs6_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Dome")
# Default to Edifice
else()
  find_package(ignition-transport10 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport10_VERSION_MAJOR})

  find_package(ignition-msgs7 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs7_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Edifice")
endif()

include_directories(include)

set(common_sources
  src/convert.cpp
  src/factories.cpp
)

set(bridge_executables
  parameter_bridge
  static_bridge
)

set(bridge_lib
  ros_ign_bridge_lib
)

add_library(${bridge_lib}
  ${common_sources}
)

target_link_libraries(${bridge_lib}
  ignition-msgs${IGN_MSGS_VER}::core
  ignition-transport${IGN_TRANSPORT_VER}::core
)
ament_target_dependencies(${bridge_lib}
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "rosgraph_msgs"
  "sensor_msgs"
  "std_msgs"
  "tf2_msgs"
  "trajectory_msgs"
)

install(TARGETS ${bridge_lib}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${bridge_lib})

foreach(bridge ${bridge_executables})
  add_executable(${bridge}
    src/${bridge}.cpp
    ${common_sources}
  )
  target_link_libraries(${bridge}
    ignition-msgs${IGN_MSGS_VER}::core
    ignition-transport${IGN_TRANSPORT_VER}::core
  )
  ament_target_dependencies(${bridge}
    "geometry_msgs"
    "nav_msgs"
    "rclcpp"
    "rosgraph_msgs"
    "sensor_msgs"
    "std_msgs"
    "tf2_msgs"
    "trajectory_msgs"
  )
  install(TARGETS ${bridge}
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_find_gtest()

  add_executable(test_ros_publisher test/publishers/ros_publisher.cpp)
  ament_target_dependencies(test_ros_publisher
    geometry_msgs
    nav_msgs
    rclcpp
    rosgraph_msgs
    sensor_msgs
    std_msgs
    tf2_msgs
    trajectory_msgs
  )
  target_link_libraries(test_ros_publisher
    ${GTEST_LIBRARIES}
    ignition-msgs${IGN_MSGS_VER}::core
    ignition-transport${IGN_TRANSPORT_VER}::core
  )
  target_include_directories(test_ros_publisher PUBLIC ${GTEST_INCLUDE_DIRS})

  add_executable(test_ros_subscriber test/subscribers/ign_subscriber.cpp)
  ament_target_dependencies(test_ros_subscriber
    geometry_msgs
    nav_msgs
    rclcpp
    rosgraph_msgs
    sensor_msgs
    std_msgs
    tf2_msgs
    trajectory_msgs
  )

  target_link_libraries(test_ros_subscriber
    ${GTEST_LIBRARIES}
    ignition-msgs${IGN_MSGS_VER}::core
    ignition-transport${IGN_TRANSPORT_VER}::core
  )
  target_include_directories(test_ros_subscriber PUBLIC ${GTEST_INCLUDE_DIRS})

  add_executable(test_ign_publisher test/publishers/ign_publisher.cpp)
  ament_target_dependencies(test_ign_publisher
    rclcpp
  )
  target_link_libraries(test_ign_publisher
    ${GTEST_LIBRARIES}
    ignition-msgs${IGN_MSGS_VER}::core
    ignition-transport${IGN_TRANSPORT_VER}::core
  )
  target_include_directories(test_ign_publisher PUBLIC ${GTEST_INCLUDE_DIRS})

  add_executable(test_ign_subscriber test/subscribers/ign_subscriber.cpp)
  ament_target_dependencies(test_ign_subscriber
    rclcpp
  )
  target_link_libraries(test_ign_subscriber
    ${GTEST_LIBRARIES}
    ignition-msgs${IGN_MSGS_VER}::core
    ignition-transport${IGN_TRANSPORT_VER}::core
  )
  target_include_directories(test_ign_subscriber PUBLIC ${GTEST_INCLUDE_DIRS})

  install(TARGETS
    test_ros_publisher
    test_ros_subscriber
    test_ign_publisher
    test_ign_subscriber
    DESTINATION lib/${PROJECT_NAME}
  )

  add_launch_test(test/launch/test_ros_subscriber.launch.py
    TIMEOUT 200
  )

  add_launch_test(test/launch/test_ign_subscriber.launch.py
    TIMEOUT 200
  )
endif()

ament_package()
