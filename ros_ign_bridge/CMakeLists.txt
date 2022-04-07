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
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ros_ign_interfaces REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Fortress
if("$ENV{IGNITION_VERSION}" STREQUAL "fortress")
  find_package(ignition-transport11 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

  find_package(ignition-msgs8 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Fortress")
# Default to Edifice
else()
  find_package(ignition-transport10 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport10_VERSION_MAJOR})

  find_package(ignition-msgs7 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs7_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Edifice")
endif()

set(PACKAGES
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_msgs
  ros_ign_interfaces
  rosgraph_msgs
  sensor_msgs
  trajectory_msgs
)

set(bridge_lib ros_ign_bridge_lib)

set(${bridge_lib}_SRCS
  src/convert/utils.cpp
  src/bridge_config.cpp
  src/bridge_handle.cpp
  src/bridge_handle_ros_to_ign.cpp
  src/bridge_handle_ign_to_ros.cpp
  src/factories.cpp
  src/factory_interface.cpp
  src/ros_ign_bridge.cpp)


foreach(PKG ${PACKAGES})
  list(APPEND ${bridge_lib}_SRCS src/convert/${PKG}.cpp)
  list(APPEND ${bridge_lib}_SRCS src/factories/${PKG}.cpp)

endforeach()

add_library(${bridge_lib} SHARED ${${bridge_lib}_SRCS})

ament_target_dependencies(${bridge_lib}
  rclcpp
  rclcpp_components
  yaml_cpp_vendor
  ${PACKAGES}
)

target_include_directories(${bridge_lib}
  PUBLIC include
  PRIVATE src
)

target_link_libraries(${bridge_lib}
  ignition-msgs${IGN_MSGS_VER}::core
  ignition-transport${IGN_TRANSPORT_VER}::core
)

rclcpp_components_register_node(
  ${bridge_lib}
  PLUGIN ros_ign_bridge::RosIgnBridge
  EXECUTABLE bridge_node)

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

set(bridge_executables static_bridge parameter_bridge)

foreach(bridge ${bridge_executables})
  add_executable(${bridge}
    src/${bridge}.cpp
  )
  target_link_libraries(${bridge}
    ${bridge_lib}
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

  add_library(test_utils
    test/utils/ign_test_msg.cpp
    test/utils/ros_test_msg.cpp
  )

  target_include_directories(test_utils
    PUBLIC test ${GTEST_INCLUDE_DIRS}
  )
  target_link_libraries(test_utils
    ${GTEST_LIBRARIES}
    ignition-msgs${IGN_MSGS_VER}::core
    ignition-transport${IGN_TRANSPORT_VER}::core
  )
  ament_target_dependencies(test_utils
    geometry_msgs
    nav_msgs
    rclcpp
    ros_ign_interfaces
    rosgraph_msgs
    sensor_msgs
    std_msgs
    tf2_msgs
    trajectory_msgs
  )

  add_executable(test_ros_publisher test/publishers/ros_publisher.cpp)
  target_link_libraries(test_ros_publisher test_utils)

  add_executable(test_ros_subscriber
        test/subscribers/ros_subscriber/ros_subscriber.cpp
        test/subscribers/ros_subscriber/geometry_msgs_subscriber.cpp
        test/subscribers/ros_subscriber/nav_msgs_subscriber.cpp
        test/subscribers/ros_subscriber/ros_ign_interfaces_subscriber.cpp
        test/subscribers/ros_subscriber/rosgraph_msgs_subscriber.cpp
        test/subscribers/ros_subscriber/sensor_msgs_subscriber.cpp
        test/subscribers/ros_subscriber/std_msgs_subscriber.cpp
        test/subscribers/ros_subscriber/trajectory_msgs_subscriber.cpp)

  target_link_libraries(test_ros_subscriber test_utils)

  add_executable(test_ign_publisher test/publishers/ign_publisher.cpp)
  target_link_libraries(test_ign_publisher test_utils)

  add_executable(test_ign_subscriber test/subscribers/ign_subscriber.cpp)
  target_link_libraries(test_ign_subscriber test_utils)

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

  ament_add_gtest(${PROJECT_NAME}_bridge_config test/bridge_config.cpp
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  target_include_directories(${PROJECT_NAME}_bridge_config PRIVATE src)
  ament_target_dependencies(${PROJECT_NAME}_bridge_config rclcpp)
  target_link_libraries(${PROJECT_NAME}_bridge_config ${bridge_lib})

endif()

ament_export_dependencies(
  rclcpp
  ${PACKAGES}
)

ament_package()
