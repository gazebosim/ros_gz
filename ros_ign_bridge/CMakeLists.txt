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

set(BRIDGE_MESSAGE_TYPES
  geometry_msgs
  nav_msgs
  ros_ign_interfaces
  rosgraph_msgs
  sensor_msgs
  std_msgs
  tf2_msgs
  trajectory_msgs
)

set(generated_path "${CMAKE_BINARY_DIR}/generated")
set(generated_files "${generated_path}/get_factory.cpp")
list(APPEND generated_files "${generated_path}/get_mappings.cpp")
set(convert_files "src/convert/utils.cpp")

foreach(package_name ${BRIDGE_MESSAGE_TYPES})
  find_package(${package_name} QUIET REQUIRED)
  message(STATUS "Found ${package_name}: ${${package_name}_VERSION} (${${package_name}_DIR})")

  list(APPEND generated_files "${generated_path}/factories/${package_name}.cpp")
  list(APPEND generated_files "${generated_path}/factories/${package_name}.hpp")

  list(APPEND convert_files "src/convert/${package_name}.cpp")
endforeach()

set(target_dependencies
  "bin/ros_ign_bridge_generate_factories"
  "resource/get_factory.cpp.em"
  "resource/get_mappings.cpp.em"
  "resource/pkg_factories.cpp.em"
  "resource/pkg_factories.hpp.em"
  "ros_ign_bridge/__init__.py"
  "ros_ign_bridge/mappings.py")

find_package(Python3 REQUIRED COMPONENTS Interpreter)

add_custom_command(
  OUTPUT ${generated_files}
  COMMAND Python3::Interpreter
  ARGS bin/ros_ign_bridge_generate_factories
    --output-path "${generated_path}" --template-dir resource
  DEPENDS ${target_dependencies}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Generating factories for interface types")

set(bridge_lib
  ros_ign_bridge_lib
)

add_library(${bridge_lib}
  SHARED
  src/factory_interface.cpp
  ${convert_files}
  ${generated_files}
)

target_link_libraries(${bridge_lib}
    ignition-msgs${IGN_MSGS_VER}::core
    ignition-transport${IGN_TRANSPORT_VER}::core
)

ament_target_dependencies(${bridge_lib}
  rclcpp
  ${BRIDGE_MESSAGE_TYPES}
)

target_include_directories(${bridge_lib}
  PUBLIC include
  PRIVATE src ${generated_path}
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

set(bridge_executables
  parameter_bridge
  static_bridge
)


foreach(bridge ${bridge_executables})
  add_executable(${bridge}
    src/${bridge}.cpp
  )
  target_link_libraries(${bridge}
    ${bridge_lib}
  )
  ament_target_dependencies(${bridge}
    "rclcpp"
    ${BRIDGE_MESSAGE_TYPES}
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
    rclcpp
    ${BRIDGE_MESSAGE_TYPES}
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
endif()

ament_export_dependencies(
  rclcpp
  ${PACKAGES}
)

ament_package()
