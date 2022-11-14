cmake_minimum_required(VERSION 3.5)

project(ros_gz_bridge)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

if(NOT DEFINED ENV{GZ_VERSION})
  set(ENV{GZ_VERSION} "garden")
endif()

# Garden
if("$ENV{GZ_VERSION}" STREQUAL "garden")
  find_package(gz-transport12 REQUIRED)
  find_package(gz-msgs9 REQUIRED)

  set(GZ_TARGET_PREFIX gz)
  set(GZ_MSGS_VER ${gz-msgs9_VERSION_MAJOR})
  set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})

  message(STATUS "Compiling against Gazebo Garden")
else()
  message(FATAL_ERROR "This branch is only compatible with Gazebo Garden forward.")
endif()

set(GZ_MSGS_VERSION_MAJOR ${${GZ_TARGET_PREFIX}-msgs${GZ_MSGS_VER}_VERSION_MAJOR})
set(GZ_MSGS_VERSION_MINOR ${${GZ_TARGET_PREFIX}-msgs${GZ_MSGS_VER}_VERSION_MINOR})
set(GZ_MSGS_VERSION_PATCH ${${GZ_TARGET_PREFIX}-msgs${GZ_MSGS_VER}_VERSION_PATCH})
set(GZ_MSGS_VERSION_FULL ${GZ_MSGS_VERSION_MAJOR}.${GZ_MSGS_VERSION_MINOR}.${GZ_MSGS_VERSION_PATCH})

set(BRIDGE_MESSAGE_TYPES
  builtin_interfaces
  geometry_msgs
  gps_msgs
  nav_msgs
  rcl_interfaces
  ros_gz_interfaces
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
  "bin/ros_gz_bridge_generate_factories"
  "resource/get_factory.cpp.em"
  "resource/get_mappings.cpp.em"
  "resource/pkg_factories.cpp.em"
  "resource/pkg_factories.hpp.em"
  "ros_gz_bridge/__init__.py"
  "ros_gz_bridge/mappings.py")

find_package(Python3 REQUIRED COMPONENTS Interpreter)

add_custom_command(
  OUTPUT ${generated_files}
  COMMAND Python3::Interpreter
  ARGS bin/ros_gz_bridge_generate_factories
    --output-path "${generated_path}" --template-dir resource
    --gz-msgs-ver "${GZ_MSGS_VERSION_FULL}"
  DEPENDS ${target_dependencies}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Generating factories for interface types")

set(bridge_lib
  ros_gz_bridge_lib
)

add_library(${bridge_lib}
  SHARED
  src/bridge_config.cpp
  src/bridge_handle.cpp
  src/bridge_handle_ros_to_gz.cpp
  src/bridge_handle_gz_to_ros.cpp
  src/factory_interface.cpp
  src/service_factories/ros_gz_interfaces.cpp
  src/ros_gz_bridge.cpp
  ${convert_files}
  ${generated_files}
)

target_link_libraries(${bridge_lib}
  ${GZ_TARGET_PREFIX}-msgs${GZ_MSGS_VER}::core
  ${GZ_TARGET_PREFIX}-transport${GZ_TRANSPORT_VER}::core
)

ament_target_dependencies(${bridge_lib}
  rclcpp
  rclcpp_components
  yaml_cpp_vendor
  ${BRIDGE_MESSAGE_TYPES}
)

target_include_directories(${bridge_lib}
  PUBLIC include
  PRIVATE src ${generated_path}
)

target_link_libraries(${bridge_lib}
  ${GZ_TARGET_PREFIX}-msgs${GZ_MSGS_VER}::core
  ${GZ_TARGET_PREFIX}-transport${GZ_TRANSPORT_VER}::core
)

rclcpp_components_register_node(
  ${bridge_lib}
  PLUGIN ros_gz_bridge::RosGzBridge
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

set(bridge_executables
  parameter_bridge
  static_bridge
  bridge_types
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

  ament_add_gtest(test_rcl_interfaces
    ${PROJECT_SOURCE_DIR}/src/convert/rcl_interfaces.cpp
    ${PROJECT_SOURCE_DIR}/src/convert/rcl_interfaces_TEST.cpp
  )
  target_link_libraries(test_rcl_interfaces
    ${GZ_TARGET_PREFIX}-msgs${GZ_MSGS_VER}::core
    ${rcl_interfaces_TARGETS}
    gtest
    gtest_main
  )
  target_include_directories(test_rcl_interfaces
    PRIVATE
    ${PROJECT_SOURCE_DIR}/include
  )

  ament_add_gtest(test_ros_gz_interfaces
    ${PROJECT_SOURCE_DIR}/src/convert/ros_gz_interfaces.cpp
    ${PROJECT_SOURCE_DIR}/src/convert/ros_gz_interfaces_TEST.cpp
  )
  target_link_libraries(test_ros_gz_interfaces
    ${bridge_lib}
    ${ros_gz_interfaces_TARGETS}
    gtest
    gtest_main
  )
  target_include_directories(test_ros_gz_interfaces
    PRIVATE
    ${PROJECT_SOURCE_DIR}/include
  )

  add_library(test_utils
    test/utils/gz_test_msg.cpp
    test/utils/ros_test_msg.cpp
  )

  target_include_directories(test_utils
    PUBLIC test ${GTEST_INCLUDE_DIRS} include
  )
  target_link_libraries(test_utils
    ${GTEST_LIBRARIES}
    ${GZ_TARGET_PREFIX}-msgs${GZ_MSGS_VER}::core
    ${GZ_TARGET_PREFIX}-transport${GZ_TRANSPORT_VER}::core
  )
  ament_target_dependencies(test_utils
    rclcpp
    ${BRIDGE_MESSAGE_TYPES}
    ros_gz_interfaces
    rosgraph_msgs
    sensor_msgs
    std_msgs
    tf2_msgs
    trajectory_msgs
  )

  set(generated_path "${CMAKE_BINARY_DIR}/generated/test")

  set(generated_files
    "${generated_path}/gz_publisher.cpp"
    "${generated_path}/gz_subscriber.cpp"
    "${generated_path}/ros_publisher.cpp"
  )

  set(ros_subscriber_files
    "test/subscribers/ros_subscriber.cpp"
  )

  foreach(package_name ${BRIDGE_MESSAGE_TYPES})
    list(APPEND generated_files "${generated_path}/${package_name}_subscriber.cpp")
    list(APPEND ros_subscriber_files "${generated_path}/${package_name}_subscriber.cpp")
  endforeach()

  set(target_dependencies
    "bin/ros_gz_bridge_generate_factories"
    "test/resource/gz_publisher.cpp.em"
    "test/resource/gz_subscriber.cpp.em"
    "test/resource/ros_publisher.cpp.em"
    "test/resource/ros_pkg_subscriber.cpp.em"
    "ros_gz_bridge/__init__.py"
    "ros_gz_bridge/mappings.py")

  add_custom_command(
    OUTPUT ${generated_files}
    COMMAND Python3::Interpreter
    ARGS bin/ros_gz_bridge_generate_tests
      --output-path "${generated_path}" --template-dir test/resource/
      --gz-msgs-ver "${GZ_MSGS_VERSION_FULL}"
    DEPENDS ${target_dependencies}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Generating test helpers")

  add_executable(test_gz_publisher ${generated_path}/gz_publisher.cpp)
  target_link_libraries(test_gz_publisher test_utils)

  add_executable(test_ros_publisher ${generated_path}/ros_publisher.cpp)
  target_link_libraries(test_ros_publisher test_utils)

  add_executable(test_gz_subscriber ${generated_path}/gz_subscriber.cpp)
  target_link_libraries(test_gz_subscriber test_utils)

  add_executable(test_gz_server test/serverclient/gz_server.cpp)
  target_link_libraries(test_gz_server test_utils)

  add_executable(test_ros_client test/serverclient/ros_client.cpp)
  target_link_libraries(test_ros_client test_utils)

  add_executable(test_ros_subscriber ${ros_subscriber_files})
  target_link_libraries(test_ros_subscriber test_utils)
  target_include_directories(test_ros_subscriber
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test/subscribers)

  install(TARGETS
    test_ros_publisher
    test_ros_subscriber
    test_gz_publisher
    test_gz_subscriber
    test_gz_server
    test_ros_client
    DESTINATION lib/${PROJECT_NAME}
  )

  add_launch_test(test/launch/test_ros_subscriber.launch.py
    TIMEOUT 200
    ARGS "gz_msgs_ver:=${GZ_MSGS_VERSION_FULL}"
  )

  add_launch_test(test/launch/test_gz_subscriber.launch.py
    TIMEOUT 200
    ARGS "gz_msgs_ver:=${GZ_MSGS_VERSION_FULL}"
  )

  add_launch_test(test/launch/test_gz_server.launch.py
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
  ${BRIDGE_MESSAGE_TYPES}
)

ament_package()
