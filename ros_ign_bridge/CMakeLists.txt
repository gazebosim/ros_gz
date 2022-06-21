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

# Fortress
if("$ENV{IGNITION_VERSION}" STREQUAL "fortress")
  find_package(ignition-transport11 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

  find_package(ignition-msgs8 REQUIRED)
  set(GZ_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})

  set(GZ_MSGS_VERSION_MAJOR ${ignition-msgs8_VERSION_MAJOR})
  set(GZ_MSGS_VERSION_MINOR ${ignition-msgs8_VERSION_MINOR})
  set(GZ_MSGS_VERSION_PATCH ${ignition-msgs8_VERSION_PATCH})

  message(STATUS "Compiling against Ignition Fortress")
# Default to Edifice
else()
  find_package(ignition-transport10 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport10_VERSION_MAJOR})

  find_package(ignition-msgs7 REQUIRED)
  set(GZ_MSGS_VER ${ignition-msgs7_VERSION_MAJOR})

  set(GZ_MSGS_VERSION_MAJOR ${ignition-msgs7_VERSION_MAJOR})
  set(GZ_MSGS_VERSION_MINOR ${ignition-msgs7_VERSION_MINOR})
  set(GZ_MSGS_VERSION_PATCH ${ignition-msgs7_VERSION_PATCH})

  message(STATUS "Compiling against Ignition Edifice")
endif()

set(GZ_MSGS_VERSION_FULL ${GZ_MSGS_VERSION_MAJOR}.${GZ_MSGS_VERSION_MINOR}.${GZ_MSGS_VERSION_PATCH})

set(BRIDGE_MESSAGE_TYPES
  geometry_msgs
  nav_msgs
  rcl_interfaces
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
    --gz-msgs-ver "${GZ_MSGS_VERSION_FULL}"
  DEPENDS ${target_dependencies}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Generating factories for interface types")

set(bridge_lib
  ros_ign_bridge_lib
)

add_library(${bridge_lib}
  SHARED
  src/bridge_config.cpp
  src/bridge_handle.cpp
  src/bridge_handle_ros_to_ign.cpp
  src/bridge_handle_ign_to_ros.cpp
  src/factory_interface.cpp
  src/ros_ign_bridge.cpp
  ${convert_files}
  ${generated_files}
)

message(STATUS ${GZ_MSGS_VER})

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
  ignition-msgs${GZ_MSGS_VER}::core
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
    ignition-msgs${GZ_MSGS_VER}::core
    ${rcl_interfaces_TARGETS}
    gtest
    gtest_main
  )
  target_include_directories(test_rcl_interfaces
    PRIVATE
    ${PROJECT_SOURCE_DIR}/include
  )

  ament_add_gtest(test_ros_ign_interfaces
    ${PROJECT_SOURCE_DIR}/src/convert/ros_ign_interfaces.cpp
    ${PROJECT_SOURCE_DIR}/src/convert/ros_ign_interfaces_TEST.cpp
  )
  target_link_libraries(test_ros_ign_interfaces
    ${bridge_lib}
    ${ros_ign_interfaces_TARGETS}
    gtest
    gtest_main
  )
  target_include_directories(test_ros_ign_interfaces
    PRIVATE
    ${PROJECT_SOURCE_DIR}/include
  )

  add_library(test_utils
    test/utils/ign_test_msg.cpp
    test/utils/ros_test_msg.cpp
  )

  target_include_directories(test_utils
    PUBLIC test ${GTEST_INCLUDE_DIRS} include
  )
  target_link_libraries(test_utils
    ${GTEST_LIBRARIES}
    ignition-msgs${GZ_MSGS_VER}::core
    ignition-transport${IGN_TRANSPORT_VER}::core
  )
  ament_target_dependencies(test_utils
    rclcpp
    ${BRIDGE_MESSAGE_TYPES}
  )

  set(generated_path "${CMAKE_BINARY_DIR}/generated/test")

  set(generated_files
    "${generated_path}/ign_publisher.cpp"
    "${generated_path}/ign_subscriber.cpp"
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
    "bin/ros_ign_bridge_generate_factories"
    "test/resource/ign_publisher.cpp.em"
    "test/resource/ign_subscriber.cpp.em"
    "test/resource/ros_publisher.cpp.em"
    "test/resource/ros_pkg_subscriber.cpp.em"
    "ros_ign_bridge/__init__.py"
    "ros_ign_bridge/mappings.py")

  add_custom_command(
    OUTPUT ${generated_files}
    COMMAND Python3::Interpreter
    ARGS bin/ros_ign_bridge_generate_tests
      --output-path "${generated_path}" --template-dir test/resource/
      --gz-msgs-ver "${GZ_MSGS_VERSION_FULL}"
    DEPENDS ${target_dependencies}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Generating test helpers")

  add_executable(test_ign_publisher ${generated_path}/ign_publisher.cpp)
  target_link_libraries(test_ign_publisher test_utils)

  add_executable(test_ros_publisher ${generated_path}/ros_publisher.cpp)
  target_link_libraries(test_ros_publisher test_utils)

  add_executable(test_ign_subscriber ${generated_path}/ign_subscriber.cpp)
  target_link_libraries(test_ign_subscriber test_utils)

  add_executable(test_ros_subscriber ${ros_subscriber_files})
  target_link_libraries(test_ros_subscriber test_utils)
  target_include_directories(test_ros_subscriber
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test/subscribers)

  install(TARGETS
    test_ros_publisher
    test_ros_subscriber
    test_ign_publisher
    test_ign_subscriber
    DESTINATION lib/${PROJECT_NAME}
  )

  add_launch_test(test/launch/test_ros_subscriber.launch.py
    TIMEOUT 200
    ARGS "gz_msgs_ver:=${GZ_MSGS_VERSION_FULL}"
  )

  add_launch_test(test/launch/test_ign_subscriber.launch.py
    TIMEOUT 200
    ARGS "gz_msgs_ver:=${GZ_MSGS_VERSION_FULL}"
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
