cmake_minimum_required(VERSION 3.5)

project(ros_gz_sim)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(CLI11 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(rclcpp REQUIRED)
<<<<<<< HEAD
=======
find_package(rclcpp_components REQUIRED)
find_package(rcpputils REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
>>>>>>> 388d931 (Spawn, set pose and delete entities using ROS 2 (#705))
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

<<<<<<< HEAD
# TODO(CH3): Deprecated. Remove on tock.
if("$ENV{GZ_VERSION}" STREQUAL "" AND NOT "$ENV{IGNITION_VERSION}" STREQUAL "")
  message(DEPRECATION "Environment variable [IGNITION_VERSION] is deprecated. Use [GZ_VERSION] instead.")
  set(ENV{GZ_VERSION} $ENV{IGNITION_VERSION})
endif()

# Edifice
if("$ENV{GZ_VERSION}" STREQUAL "edifice")
  find_package(ignition-math6 REQUIRED)
  set(GZ_MATH_VER ${ignition-math6_VERSION_MAJOR})

  find_package(ignition-transport10 REQUIRED)
  set(GZ_TRANSPORT_VER ${ignition-transport10_VERSION_MAJOR})

  find_package(ignition-msgs7 REQUIRED)
  set(GZ_MSGS_VER ${ignition-msgs7_VERSION_MAJOR})

  find_package(ignition-gazebo5 REQUIRED)
  set(GZ_SIM_VER ${ignition-gazebo5_VERSION_MAJOR})

  set(GZ_TARGET_PREFIX ignition)
  macro(gz_find_package)
    ign_find_package(${ARGV})
  endmacro()

  message(STATUS "Compiling against Gazebo Edifice")
# Garden
elseif("$ENV{GZ_VERSION}" STREQUAL "garden")
  find_package(gz-math7 REQUIRED)
  set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})

  find_package(gz-transport12 REQUIRED)
  set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})

  find_package(gz-msgs9 REQUIRED)
  set(GZ_MSGS_VER ${gz-msgs9_VERSION_MAJOR})

  find_package(gz-sim7 REQUIRED)
  set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

  set(GZ_TARGET_PREFIX gz)

  message(STATUS "Compiling against Gazebo Garden")
elseif("$ENV{GZ_VERSION}" STREQUAL "harmonic")
  find_package(gz-math7 REQUIRED)
  set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})

  find_package(gz-transport13 REQUIRED)
  set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

  find_package(gz-msgs10 REQUIRED)
  set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

  find_package(gz-sim8 REQUIRED)
  set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

  set(GZ_TARGET_PREFIX gz)

  message(STATUS "Compiling against Gazebo Harmonic")
# Default to Fortress
else()
  find_package(ignition-math6 REQUIRED)
  set(GZ_MATH_VER ${ignition-math6_VERSION_MAJOR})

  find_package(ignition-transport11 REQUIRED)
  set(GZ_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

  find_package(ignition-msgs8 REQUIRED)
  set(GZ_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})

  find_package(ignition-gazebo6 REQUIRED)
  set(GZ_SIM_VER ${ignition-gazebo6_VERSION_MAJOR})

  set(GZ_TARGET_PREFIX ignition)

  message(STATUS "Compiling against Gazebo Fortress")
  macro(gz_find_package)
    ign_find_package(${ARGV})
  endmacro()
endif()
=======
# Needed in launch/gz_sim.launch.py.in
set(GZ_SIM_VER ${gz-sim_VERSION_MAJOR})
>>>>>>> 388d931 (Spawn, set pose and delete entities using ROS 2 (#705))

gz_find_package(gflags REQUIRED PKGCONFIG gflags)

<<<<<<< HEAD
=======
# Install the python module for this package
ament_python_install_package(${PROJECT_NAME})

# create
>>>>>>> 388d931 (Spawn, set pose and delete entities using ROS 2 (#705))
add_executable(create src/create.cpp)
ament_target_dependencies(create
  rclcpp
  std_msgs
)
target_link_libraries(create
  gflags
<<<<<<< HEAD
  ${GZ_TARGET_PREFIX}-math${GZ_MATH_VER}::core
  ${GZ_TARGET_PREFIX}-msgs${GZ_MSGS_VER}::core
  ${GZ_TARGET_PREFIX}-transport${GZ_TRANSPORT_VER}::core
)
ament_target_dependencies(create std_msgs)

add_library(${PROJECT_NAME} SHARED src/Stopwatch.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
=======
  gz-math::core
  gz-msgs::core
  gz-transport::core
  rclcpp::rclcpp
  rcpputils::rcpputils
  ${std_msgs_TARGETS}
)

# delete_entity library
add_library(${PROJECT_NAME}_delete_entity SHARED src/delete_entity.cpp)
target_link_libraries(${PROJECT_NAME}_delete_entity
                      ${builtin_interfaces_TARGETS}
                      ${ros_gz_interfaces_TARGETS}
                      rclcpp::rclcpp)
target_include_directories(
  ${PROJECT_NAME}_delete_entity
  PUBLIC
>>>>>>> 388d931 (Spawn, set pose and delete entities using ROS 2 (#705))
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> ${CLI11_INCLUDE_DIRS})

# delete_entity
add_executable(delete_entity src/delete_entity.cpp)
target_link_libraries(delete_entity ${PROJECT_NAME}_delete_entity
                      ${builtin_interfaces_TARGETS}
                      ${geometry_msgs_TARGETS}
                      ${ros_gz_interfaces_TARGETS}
                      rclcpp::rclcpp)
target_include_directories(delete_entity PUBLIC ${CLI11_INCLUDE_DIRS})

<<<<<<< HEAD
configure_file(
  launch/gz_sim.launch.py.in
  launch/gz_sim.launch.py.configured
  @ONLY
)
file(GENERATE
=======
# gzserver_component
add_library(gzserver_component SHARED src/gzserver.cpp)
rclcpp_components_register_nodes(gzserver_component "ros_gz_sim::GzServer")
target_link_libraries(gzserver_component PUBLIC
  ${std_msgs_TARGETS}
  gz-sim::core
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
)
rclcpp_components_register_node(
  gzserver_component
  PLUGIN "ros_gz_sim::GzServer"
  EXECUTABLE gzserver
)
target_include_directories(
  gzserver_component
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# spawn_entity library
add_library(${PROJECT_NAME}_spawn_entity SHARED src/spawn_entity.cpp)
target_link_libraries(${PROJECT_NAME}_spawn_entity
                      ${builtin_interfaces_TARGETS}
                      ${geometry_msgs_TARGETS}
                      ${ros_gz_interfaces_TARGETS}
                      rclcpp::rclcpp
                      tf2::tf2
                      tf2_ros::tf2_ros)
target_include_directories(
  ${PROJECT_NAME}_spawn_entity
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> ${CLI11_INCLUDE_DIRS})

# set_entity_pose library
add_library(${PROJECT_NAME}_set_entity_pose SHARED src/set_entity_pose.cpp)
target_link_libraries(${PROJECT_NAME}_set_entity_pose
                      ${builtin_interfaces_TARGETS}
                      ${geometry_msgs_TARGETS}
                      ${ros_gz_interfaces_TARGETS}
                      rclcpp::rclcpp
                      tf2::tf2
                      tf2_ros::tf2_ros)
target_include_directories(
  ${PROJECT_NAME}_set_entity_pose
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> ${CLI11_INCLUDE_DIRS})

# project library (Stopwatch)
add_library(${PROJECT_NAME} SHARED src/Stopwatch.cpp)
target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
)
target_include_directories(
  ${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# remove
add_executable(remove src/remove.cpp)
target_link_libraries(remove
  gz-msgs::core
  gz-transport::core
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)

# set_entity_pose
add_executable(set_entity_pose src/set_entity_pose.cpp)
target_include_directories(set_entity_pose PUBLIC ${CLI11_INCLUDE_DIRS})
target_link_libraries(set_entity_pose ${PROJECT_NAME}_set_entity_pose
                      ${builtin_interfaces_TARGETS}
                      ${geometry_msgs_TARGETS}
                      ${ros_gz_interfaces_TARGETS}
                      rclcpp::rclcpp
                      tf2::tf2
                      tf2_ros::tf2_ros)

# spawn_entity executable
add_executable(spawn_entity src/spawn_entity.cpp)
target_link_libraries(spawn_entity ${PROJECT_NAME}_spawn_entity
                      ${builtin_interfaces_TARGETS}
                      ${geometry_msgs_TARGETS}
                      ${ros_gz_interfaces_TARGETS}
                      rclcpp::rclcpp
                      tf2::tf2
                      tf2_ros::tf2_ros)

# Headers
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

# Launch files
configure_file(launch/gz_sim.launch.py.in launch/gz_sim.launch.py.configured
  @ONLY)
file(
  GENERATE
>>>>>>> 388d931 (Spawn, set pose and delete entities using ROS 2 (#705))
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/launch/gz_sim.launch.py"
  INPUT "${CMAKE_CURRENT_BINARY_DIR}/launch/gz_sim.launch.py.configured")

<<<<<<< HEAD
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/launch/gz_sim.launch.py"
  DESTINATION share/${PROJECT_NAME}/launch
)

install(TARGETS
  create
  DESTINATION lib/${PROJECT_NAME}
)
=======
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/launch/gz_sim.launch.py"
        DESTINATION share/${PROJECT_NAME}/launch)
>>>>>>> 388d931 (Spawn, set pose and delete entities using ROS 2 (#705))

install(
  FILES launch/gz_remove_model.launch.py
        launch/gz_server.launch
        launch/gz_server.launch.py
        launch/gz_spawn_model.launch
        launch/gz_spawn_model.launch.py
        launch/ros_gz_sim.launch
        launch/ros_gz_sim.launch.py
        launch/ros_gz_spawn_model.launch.py
  DESTINATION share/${PROJECT_NAME}/launch)

# Install targets
install(TARGETS
    create
    delete_entity
    remove
    set_entity_pose
    spawn_entity
  DESTINATION
    lib/${PROJECT_NAME})

install(TARGETS gzserver DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_gzserver_component)
install(
  TARGETS gzserver_component
  EXPORT  export_gzserver_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  TARGETS ${PROJECT_NAME}
  ${PROJECT_NAME}_spawn_entity
  ${PROJECT_NAME}_set_entity_pose
  ${PROJECT_NAME}_delete_entity
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(rclcpp tf2 tf2_ros)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # GTest
  find_package(ament_cmake_gtest REQUIRED)
  ament_find_gtest()

<<<<<<< HEAD
  ament_add_gtest_executable(test_stopwatch
    test/test_stopwatch.cpp
  )

  ament_target_dependencies(test_stopwatch rclcpp)
=======
  ament_add_gtest_executable(test_create test/test_create.cpp)
  ament_add_gtest_executable(test_remove test/test_remove.cpp)
  ament_add_gtest_executable(test_spawn_entity test/test_spawn_entity.cpp)
  ament_add_gtest_executable(test_set_entity_pose test/test_set_entity_pose.cpp)
  ament_add_gtest_executable(test_delete_entity test/test_delete_entity.cpp)
  ament_add_gtest_executable(test_stopwatch test/test_stopwatch.cpp)

  target_include_directories(
    test_stopwatch PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                          $<INSTALL_INTERFACE:include>)
  target_include_directories(
    test_spawn_entity
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_include_directories(
    test_set_entity_pose
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_include_directories(
    test_delete_entity
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
>>>>>>> 388d931 (Spawn, set pose and delete entities using ROS 2 (#705))

  target_link_libraries(test_stopwatch ${PROJECT_NAME} rclcpp::rclcpp)
  target_link_libraries(test_create gz-transport::core)
  target_link_libraries(test_remove gz-transport::core)
  target_link_libraries(test_spawn_entity ${PROJECT_NAME}_spawn_entity
                        ${builtin_interfaces_TARGETS}
                        ${geometry_msgs_TARGETS}
                        ${ros_gz_interfaces_TARGETS}
                        rclcpp::rclcpp
                        tf2::tf2
                        tf2_ros::tf2_ros)
  target_link_libraries(test_set_entity_pose ${PROJECT_NAME}_set_entity_pose
                        ${builtin_interfaces_TARGETS}
                        ${geometry_msgs_TARGETS}
                        ${ros_gz_interfaces_TARGETS}
                        rclcpp::rclcpp
                        tf2::tf2
                        tf2_ros::tf2_ros)
  target_link_libraries(test_delete_entity ${PROJECT_NAME}_delete_entity
                        ${builtin_interfaces_TARGETS}
                        ${ros_gz_interfaces_TARGETS}
                        rclcpp::rclcpp)

<<<<<<< HEAD
  target_link_libraries(test_stopwatch
    ${PROJECT_NAME}
  )

  install(
    TARGETS test_stopwatch
    DESTINATION lib/${PROJECT_NAME}
  )
  ament_add_gtest_test(test_stopwatch)
=======
  install(TARGETS
      test_create
      test_remove
      test_spawn_entity
      test_set_entity_pose
      test_delete_entity
      test_stopwatch
    DESTINATION
      lib/${PROJECT_NAME})

  ament_add_gtest_test(test_stopwatch)
  ament_add_gtest_test(test_spawn_entity)
  ament_add_gtest_test(test_set_entity_pose)
  ament_add_gtest_test(test_delete_entity)

  add_launch_test(test/test_create_node.launch.py TIMEOUT 200)
  add_launch_test(test/test_remove_node.launch.py TIMEOUT 200)
>>>>>>> 388d931 (Spawn, set pose and delete entities using ROS 2 (#705))
endif()

ament_package()
