cmake_minimum_required(VERSION 3.5)

project(ros_gz_sim)

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
find_package(std_msgs REQUIRED)

if(NOT DEFINED ENV{GZ_VERSION})
  set(ENV{GZ_VERSION} "garden")
endif()

if("$ENV{GZ_VERSION}" STREQUAL "garden")
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
else()
  message(FATAL_ERROR "This branch is only compatible with Gazebo Garden forward.")
endif()

gz_find_package(gflags
    REQUIRED
    PKGCONFIG gflags)
find_package(std_msgs REQUIRED)

add_executable(create src/create.cpp)
ament_target_dependencies(create
  rclcpp
  std_msgs
)
target_link_libraries(create
  gflags
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
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

add_library(gzserver_component SHARED src/gzserver.cpp)
rclcpp_components_register_nodes(gzserver_component "ros_gz_sim::GzServer")
ament_target_dependencies(gzserver_component
  rclcpp
  rclcpp_components
  std_msgs
)
target_link_libraries(gzserver_component
  ${GZ_TARGET_PREFIX}-sim${GZ_SIM_VER}::core
)
ament_target_dependencies(gzserver_component std_msgs)
rclcpp_components_register_node(
  gzserver_component
  PLUGIN "ros_gz_sim::GzServer"
  EXECUTABLE gzserver
)

configure_file(
  launch/gz_sim.launch.py.in
  launch/gz_sim.launch.py.configured
  @ONLY
)
file(GENERATE
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/launch/gz_sim.launch.py"
  INPUT "${CMAKE_CURRENT_BINARY_DIR}/launch/gz_sim.launch.py.configured"
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/launch/gz_sim.launch.py"
  DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES
  "launch/gzserver.launch"
  "launch/gzserver.launch.py"
  "launch/gz_spawn_model.launch.py"
  "launch/ros_gz_sim.launch.py"
  "launch/ros_gz_spawn_model.launch.py"
  DESTINATION share/${PROJECT_NAME}/launch
)

install(TARGETS
  create
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_gzserver_component)
install(TARGETS gzserver_component
        EXPORT export_gzserver_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
  TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(${PROJECT_NAME})

ament_export_dependencies(rclcpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # GTest
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_find_gtest()

  ament_add_gtest_executable(test_stopwatch
    test/test_stopwatch.cpp
  )
  ament_add_gtest_executable(test_create
    test/test_create.cpp
  )

  ament_target_dependencies(test_stopwatch rclcpp)

  target_include_directories(test_stopwatch PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(test_stopwatch
    ${PROJECT_NAME}
  )
  target_link_libraries(test_create
    ${GZ_TARGET_PREFIX}-transport${GZ_TRANSPORT_VER}::core
  )

  install(
    TARGETS test_stopwatch test_create
    DESTINATION lib/${PROJECT_NAME}
  )
  ament_add_gtest_test(test_stopwatch)
  add_launch_test(test/test_create_node.launch.py TIMEOUT 200)
endif()

ament_package()
