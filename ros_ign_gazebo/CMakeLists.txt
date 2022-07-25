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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# TODO(CH3): Deprecated. Remove on tock.
if("$ENV{GZ_VERSION}" STREQUAL "" AND NOT "$ENV{IGNITION_VERSION}" STREQUAL "")
  message(DEPRECATION "Environment variable [IGNITION_VERSION] is deprecated. Use [GZ_VERSION] instead.")
  set(ENV{GZ_VERSION} ENV{IGNITION_VERSION})
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
endif()

ign_find_package(gflags
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

configure_file(
  launch/ign_gazebo.launch.py.in
  launch/ign_gazebo.launch.py.configured
  @ONLY
)
file(GENERATE
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/launch/ign_gazebo.launch.py"
  INPUT "${CMAKE_CURRENT_BINARY_DIR}/launch/ign_gazebo.launch.py.configured"
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/launch/ign_gazebo.launch.py"
  DESTINATION share/${PROJECT_NAME}/launch
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
