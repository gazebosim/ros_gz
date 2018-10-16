cmake_minimum_required(VERSION 3.5)

project(ros1_ign_bridge)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(ignition-common2 QUIET REQUIRED)
find_package(ignition-transport5 QUIET REQUIRED)
set(IGN_COMMON_VER ${ignition-common2_VERSION_MAJOR})
set(IGN_TRANSPORT_VER ${ignition-transport5_VERSION_MAJOR})

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

set(common_sources
  src/convert_builtin_interfaces.cpp
  src/builtin_interfaces_factories.cpp
)

set(bridge_executables
  parameter_bridge
  static_bridge
)

foreach(bridge ${bridge_executables})
  add_executable(${bridge}
    src/${bridge}.cpp
    ${common_sources}
  )
  target_link_libraries(${bridge}
    ${catkin_LIBRARIES}
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-transport${IGN_TRANSPORT_VER}::core
  )
  install(TARGETS ${bridge}
          DESTINATION lib/${PROJECT_NAME}
  )
endforeach(bridge)

# Tests
find_package(rostest REQUIRED)

set(common_test_sources
  test/ros1_string_publisher.cpp
)

set(test_executables
  ros1_string_publisher
)

foreach(test_executable ${test_executables})
  add_executable(${test_executable}
    test/${test_executable}.cpp
    ${common_test_sources}
  )
  target_link_libraries(${test_executable}
    ${catkin_LIBRARIES}
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-transport${IGN_TRANSPORT_VER}::core
  )
endforeach(test_executable)

add_rostest_gtest(test_ros1_ign_string test/ros1_ign_string.test test/ign_subscriber.cpp)
target_link_libraries(test_ros1_ign_string
  ${catkin_LIBRARIES}
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  ignition-transport${IGN_TRANSPORT_VER}::core
)