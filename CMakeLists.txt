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

set(test_publishers
  ign_string_publisher
  ros1_string_publisher
)

set(test_subscribers
  ign_string_subscriber
  ros1_string_subscriber
)

foreach(test_publisher ${test_publishers})
  add_executable(${test_publisher}
    test/publishers/${test_publisher}.cpp
  )
  target_link_libraries(${test_publisher}
    ${catkin_LIBRARIES}
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-transport${IGN_TRANSPORT_VER}::core
  )
endforeach(test_publisher)

foreach(test_subscriber ${test_subscribers})
  add_rostest_gtest(test_${test_subscriber}
    test/${test_subscriber}.test
    test/subscribers/${test_subscriber}.cpp)
  target_link_libraries(test_${test_subscriber}
    ${catkin_LIBRARIES}
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-transport${IGN_TRANSPORT_VER}::core
  )
endforeach(test_subscriber)