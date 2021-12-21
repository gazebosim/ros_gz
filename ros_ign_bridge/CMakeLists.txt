cmake_minimum_required(VERSION 3.5)

project(ros_ign_bridge)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(catkin REQUIRED COMPONENTS
               geometry_msgs
               rosconsole
               roscpp
               rostest
               sensor_msgs
               std_msgs
               tf2_msgs
               visualization_msgs)

# Fortress
if ("$ENV{IGNITION_VERSION}" STREQUAL "fortress")
  find_package(ignition-transport11 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

  find_package(ignition-msgs8 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Fortress")
# Edifice
elseif ("$ENV{IGNITION_VERSION}" STREQUAL "edifice")
  find_package(ignition-transport10 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport10_VERSION_MAJOR})

  find_package(ignition-msgs7 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs7_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Edifice")
# Dome
elseif ("$ENV{IGNITION_VERSION}" STREQUAL "dome")
  find_package(ignition-transport9 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport9_VERSION_MAJOR})

  find_package(ignition-msgs6 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs6_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Dome")
# Default to Citadel
else()
  find_package(ignition-transport8 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport8_VERSION_MAJOR})

  find_package(ignition-msgs5 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs5_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Citadel")
endif()

catkin_package()

include_directories(include src ${catkin_INCLUDE_DIRS})

set(common_sources
  src/factories.cpp
  src/convert/frame_id.cpp
  src/convert/geometry_msgs.cpp
  src/convert/nav_msgs.cpp
  src/convert/rosgraph_msgs.cpp
  src/convert/sensor_msgs.cpp
  src/convert/std_msgs.cpp
  src/convert/visualization_msgs.cpp
  src/factories/geometry_msgs.cpp
  src/factories/nav_msgs.cpp
  src/factories/rosgraph_msgs.cpp
  src/factories/sensor_msgs.cpp
  src/factories/std_msgs.cpp
  src/factories/visualization_msgs.cpp
)

set(bridge_lib
  ros_ign_bridge_lib
)

add_library(${bridge_lib}
  ${common_sources}
)
target_link_libraries(${bridge_lib}
  ${catkin_LIBRARIES}
  ignition-msgs${IGN_MSGS_VER}::core
  ignition-transport${IGN_TRANSPORT_VER}::core
)

set(bridge_lib
  ros_ign_bridge_lib
)

add_library(${bridge_lib}
  ${common_sources}
)
target_link_libraries(${bridge_lib}
  ${catkin_LIBRARIES}
  ignition-msgs${IGN_MSGS_VER}::core
  ignition-transport${IGN_TRANSPORT_VER}::core
)

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
  install(TARGETS ${bridge}
          DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endforeach(bridge)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${bridge_lib})

install(TARGETS ${bridge_lib}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  # Tests
  find_package(rostest REQUIRED)
  set(test_publishers
    ign_publisher
    ros_publisher
  )

  set(test_subscribers
    ign_subscriber
    ros_subscriber
  )

  foreach(test_publisher ${test_publishers})
    add_executable(${test_publisher}
      test/publishers/${test_publisher}.cpp
    )
    target_link_libraries(${test_publisher}
      ${catkin_LIBRARIES}
      ignition-msgs${IGN_MSGS_VER}::core
      ignition-transport${IGN_TRANSPORT_VER}::core
      gtest
      gtest_main
    )
  endforeach(test_publisher)

  foreach(test_subscriber ${test_subscribers})
    add_rostest_gtest(test_${test_subscriber}
      test/${test_subscriber}.test
      test/subscribers/${test_subscriber}.cpp)
    target_link_libraries(test_${test_subscriber}
      ${catkin_LIBRARIES}
      ignition-msgs${IGN_MSGS_VER}::core
      ignition-transport${IGN_TRANSPORT_VER}::core
    )
  endforeach(test_subscriber)
endif()
