cmake_minimum_required(VERSION 3.5)

project(ros_ign_image)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(ros_ign_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

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

include_directories(include)

set(executable
  image_bridge
)

add_executable(${executable}
  src/image_bridge.cpp
)

target_link_libraries(${executable}
  ignition-msgs${IGN_MSGS_VER}::core
  ignition-transport${IGN_TRANSPORT_VER}::core
)

ament_target_dependencies(${executable}
  "image_transport"
  "ros_ign_bridge"
  "rclcpp"
  "sensor_msgs"
)

install(TARGETS ${executable}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # find_package(rostest REQUIRED)
  #
  # set(test_publishers
  #   ign_publisher
  # )
  #
  # set(test_subscribers
  #   ros_subscriber
  # )
  #
  # foreach(test_publisher ${test_publishers})
  #   add_executable(${test_publisher}_image
  #     test/publishers/${test_publisher}.cpp
  #   )
  #   target_link_libraries(${test_publisher}_image
  #     ${catkin_LIBRARIES}
  #     ignition-msgs${IGN_MSGS_VER}::core
  #     ignition-transport${IGN_TRANSPORT_VER}::core
  #     gtest
  #     gtest_main
  #   )
  # endforeach(test_publisher)
  #
  # foreach(test_subscriber ${test_subscribers})
  #   add_rostest_gtest(test_${test_subscriber}_image
  #     test/${test_subscriber}.test
  #     test/subscribers/${test_subscriber}.cpp)
  #   target_link_libraries(test_${test_subscriber}_image
  #     ${catkin_LIBRARIES}
  #     ignition-msgs${IGN_MSGS_VER}::core
  #     ignition-transport${IGN_TRANSPORT_VER}::core
  #   )
  # endforeach(test_subscriber)
endif()

ament_package()
