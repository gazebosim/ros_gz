cmake_minimum_required(VERSION 3.5)

project(ros_gz_image)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)

find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)

set(executable
  image_bridge
)

add_executable(${executable}
  src/image_bridge.cpp
)

target_link_libraries(${executable}
  ${sensor_msgs_TARGETS}
  image_transport::image_transport
  ros_gz_bridge::ros_gz_bridge
  rclcpp::rclcpp
  gz-msgs::core
  gz-transport::core
)

install(TARGETS ${executable}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  if(UNIX)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_image_bridge_teardown
      test/test_image_bridge_teardown.cpp
      TIMEOUT 120
    )
    target_compile_definitions(test_image_bridge_teardown PRIVATE
      IMAGE_BRIDGE_EXECUTABLE="$<TARGET_FILE:${executable}>"
    )
    target_link_libraries(test_image_bridge_teardown
      gz-msgs::core
      gz-transport::core
    )
    add_dependencies(test_image_bridge_teardown ${executable})
  endif()
endif()

ament_package()
