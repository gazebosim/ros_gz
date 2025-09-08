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

include_directories(include)

set(executable
  image_bridge
)

add_executable(${executable}
  src/image_bridge.cpp
)

target_link_libraries(${executable}
  ${std_msgs_TARGETS}
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

  # find_package(rostest REQUIRED)
  #
  # set(test_publishers
  #   gz_publisher
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
  #     $gz-msgs$::core
  #     $gz-transport::core
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
  #     $gz-msgs::core
  #     $gz-transport::core
  #   )
  # endforeach(test_subscriber)
endif()

ament_package()
