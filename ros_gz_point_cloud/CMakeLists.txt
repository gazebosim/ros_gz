cmake_minimum_required(VERSION 3.5)

project(ros_gz_point_cloud)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs)

find_package(ignition-gazebo2 2.1 QUIET REQUIRED)
set(GZ_SIM_VER ${ignition-gazebo2_VERSION_MAJOR})

find_package(ignition-rendering2 QUIET REQUIRED)
set(GZ_RENDERING_VER ${ignition-rendering2_VERSION_MAJOR})

find_package(ignition-sensors2 QUIET REQUIRED)
set(GZ_SENSORS_VER ${ignition-sensors2_VERSION_MAJOR})

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(plugin_name RosGzPointCloud)
add_library(${plugin_name} SHARED
  src/point_cloud.cc
)
target_link_libraries(${plugin_name}
  ignition-gazebo${GZ_SIM_VER}::core
  ignition-rendering${GZ_RENDERING_VER}::core
  ignition-sensors${GZ_SENSORS_VER}::core
  ${catkin_LIBRARIES}
)
install(TARGETS ${plugin_name}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY
    examples/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples
)

# TODO(CH3): Install symlink to deprecated library name. Remove on tock.
string(REPLACE "RosGz" "RosIgn" plugin_name_ign ${plugin_name})
if(WIN32)
  # symlinks on Windows require admin priviledges, fallback to copy
  ADD_CUSTOM_COMMAND(TARGET ${plugin_name} POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E copy
      "$<TARGET_FILE:${plugin_name}>"
      "$<TARGET_FILE_DIR:${CATKIN_PACKAGE_LIB_DESTINATION}>/${plugin_name_ign}")
else()
  file(MAKE_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
  EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E create_symlink ${plugin_name} ${plugin_name_ign} WORKING_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
  INSTALL(FILES ${PROJECT_BINARY_DIR}/lib/${plugin_name_ign} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
endif()
