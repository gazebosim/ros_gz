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
endforeach(bridge)
