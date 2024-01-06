cmake_minimum_required(VERSION 3.5)

project(ros_ign_gazebo)

find_package(catkin
  REQUIRED
  COMPONENTS
    roscpp
)

# Garden
if ("$ENV{IGNITION_VERSION}" STREQUAL "garden")
  find_package(ignition-transport12 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport12_VERSION_MAJOR})

  find_package(ignition-msgs9 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs9_VERSION_MAJOR})

  find_package(ignition-gazebo7 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo7_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Garden")
# Fortress
elseif ("$ENV{IGNITION_VERSION}" STREQUAL "fortress")
  find_package(ignition-transport11 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

  find_package(ignition-msgs8 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})

  find_package(ignition-gazebo6 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Fortress")
# Edifice
elseif ("$ENV{IGNITION_VERSION}" STREQUAL "edifice")
  find_package(ignition-transport10 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport10_VERSION_MAJOR})

  find_package(ignition-msgs7 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs7_VERSION_MAJOR})

  find_package(ignition-gazebo5 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo5_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Edifice")
# Default to Citadel
else()
  find_package(ignition-transport8 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport8_VERSION_MAJOR})

  find_package(ignition-msgs5 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs5_VERSION_MAJOR})

  find_package(ignition-gazebo3 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo3_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Citadel")
endif()

ign_find_package(gflags
    REQUIRED
    PKGCONFIG gflags)

catkin_package()


include_directories(include ${catkin_INCLUDE_DIRS})

set(cpp_execs
  create
)

foreach(exec ${cpp_execs})
  add_executable(${exec} src/${exec}.cpp)
  target_link_libraries(${exec}
    ${catkin_LIBRARIES}
    gflags
    ignition-msgs${IGN_MSGS_VER}::core
    ignition-transport${IGN_TRANSPORT_VER}::core
  )
  install(TARGETS ${exec}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endforeach()

install(
  PROGRAMS
    scripts/ign_gazebo
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
  DIRECTORY
    launch/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

