cmake_minimum_required(VERSION 3.5)
project(ros_ign_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/Contact.msg"
  "msg/Contacts.msg"
  "msg/Dataframe.msg"
  "msg/Entity.msg"
  "msg/EntityFactory.msg"
  "msg/GuiCamera.msg"
  "msg/JointWrench.msg"
  "msg/Light.msg"
  "msg/ParamVec.msg"
  "msg/StringVec.msg"
  "msg/TrackVisual.msg"
  "msg/VideoRecord.msg"
  "msg/WorldControl.msg"
  "msg/WorldReset.msg"
)

set(srv_files
  "srv/ControlWorld.srv"
  "srv/DeleteEntity.srv"
  "srv/SetEntityPose.srv"
  "srv/SpawnEntity.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs rcl_interfaces
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
