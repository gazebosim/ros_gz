// Copyright 2022 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/pose.pb.h>

#include <memory>
#include <string>

#include "service_factories/ros_gz_interfaces.hpp"
#include "ros_gz_interfaces/srv/control_world.hpp"
#include "ros_gz_interfaces/srv/delete_entity.hpp"
#include "ros_gz_interfaces/srv/spawn_entity.hpp"
#include "ros_gz_interfaces/srv/set_entity_pose.hpp"
#include "ros_gz_bridge/convert/ros_gz_interfaces.hpp"

#include "service_factory.hpp"

namespace ros_gz_bridge
{

std::shared_ptr<ServiceFactoryInterface>
get_service_factory__ros_gz_interfaces(
  const std::string & ros_type_name,
  const std::string & gz_req_type_name,
  const std::string & gz_rep_type_name)
{
  if (
    ros_type_name == "ros_gz_interfaces/srv/ControlWorld" &&
    (gz_req_type_name.empty() || gz_req_type_name == "gz.msgs.WorldControl") &&
    (gz_rep_type_name.empty() || gz_rep_type_name == "gz.msgs.Boolean"))
  {
    return std::make_shared<
      ServiceFactory<
        ros_gz_interfaces::srv::ControlWorld,
        gz::msgs::WorldControl,
        gz::msgs::Boolean>
    >(ros_type_name, "gz.msgs.WorldControl", "gz.msgs.Boolean");
  }

  if (
    ros_type_name == "ros_gz_interfaces/srv/DeleteEntity" &&
    (gz_req_type_name.empty() || gz_req_type_name == "gz.msgs.Entity") &&
    (gz_rep_type_name.empty() || gz_rep_type_name == "gz.msgs.Boolean"))
  {
    return std::make_shared<
      ServiceFactory<
        ros_gz_interfaces::srv::DeleteEntity,
        gz::msgs::Entity,
        gz::msgs::Boolean>
    >(ros_type_name, "gz.msgs.Entity", "gz.msgs.Boolean");
  }

  if (
    ros_type_name == "ros_gz_interfaces/srv/SpawnEntity" &&
    (gz_req_type_name.empty() || gz_req_type_name == "gz.msgs.EntityFactory") &&
    (gz_rep_type_name.empty() || gz_rep_type_name == "gz.msgs.Boolean"))
  {
    return std::make_shared<
      ServiceFactory<
        ros_gz_interfaces::srv::SpawnEntity,
        gz::msgs::EntityFactory,
        gz::msgs::Boolean>
    >(ros_type_name, "gz.msgs.EntityFactory", "gz.msgs.Boolean");
  }

  if (
    ros_type_name == "ros_gz_interfaces/srv/SetEntityPose" &&
    (gz_req_type_name.empty() || gz_req_type_name == "gz.msgs.Pose") &&
    (gz_rep_type_name.empty() || gz_rep_type_name == "gz.msgs.Boolean"))
  {
    return std::make_shared<
      ServiceFactory<
        ros_gz_interfaces::srv::SetEntityPose,
        gz::msgs::Pose,
        gz::msgs::Boolean>
    >(ros_type_name, "gz.msgs.Pose", "gz.msgs.Boolean");
  }

  return nullptr;
}

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::srv::ControlWorld::Request & ros_req,
  gz::msgs::WorldControl & gz_req)
{
  convert_ros_to_gz(ros_req.world_control, gz_req);
}

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::srv::DeleteEntity::Request & ros_req,
  gz::msgs::Entity & gz_req)
{
  convert_ros_to_gz(ros_req.entity, gz_req);
}

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::srv::SpawnEntity::Request & ros_req,
  gz::msgs::EntityFactory & gz_req)
{
  convert_ros_to_gz(ros_req.entity_factory, gz_req);
}

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::srv::SetEntityPose::Request & ros_req,
  gz::msgs::Pose & gz_req)
{
  convert_ros_to_gz(ros_req.entity, gz_req);
  convert_ros_to_gz(ros_req.pose, gz_req);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Boolean & gz_rep,
  ros_gz_interfaces::srv::ControlWorld::Response & ros_res)
{
  ros_res.success = gz_rep.data();
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Boolean & gz_rep,
  ros_gz_interfaces::srv::DeleteEntity::Response & ros_res)
{
  ros_res.success = gz_rep.data();
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Boolean & gz_rep,
  ros_gz_interfaces::srv::SpawnEntity::Response & ros_res)
{
  ros_res.success = gz_rep.data();
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Boolean & gz_rep,
  ros_gz_interfaces::srv::SetEntityPose::Response & ros_res)
{
  ros_res.success = gz_rep.data();
}

template<>
bool
send_response_on_error(ros_gz_interfaces::srv::ControlWorld::Response & ros_res)
{
  // TODO(now): Is it worth it to have a different field to encode Gazebo request errors?
  //  Currently we're reusing the success field, which seems fine for this case.
  ros_res.success = false;
  return true;
}

template<>
bool
send_response_on_error(ros_gz_interfaces::srv::DeleteEntity::Response & ros_res)
{
  // TODO(now): Is it worth it to have a different field to encode Gazebo request errors?
  //  Currently we're reusing the success field, which seems fine for this case.
  ros_res.success = false;
  return true;
}

template<>
bool
send_response_on_error(ros_gz_interfaces::srv::SpawnEntity::Response & ros_res)
{
  // TODO(now): Is it worth it to have a different field to encode Gazebo request errors?
  //  Currently we're reusing the success field, which seems fine for this case.
  ros_res.success = false;
  return true;
}

template<>
bool
send_response_on_error(ros_gz_interfaces::srv::SetEntityPose::Response & ros_res)
{
  // TODO(now): Is it worth it to have a different field to encode Gazebo request errors?
  //  Currently we're reusing the success field, which seems fine for this case.
  ros_res.success = false;
  return true;
}
}  // namespace ros_gz_bridge
