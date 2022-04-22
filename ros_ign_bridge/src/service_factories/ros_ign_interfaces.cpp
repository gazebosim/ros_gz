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

#include "factories/ros_ign_interfaces.hpp"

#include <memory>
#include <string>

#include "ros_ign_interfaces/srv/control_world.hpp"

#include "service_factory.hpp"
#include "ros_ign_bridge/convert/ros_ign_interfaces.hpp"


namespace ros_ign_bridge
{

std::shared_ptr<ServiceFactoryInterface>
get_service_factory__ros_ign_interfaces(
  const std::string & ros_type_name,
  const std::string & ign_req_type_name,
  const std::string & ign_rep_type_name)
{
  if (
    ros_type_name == "ros_ign_interfaces/srv/ControlWorld" &&
    (ign_req_type_name.empty() || ign_req_type_name == "ignition.msgs.WorldControl") &&
    (ign_rep_type_name.empty() || ign_rep_type_name == "ignition.msgs.Boolean"))
  {
    return std::make_shared<
      ServiceFactory<
        ros_ign_interfaces::srv::ControlWorld,
        ignition::msgs::WorldControl,
        ignition::msgs::Boolean>
    >(ros_type_name, "ignition.msgs.WorldControl", "ignition.msgs.Boolean");
  }
  return nullptr;
}

template<>
void
convert_ros_to_ign(
  const ros_ign_interfaces::srv::ControlWorld::Request & ros_req,
  ignition::msgs::WorldControl & ign_req)
{
  convert_ros_to_ign(ros_req.world_control, ign_req);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Boolean & ign_rep,
  ros_ign_interfaces::srv::ControlWorld::Response & ros_res)
{
  ros_res.success = ign_rep.data();
}

template<>
bool
send_response_on_error(ros_ign_interfaces::srv::ControlWorld::Response & ros_res)
{
  // TODO(now): Is it worth it to have a different field to encode ignition request errors?
  //  Currently we're reusing the success field, which seems fine for this case.
  ros_res.success = false;
  return true;
}
}  // namespace ros_ign_bridge
