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

#ifndef  SERVICE_FACTORY_INTERFACE_HPP_
#define  SERVICE_FACTORY_INTERFACE_HPP_

#include <memory>
#include <string>

#include <ignition/transport/Node.hh>

#include <rclcpp/service.hpp>
#include <rclcpp/node.hpp>

namespace ros_gz_bridge
{

class ServiceFactoryInterface
{
public:
  virtual
  rclcpp::ServiceBase::SharedPtr
  create_ros_service(
    rclcpp::Node::SharedPtr ros_node,
    std::shared_ptr<ignition::transport::Node> gz_node,
    const std::string & service_name) = 0;
};

}  // namespace ros_gz_bridge

#endif  // SERVICE_FACTORY_INTERFACE_HPP_
