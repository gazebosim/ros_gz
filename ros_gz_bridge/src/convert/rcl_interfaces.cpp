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

#include "ros_gz_bridge/convert/rcl_interfaces.hpp"

#include <limits>
#include <string>

namespace ros_gz_bridge
{

constexpr auto kIntMax = static_cast<int64_t>(std::numeric_limits<int32_t>::max());
constexpr auto kIntMin = static_cast<int64_t>(std::numeric_limits<int32_t>::min());

template<>
void
convert_ros_to_gz(
  const rcl_interfaces::msg::ParameterValue & ros_msg,
  gz::msgs::Any & gz_msg)
{
  using ParameterType = rcl_interfaces::msg::ParameterType;
  using Any_ValueType = gz::msgs::Any_ValueType;

  std::string unsupported_type;
  gz_msg.set_type(Any_ValueType::Any_ValueType_NONE);

  switch (ros_msg.type) {
    case ParameterType::PARAMETER_NOT_SET:
      break;
    case ParameterType::PARAMETER_BOOL:
      gz_msg.set_type(Any_ValueType::Any_ValueType_BOOLEAN);
      gz_msg.set_bool_value(ros_msg.bool_value);
      break;
    case ParameterType::PARAMETER_INTEGER:
      gz_msg.set_type(Any_ValueType::Any_ValueType_INT32);

      if (ros_msg.integer_value > kIntMax) {
        gz_msg.set_int_value(kIntMax);
        std::cerr << "ParameterValue INTEGER clamped to INT32_MAX\n";
      } else if (ros_msg.integer_value < kIntMin) {
        gz_msg.set_int_value(kIntMin);
        std::cerr << "ParameterValue INTEGER clamped to INT32_MIN\n";
      } else {
        gz_msg.set_int_value(ros_msg.integer_value);
      }
      break;
    case ParameterType::PARAMETER_DOUBLE:
      gz_msg.set_type(Any_ValueType::Any_ValueType_DOUBLE);
      gz_msg.set_double_value(ros_msg.double_value);
      break;
    case ParameterType::PARAMETER_STRING:
      gz_msg.set_type(Any_ValueType::Any_ValueType_STRING);
      gz_msg.set_string_value(ros_msg.string_value);
      break;
    case ParameterType::PARAMETER_BYTE_ARRAY:
      unsupported_type = "BYTE_ARRAY";
      break;
    case ParameterType::PARAMETER_BOOL_ARRAY:
      unsupported_type = "BOOL_ARRAY";
      break;
    case ParameterType::PARAMETER_INTEGER_ARRAY:
      unsupported_type = "INTEGER_ARRAY";
      break;
    case ParameterType::PARAMETER_DOUBLE_ARRAY:
      unsupported_type = "DOUBLE_ARRAY";
      break;
    case ParameterType::PARAMETER_STRING_ARRAY:
      unsupported_type = "STRING_ARRAY";
      break;
    default:
      unsupported_type = "UNKNOWN";
      break;
  }

  if (!unsupported_type.empty()) {
    std::cerr << "Converting unsupported ParameterValue [" <<
      unsupported_type << "] failed\n";
  }
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Any & gz_msg,
  rcl_interfaces::msg::ParameterValue & ros_msg)
{
  using ParameterType = rcl_interfaces::msg::ParameterType;
  using Any_ValueType = gz::msgs::Any_ValueType;

  ros_msg.type = ParameterType::PARAMETER_NOT_SET;

  std::string unsupported_type;
  switch (gz_msg.type()) {
    case Any_ValueType::Any_ValueType_NONE:
      break;
    case Any_ValueType::Any_ValueType_DOUBLE:
      ros_msg.type = ParameterType::PARAMETER_DOUBLE;
      ros_msg.double_value = gz_msg.double_value();
      break;
    case Any_ValueType::Any_ValueType_INT32:
      ros_msg.type = ParameterType::PARAMETER_INTEGER;
      ros_msg.integer_value = gz_msg.int_value();
      break;
    case Any_ValueType::Any_ValueType_STRING:
      ros_msg.type = ParameterType::PARAMETER_STRING;
      ros_msg.string_value = gz_msg.string_value();
      break;
    case Any_ValueType::Any_ValueType_BOOLEAN:
      ros_msg.type = ParameterType::PARAMETER_BOOL;
      ros_msg.bool_value = gz_msg.bool_value();
      break;
    case Any_ValueType::Any_ValueType_VECTOR3D:
      unsupported_type = "VECTOR3D";
      break;
    case Any_ValueType::Any_ValueType_COLOR:
      unsupported_type = "COLOR";
      break;
    case Any_ValueType::Any_ValueType_POSE3D:
      unsupported_type = "POSE3D";
      break;
    case Any_ValueType::Any_ValueType_QUATERNIOND:
      unsupported_type = "QUATERNIOND";
      break;
    case Any_ValueType::Any_ValueType_TIME:
      unsupported_type = "TIME";
      break;
    default:
      break;
  }

  if (!unsupported_type.empty()) {
    std::cerr << "Converting unsupported gz::msgs::Any [" <<
      unsupported_type << "] failed\n";
  }
}
}  // namespace ros_gz_bridge
