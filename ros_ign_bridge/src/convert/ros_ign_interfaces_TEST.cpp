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

#include <gtest/gtest.h>

#include <ros_ign_bridge/convert/ros_ign_interfaces.hpp>

// A more specific set of tests for the ros_ign_interfaces/msg/ParamVec to
// to verify behaviors that couldn't easily be captured by the generic test framework
struct RosToIgnTest : public ::testing::Test
{
  using ParameterValue = rcl_interfaces::msg::ParameterValue;
  using ParameterType = rcl_interfaces::msg::ParameterType;

  using Any = ignition::msgs::Any;
  using Any_ValueType = ignition::msgs::Any_ValueType;

  using ParamVec = ros_ign_interfaces::msg::ParamVec;
  using Param = ignition::msgs::Param;

  Param ign_msg;
  ParamVec ros_msg;
};

constexpr const char * kParamNotSetName = "notset_param";

constexpr const char * kParamBoolName = "bool_param";
constexpr auto kParamBoolValue = true;

constexpr const char * kParamIntName = "int_param";
constexpr auto kParamIntValue = 15;

constexpr const char * kParamDoubleName = "double_param";
constexpr auto kParamDoubleValue = 16.0;

constexpr const char * kParamStringName = "string_param";
constexpr const char * kParamStringValue = "foo";

TEST_F(RosToIgnTest, FlatParameters)
{
  {
    rcl_interfaces::msg::Parameter param;
    param.name = kParamNotSetName;
    param.value.type = ParameterType::PARAMETER_NOT_SET;
    ros_msg.params.push_back(param);
  }
  {
    rcl_interfaces::msg::Parameter param;
    param.name = kParamBoolName;
    param.value.type = ParameterType::PARAMETER_BOOL;
    param.value.bool_value = kParamBoolValue;
    ros_msg.params.push_back(param);
  }
  {
    rcl_interfaces::msg::Parameter param;
    param.name = kParamIntName;
    param.value.type = ParameterType::PARAMETER_INTEGER;
    param.value.integer_value = kParamIntValue;
    ros_msg.params.push_back(param);
  }
  {
    rcl_interfaces::msg::Parameter param;
    param.name = kParamDoubleName;
    param.value.type = ParameterType::PARAMETER_DOUBLE;
    param.value.double_value = kParamDoubleValue;
    ros_msg.params.push_back(param);
  }
  {
    rcl_interfaces::msg::Parameter param;
    param.name = kParamStringName;
    param.value.type = ParameterType::PARAMETER_STRING;
    param.value.string_value = kParamStringValue;
    ros_msg.params.push_back(param);
  }

  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);

  ASSERT_EQ(5u, ign_msg.params().size());

  ASSERT_NE(ign_msg.params().end(), ign_msg.params().find(kParamNotSetName));
  EXPECT_EQ(Any_ValueType::Any_ValueType_NONE, ign_msg.params().at(kParamNotSetName).type());

  ASSERT_NE(ign_msg.params().end(), ign_msg.params().find(kParamBoolName));
  EXPECT_EQ(Any_ValueType::Any_ValueType_BOOLEAN, ign_msg.params().at(kParamBoolName).type());
  EXPECT_EQ(kParamBoolValue, ign_msg.params().at(kParamBoolName).bool_value());

  ASSERT_NE(ign_msg.params().end(), ign_msg.params().find(kParamIntName));
  EXPECT_EQ(Any_ValueType::Any_ValueType_INT32, ign_msg.params().at(kParamIntName).type());
  EXPECT_EQ(kParamIntValue, ign_msg.params().at(kParamIntName).int_value());

  ASSERT_NE(ign_msg.params().end(), ign_msg.params().find(kParamDoubleName));
  EXPECT_EQ(Any_ValueType::Any_ValueType_DOUBLE, ign_msg.params().at(kParamDoubleName).type());
  EXPECT_EQ(kParamDoubleValue, ign_msg.params().at(kParamDoubleName).double_value());

  ASSERT_NE(ign_msg.params().end(), ign_msg.params().find(kParamStringName));
  EXPECT_EQ(Any_ValueType::Any_ValueType_STRING, ign_msg.params().at(kParamStringName).type());
  EXPECT_EQ(kParamStringValue, ign_msg.params().at(kParamStringName).string_value());
}

using IgnToRosTest = RosToIgnTest;

TEST_F(IgnToRosTest, FlatParameters)
{
  auto * params = ign_msg.mutable_params();
  {
    Any param;
    param.set_type(Any_ValueType::Any_ValueType_NONE);
    (*params)[kParamNotSetName] = param;
  }
  {
    Any param;
    param.set_type(Any_ValueType::Any_ValueType_DOUBLE);
    param.set_double_value(kParamDoubleValue);
    (*params)[kParamDoubleName] = param;
  }
  {
    Any param;
    param.set_type(Any_ValueType::Any_ValueType_INT32);
    param.set_int_value(kParamIntValue);
    (*params)[kParamIntName] = param;
  }
  {
    Any param;
    param.set_type(Any_ValueType::Any_ValueType_STRING);
    param.set_string_value(kParamStringValue);
    (*params)[kParamStringName] = param;
  }
  {
    Any param;
    param.set_type(Any_ValueType::Any_ValueType_BOOLEAN);
    param.set_bool_value(kParamBoolValue);
    (*params)[kParamBoolName] = param;
  }

  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);

  ASSERT_EQ(5u, ros_msg.params.size());

  // ROS doesn't store as a map, so we need to iterate the entries
  for (size_t ii = 0; ii < 5; ++ii) {
    auto toCheck = ros_msg.params[ii];

    if (toCheck.name == kParamNotSetName) {
      continue;
    } else if (toCheck.name == kParamDoubleName) {
      EXPECT_EQ(kParamDoubleValue, toCheck.value.double_value);
    } else if (toCheck.name == kParamIntName) {
      EXPECT_EQ(kParamIntValue, toCheck.value.integer_value);
    } else if (toCheck.name == kParamStringName) {
      EXPECT_EQ(kParamStringValue, toCheck.value.string_value);
    } else if (toCheck.name == kParamBoolName) {
      EXPECT_EQ(kParamBoolValue, toCheck.value.bool_value);
    }
  }
}

TEST_F(IgnToRosTest, ChildParameters)
{
  auto * root_params = ign_msg.mutable_params();
  auto * child1_params = ign_msg.mutable_children()->Add()->mutable_params();
  auto * child2_params = ign_msg.mutable_children()->Add()->mutable_params();

  auto * child3 = ign_msg.mutable_children()->Add();
  EXPECT_NE(nullptr, child3);
  auto * child3_params = child3->mutable_params();
  EXPECT_NE(nullptr, child3_params);
  auto * child3_child = child3->mutable_children()->Add();
  EXPECT_NE(nullptr, child3_child);
  auto * child3_child_params = child3_child->mutable_params();
  EXPECT_NE(nullptr, child3_child_params);

  /// Build a tree of parameters (5 in total)
  /// Root
  /// Parameters: [ NoneType ],
  /// Children: [
  ///   1: Parameters [Int32],
  ///   2: Parameters [Double] ,
  ///   3: Parameters [String],
  //       Children: [
  //        Parameters [ Boolean ],
  //  ]

  {
    // One parameter in root namesapce
    Any param;
    param.set_type(Any_ValueType::Any_ValueType_NONE);
    (*root_params)[kParamNotSetName] = param;
  }
  {
    // First child parameter
    Any param;
    param.set_type(Any_ValueType::Any_ValueType_INT32);
    param.set_int_value(kParamIntValue);
    (*child1_params)[kParamIntName] = param;
  }
  {
    // Second child parameter
    Any param;
    param.set_type(Any_ValueType::Any_ValueType_DOUBLE);
    param.set_double_value(kParamDoubleValue);
    (*child2_params)[kParamDoubleName] = param;
  }
  {
    Any param;
    param.set_type(Any_ValueType::Any_ValueType_STRING);
    param.set_string_value(kParamStringValue);
    (*child3_params)[kParamStringName] = param;
  }
  {
    Any param;
    param.set_type(Any_ValueType::Any_ValueType_BOOLEAN);
    param.set_bool_value(kParamBoolValue);
    (*child3_child_params)[kParamBoolName] = param;
  }

  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);

  ASSERT_EQ(5u, ros_msg.params.size());

  EXPECT_EQ(kParamNotSetName, ros_msg.params[0].name);

  EXPECT_EQ("child_0/int_param", ros_msg.params[1].name);
  EXPECT_EQ(kParamIntValue, ros_msg.params[1].value.integer_value);

  EXPECT_EQ("child_1/double_param", ros_msg.params[2].name);
  EXPECT_EQ(kParamDoubleValue, ros_msg.params[2].value.double_value);

  EXPECT_EQ("child_2/string_param", ros_msg.params[3].name);
  EXPECT_EQ(kParamStringValue, ros_msg.params[3].value.string_value);

  EXPECT_EQ("child_2/child_0/bool_param", ros_msg.params[4].name);
  EXPECT_EQ(kParamBoolValue, ros_msg.params[4].value.bool_value);
}
