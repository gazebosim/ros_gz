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

#include <ros_ign_bridge/convert/rcl_interfaces.hpp>

#include <limits>

// A more specific set of tests for the rcl_interfaces/msg/ParamValue to
// ignition::msgs::Any to verify behaviors that couldn't easily be captured
// by the generic test framework

struct RosToIgnTest : public ::testing::Test
{
  using ParameterValue = rcl_interfaces::msg::ParameterValue;
  using ParameterType = rcl_interfaces::msg::ParameterType;
  using Any = ignition::msgs::Any;
  using Any_ValueType = ignition::msgs::Any_ValueType;

  Any ign_msg;
  ParameterValue ros_msg;
};

TEST_F(RosToIgnTest, NotSet)
{
  ros_msg.type = ParameterType::PARAMETER_NOT_SET;
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
  EXPECT_EQ(Any_ValueType::Any_ValueType_NONE, ign_msg.type());
}

TEST_F(RosToIgnTest, Boolean)
{
  ros_msg.type = ParameterType::PARAMETER_BOOL;
  ros_msg.bool_value = true;
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
  EXPECT_EQ(Any_ValueType::Any_ValueType_BOOLEAN, ign_msg.type());
  EXPECT_EQ(ros_msg.bool_value, ign_msg.bool_value());
}

TEST_F(RosToIgnTest, Integer)
{
  // Within range of int32
  ros_msg.type = ParameterType::PARAMETER_INTEGER;
  ros_msg.integer_value = 55;
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
  EXPECT_EQ(Any_ValueType::Any_ValueType_INT32, ign_msg.type());
  EXPECT_EQ(ros_msg.integer_value, ign_msg.int_value());

  // Greater than int32 max, Clamp to max
  ros_msg.integer_value =
    static_cast<int64_t>(std::numeric_limits<int32_t>::max()) + 1;
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
  EXPECT_EQ(Any_ValueType::Any_ValueType_INT32, ign_msg.type());
  EXPECT_EQ(std::numeric_limits<int32_t>::max(), ign_msg.int_value());

  // Less than int32 min, Clamp to min
  ros_msg.integer_value =
    static_cast<int64_t>(std::numeric_limits<int32_t>::min()) - 1;
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
  EXPECT_EQ(Any_ValueType::Any_ValueType_INT32, ign_msg.type());
  EXPECT_EQ(std::numeric_limits<int32_t>::min(), ign_msg.int_value());
}

TEST_F(RosToIgnTest, Double)
{
  ros_msg.type = ParameterType::PARAMETER_DOUBLE;
  ros_msg.double_value = 1.0;
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
  EXPECT_EQ(Any_ValueType::Any_ValueType_DOUBLE, ign_msg.type());
  EXPECT_EQ(ros_msg.double_value, ign_msg.double_value());
}

TEST_F(RosToIgnTest, String)
{
  ros_msg.type = ParameterType::PARAMETER_STRING;
  ros_msg.string_value = "baz";
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
  EXPECT_EQ(Any_ValueType::Any_ValueType_STRING, ign_msg.type());
  EXPECT_EQ(ros_msg.string_value, ign_msg.string_value());
}

TEST_F(RosToIgnTest, ByteArray)
{
  ros_msg.type = ParameterType::PARAMETER_BYTE_ARRAY;
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
  EXPECT_EQ(Any_ValueType::Any_ValueType_NONE, ign_msg.type());
}

TEST_F(RosToIgnTest, BoolArray)
{
  ros_msg.type = ParameterType::PARAMETER_BOOL_ARRAY;
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
  EXPECT_EQ(Any_ValueType::Any_ValueType_NONE, ign_msg.type());
}

TEST_F(RosToIgnTest, IntegerArray)
{
  ros_msg.type = ParameterType::PARAMETER_INTEGER_ARRAY;
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
  EXPECT_EQ(Any_ValueType::Any_ValueType_NONE, ign_msg.type());
}

TEST_F(RosToIgnTest, DoubleArray)
{
  ros_msg.type = ParameterType::PARAMETER_DOUBLE_ARRAY;
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
  EXPECT_EQ(Any_ValueType::Any_ValueType_NONE, ign_msg.type());
}

TEST_F(RosToIgnTest, StringArray)
{
  ros_msg.type = ParameterType::PARAMETER_STRING_ARRAY;
  ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
  EXPECT_EQ(Any_ValueType::Any_ValueType_NONE, ign_msg.type());
}

using IgnToRosTest = RosToIgnTest;

TEST_F(IgnToRosTest, None)
{
  ign_msg.set_type(Any_ValueType::Any_ValueType_NONE);
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
  EXPECT_EQ(ParameterType::PARAMETER_NOT_SET, ros_msg.type);
}

TEST_F(IgnToRosTest, Double)
{
  ign_msg.set_type(Any_ValueType::Any_ValueType_DOUBLE);
  ign_msg.set_double_value(10.0);
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
  EXPECT_EQ(ParameterType::PARAMETER_DOUBLE, ros_msg.type);
  EXPECT_EQ(ign_msg.double_value(), ros_msg.double_value);
}

TEST_F(IgnToRosTest, Int32)
{
  ign_msg.set_type(Any_ValueType::Any_ValueType_INT32);
  ign_msg.set_int_value(255);
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
  EXPECT_EQ(ParameterType::PARAMETER_INTEGER, ros_msg.type);
  EXPECT_EQ(ign_msg.int_value(), ros_msg.integer_value);
}

TEST_F(IgnToRosTest, String)
{
  ign_msg.set_type(Any_ValueType::Any_ValueType_STRING);
  ign_msg.set_string_value("foobar");
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
  EXPECT_EQ(ParameterType::PARAMETER_STRING, ros_msg.type);
  EXPECT_EQ(ign_msg.string_value(), ros_msg.string_value);
}

TEST_F(IgnToRosTest, Boolean)
{
  ign_msg.set_type(Any_ValueType::Any_ValueType_BOOLEAN);
  ign_msg.set_bool_value(true);
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
  EXPECT_EQ(ParameterType::PARAMETER_BOOL, ros_msg.type);
  EXPECT_EQ(ign_msg.bool_value(), ros_msg.bool_value);
}

TEST_F(IgnToRosTest, Vector3d)
{
  ign_msg.set_type(Any_ValueType::Any_ValueType_VECTOR3D);
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
  EXPECT_EQ(ParameterType::PARAMETER_NOT_SET, ros_msg.type);
}

TEST_F(IgnToRosTest, Color)
{
  ign_msg.set_type(Any_ValueType::Any_ValueType_COLOR);
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
  EXPECT_EQ(ParameterType::PARAMETER_NOT_SET, ros_msg.type);
}

TEST_F(IgnToRosTest, Pose3d)
{
  ign_msg.set_type(Any_ValueType::Any_ValueType_POSE3D);
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
  EXPECT_EQ(ParameterType::PARAMETER_NOT_SET, ros_msg.type);
}

TEST_F(IgnToRosTest, Quaterniond)
{
  ign_msg.set_type(Any_ValueType::Any_ValueType_QUATERNIOND);
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
  EXPECT_EQ(ParameterType::PARAMETER_NOT_SET, ros_msg.type);
}

TEST_F(IgnToRosTest, Time)
{
  ign_msg.set_type(Any_ValueType::Any_ValueType_TIME);
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_msg);
  EXPECT_EQ(ParameterType::PARAMETER_NOT_SET, ros_msg.type);
}
