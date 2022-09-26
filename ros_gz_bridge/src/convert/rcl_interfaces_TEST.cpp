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

#include <limits>

#include <ros_gz_bridge/convert/rcl_interfaces.hpp>

// A more specific set of tests for the rcl_interfaces/msg/ParamValue to
// gz::msgs::Any to verify behaviors that couldn't easily be captured
// by the generic test framework

struct RosToGzTest : public ::testing::Test
{
  using ParameterValue = rcl_interfaces::msg::ParameterValue;
  using ParameterType = rcl_interfaces::msg::ParameterType;
  using Any = gz::msgs::Any;
  using Any_ValueType = gz::msgs::Any_ValueType;

  Any gz_msg;
  ParameterValue ros_msg;
};

TEST_F(RosToGzTest, NotSet)
{
  ros_msg.type = ParameterType::PARAMETER_NOT_SET;
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
  EXPECT_EQ(Any_ValueType::Any_ValueType_NONE, gz_msg.type());
}

TEST_F(RosToGzTest, Boolean)
{
  ros_msg.type = ParameterType::PARAMETER_BOOL;
  ros_msg.bool_value = true;
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
  EXPECT_EQ(Any_ValueType::Any_ValueType_BOOLEAN, gz_msg.type());
  EXPECT_EQ(ros_msg.bool_value, gz_msg.bool_value());
}

TEST_F(RosToGzTest, Integer)
{
  // Within range of int32
  ros_msg.type = ParameterType::PARAMETER_INTEGER;
  ros_msg.integer_value = 55;
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
  EXPECT_EQ(Any_ValueType::Any_ValueType_INT32, gz_msg.type());
  EXPECT_EQ(ros_msg.integer_value, gz_msg.int_value());

  // Greater than int32 max, Clamp to max
  ros_msg.integer_value =
    static_cast<int64_t>(std::numeric_limits<int32_t>::max()) + 1;
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
  EXPECT_EQ(Any_ValueType::Any_ValueType_INT32, gz_msg.type());
  EXPECT_EQ(std::numeric_limits<int32_t>::max(), gz_msg.int_value());

  // Less than int32 min, Clamp to min
  ros_msg.integer_value =
    static_cast<int64_t>(std::numeric_limits<int32_t>::min()) - 1;
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
  EXPECT_EQ(Any_ValueType::Any_ValueType_INT32, gz_msg.type());
  EXPECT_EQ(std::numeric_limits<int32_t>::min(), gz_msg.int_value());
}

TEST_F(RosToGzTest, Double)
{
  ros_msg.type = ParameterType::PARAMETER_DOUBLE;
  ros_msg.double_value = 1.0;
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
  EXPECT_EQ(Any_ValueType::Any_ValueType_DOUBLE, gz_msg.type());
  EXPECT_EQ(ros_msg.double_value, gz_msg.double_value());
}

TEST_F(RosToGzTest, String)
{
  ros_msg.type = ParameterType::PARAMETER_STRING;
  ros_msg.string_value = "baz";
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
  EXPECT_EQ(Any_ValueType::Any_ValueType_STRING, gz_msg.type());
  EXPECT_EQ(ros_msg.string_value, gz_msg.string_value());
}

TEST_F(RosToGzTest, ByteArray)
{
  ros_msg.type = ParameterType::PARAMETER_BYTE_ARRAY;
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
  EXPECT_EQ(Any_ValueType::Any_ValueType_NONE, gz_msg.type());
}

TEST_F(RosToGzTest, BoolArray)
{
  ros_msg.type = ParameterType::PARAMETER_BOOL_ARRAY;
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
  EXPECT_EQ(Any_ValueType::Any_ValueType_NONE, gz_msg.type());
}

TEST_F(RosToGzTest, IntegerArray)
{
  ros_msg.type = ParameterType::PARAMETER_INTEGER_ARRAY;
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
  EXPECT_EQ(Any_ValueType::Any_ValueType_NONE, gz_msg.type());
}

TEST_F(RosToGzTest, DoubleArray)
{
  ros_msg.type = ParameterType::PARAMETER_DOUBLE_ARRAY;
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
  EXPECT_EQ(Any_ValueType::Any_ValueType_NONE, gz_msg.type());
}

TEST_F(RosToGzTest, StringArray)
{
  ros_msg.type = ParameterType::PARAMETER_STRING_ARRAY;
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
  EXPECT_EQ(Any_ValueType::Any_ValueType_NONE, gz_msg.type());
}

using GzToRosTest = RosToGzTest;

TEST_F(GzToRosTest, None)
{
  gz_msg.set_type(Any_ValueType::Any_ValueType_NONE);
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
  EXPECT_EQ(ParameterType::PARAMETER_NOT_SET, ros_msg.type);
}

TEST_F(GzToRosTest, Double)
{
  gz_msg.set_type(Any_ValueType::Any_ValueType_DOUBLE);
  gz_msg.set_double_value(10.0);
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
  EXPECT_EQ(ParameterType::PARAMETER_DOUBLE, ros_msg.type);
  EXPECT_EQ(gz_msg.double_value(), ros_msg.double_value);
}

TEST_F(GzToRosTest, Int32)
{
  gz_msg.set_type(Any_ValueType::Any_ValueType_INT32);
  gz_msg.set_int_value(255);
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
  EXPECT_EQ(ParameterType::PARAMETER_INTEGER, ros_msg.type);
  EXPECT_EQ(gz_msg.int_value(), ros_msg.integer_value);
}

TEST_F(GzToRosTest, String)
{
  gz_msg.set_type(Any_ValueType::Any_ValueType_STRING);
  gz_msg.set_string_value("foobar");
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
  EXPECT_EQ(ParameterType::PARAMETER_STRING, ros_msg.type);
  EXPECT_EQ(gz_msg.string_value(), ros_msg.string_value);
}

TEST_F(GzToRosTest, Boolean)
{
  gz_msg.set_type(Any_ValueType::Any_ValueType_BOOLEAN);
  gz_msg.set_bool_value(true);
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
  EXPECT_EQ(ParameterType::PARAMETER_BOOL, ros_msg.type);
  EXPECT_EQ(gz_msg.bool_value(), ros_msg.bool_value);
}

TEST_F(GzToRosTest, Vector3d)
{
  gz_msg.set_type(Any_ValueType::Any_ValueType_VECTOR3D);
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
  EXPECT_EQ(ParameterType::PARAMETER_NOT_SET, ros_msg.type);
}

TEST_F(GzToRosTest, Color)
{
  gz_msg.set_type(Any_ValueType::Any_ValueType_COLOR);
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
  EXPECT_EQ(ParameterType::PARAMETER_NOT_SET, ros_msg.type);
}

TEST_F(GzToRosTest, Pose3d)
{
  gz_msg.set_type(Any_ValueType::Any_ValueType_POSE3D);
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
  EXPECT_EQ(ParameterType::PARAMETER_NOT_SET, ros_msg.type);
}

TEST_F(GzToRosTest, Quaterniond)
{
  gz_msg.set_type(Any_ValueType::Any_ValueType_QUATERNIOND);
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
  EXPECT_EQ(ParameterType::PARAMETER_NOT_SET, ros_msg.type);
}

TEST_F(GzToRosTest, Time)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
  gz_msg.set_type(Any_ValueType::Any_ValueType_TIME);
  EXPECT_EQ(ParameterType::PARAMETER_NOT_SET, ros_msg.type);
}
