// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include "ros_test_msg.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>

namespace ros_ign_bridge
{
namespace testing
{

void createTestMsg(std_msgs::msg::Bool & _msg)
{
  _msg.data = true;
}

void compareTestMsg(const std::shared_ptr<std_msgs::msg::Bool> & _msg)
{
  std_msgs::msg::Bool expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.data, _msg->data);
}

void createTestMsg(std_msgs::msg::ColorRGBA & _msg)
{
  _msg.r = 0.2;
  _msg.g = 0.4;
  _msg.b = 0.6;
  _msg.a = 0.8;
}

void compareTestMsg(const std::shared_ptr<std_msgs::msg::ColorRGBA> & _msg)
{
  std_msgs::msg::ColorRGBA expected_msg;
  createTestMsg(expected_msg);

  EXPECT_FLOAT_EQ(expected_msg.r, _msg->r);
  EXPECT_FLOAT_EQ(expected_msg.g, _msg->g);
  EXPECT_FLOAT_EQ(expected_msg.b, _msg->b);
  EXPECT_FLOAT_EQ(expected_msg.a, _msg->a);
}

void compareTestMsg(const std::shared_ptr<std_msgs::msg::Empty> &)
{
}

void createTestMsg(std_msgs::msg::Float32 & _msg)
{
  _msg.data = 1.5;
}

void compareTestMsg(const std::shared_ptr<std_msgs::msg::Float32> & _msg)
{
  std_msgs::msg::Float32 expected_msg;
  createTestMsg(expected_msg);

  EXPECT_FLOAT_EQ(expected_msg.data, _msg->data);
}

void createTestMsg(std_msgs::msg::Float64 & _msg)
{
  _msg.data = 1.5;
}

void compareTestMsg(const std::shared_ptr<std_msgs::msg::Float64> & _msg)
{
  std_msgs::msg::Float64 expected_msg;
  createTestMsg(expected_msg);

  EXPECT_FLOAT_EQ(expected_msg.data, _msg->data);
}


void createTestMsg(std_msgs::msg::UInt32 & _msg)
{
  _msg.data = 1;
}

void compareTestMsg(const std::shared_ptr<std_msgs::msg::UInt32> & _msg)
{
  std_msgs::msg::UInt32 expected_msg;
  createTestMsg(expected_msg);

  EXPECT_FLOAT_EQ(expected_msg.data, _msg->data);
}

void createTestMsg(std_msgs::msg::Header & _msg)
{
  // _msg.seq        = 1;
  _msg.stamp.sec = 2;
  _msg.stamp.nanosec = 3;
  _msg.frame_id = "frame_id_value";
}

void compareTestMsg(const std::shared_ptr<std_msgs::msg::Header> & _msg)
{
  std_msgs::msg::Header expected_msg;
  createTestMsg(expected_msg);

  // EXPECT_GE(expected_msg.seq,        0u);
  EXPECT_EQ(expected_msg.stamp.sec, _msg->stamp.sec);
  EXPECT_EQ(expected_msg.stamp.nanosec, _msg->stamp.nanosec);
  EXPECT_EQ(expected_msg.frame_id, _msg->frame_id);
}

void compareTestMsg(const std_msgs::msg::Header & _msg)
{
  std_msgs::msg::Header expected_msg;
  createTestMsg(expected_msg);

  // EXPECT_GE(expected_msg.seq,        0u);
  EXPECT_EQ(expected_msg.stamp.sec, _msg.stamp.sec);
  EXPECT_EQ(expected_msg.stamp.nanosec, _msg.stamp.nanosec);
  EXPECT_EQ(expected_msg.frame_id, _msg.frame_id);
}

void createTestMsg(std_msgs::msg::String & _msg)
{
  _msg.data = "string";
}

void compareTestMsg(const std::shared_ptr<std_msgs::msg::String> & _msg)
{
  std_msgs::msg::String expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.data, _msg->data);
}

void createTestMsg(geometry_msgs::msg::Quaternion & _msg)
{
  _msg.x = 1;
  _msg.y = 2;
  _msg.z = 3;
  _msg.w = 4;
}

void compareTestMsg(const geometry_msgs::msg::Quaternion & _msg)
{
  geometry_msgs::msg::Quaternion expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.x, _msg.x);
  EXPECT_EQ(expected_msg.y, _msg.y);
  EXPECT_EQ(expected_msg.z, _msg.z);
  EXPECT_EQ(expected_msg.w, _msg.w);
}

void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::Quaternion> & _msg)
{
  geometry_msgs::msg::Quaternion expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.x, _msg->x);
  EXPECT_EQ(expected_msg.y, _msg->y);
  EXPECT_EQ(expected_msg.z, _msg->z);
  EXPECT_EQ(expected_msg.w, _msg->w);
}

void createTestMsg(geometry_msgs::msg::Vector3 & _msg)
{
  _msg.x = 1;
  _msg.y = 2;
  _msg.z = 3;
}

void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::Vector3> & _msg)
{
  geometry_msgs::msg::Vector3 expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.x, _msg->x);
  EXPECT_EQ(expected_msg.y, _msg->y);
  EXPECT_EQ(expected_msg.z, _msg->z);
}

void createTestMsg(rosgraph_msgs::msg::Clock & _msg)
{
  _msg.clock.sec = 1;
  _msg.clock.nanosec = 2;
}

void createTestMsg(geometry_msgs::msg::Point & _msg)
{
  _msg.x = 1;
  _msg.y = 2;
  _msg.z = 3;
}

void compareTestMsg(const std::shared_ptr<rosgraph_msgs::msg::Clock> & _msg)
{
  rosgraph_msgs::msg::Clock expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.clock.sec, _msg->clock.sec);
  EXPECT_EQ(expected_msg.clock.nanosec, _msg->clock.nanosec);
}

void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::Point> & _msg)
{
  geometry_msgs::msg::Point expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.x, _msg->x);
  EXPECT_EQ(expected_msg.y, _msg->y);
  EXPECT_EQ(expected_msg.z, _msg->z);
}

void compareTestMsg(const geometry_msgs::msg::Point & _msg)
{
  geometry_msgs::msg::Point expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.x, _msg.x);
  EXPECT_EQ(expected_msg.y, _msg.y);
  EXPECT_EQ(expected_msg.z, _msg.z);
}

void compareTestMsg(const geometry_msgs::msg::Pose & _msg)
{
  compareTestMsg(_msg.position);
  compareTestMsg(_msg.orientation);
}

void createTestMsg(geometry_msgs::msg::Pose & _msg)
{
  createTestMsg(_msg.position);
  createTestMsg(_msg.orientation);
}

void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::Pose> & _msg)
{
  compareTestMsg(_msg->position);
  compareTestMsg(_msg->orientation);
}

void createTestMsg(geometry_msgs::msg::PoseStamped & _msg)
{
  createTestMsg(_msg.header);
  createTestMsg(_msg.pose);
}

void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::PoseStamped> & _msg)
{
  compareTestMsg(_msg->header);
  compareTestMsg(_msg->pose);
}

void compareTestMsg(const geometry_msgs::msg::PoseStamped & _msg)
{
  compareTestMsg(_msg.header);
  compareTestMsg(_msg.pose);
}

void createTestMsg(geometry_msgs::msg::Transform & _msg)
{
  createTestMsg(_msg.translation);
  createTestMsg(_msg.rotation);
}

void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::Transform> & _msg)
{
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->translation));
  compareTestMsg(std::make_shared<geometry_msgs::msg::Quaternion>(_msg->rotation));
}
void createTestMsg(geometry_msgs::msg::TransformStamped & _msg)
{
  createTestMsg(_msg.header);
  createTestMsg(_msg.transform);
  _msg.child_frame_id = "child_frame_id_value";
}

void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::TransformStamped> & _msg)
{
  geometry_msgs::msg::TransformStamped expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  compareTestMsg(std::make_shared<geometry_msgs::msg::Transform>(_msg->transform));
  EXPECT_EQ(expected_msg.child_frame_id, _msg->child_frame_id);
}

void createTestMsg(tf2_msgs::msg::TFMessage & _msg)
{
  geometry_msgs::msg::TransformStamped tf;
  createTestMsg(tf);
  _msg.transforms.push_back(tf);
}

void compareTestMsg(const std::shared_ptr<tf2_msgs::msg::TFMessage> & _msg)
{
  tf2_msgs::msg::TFMessage expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(std::make_shared<geometry_msgs::msg::TransformStamped>(_msg->transforms[0]));
}

void createTestMsg(geometry_msgs::msg::Twist & _msg)
{
  createTestMsg(_msg.linear);
  createTestMsg(_msg.angular);
}

void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::Twist> & _msg)
{
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->linear));
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->angular));
}

void createTestMsg(geometry_msgs::msg::Wrench & _msg)
{
  createTestMsg(_msg.force);
  createTestMsg(_msg.torque);
}

void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::Wrench> & _msg)
{
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->force));
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->torque));
}

void createTestMsg(ros_ign_interfaces::msg::Light & _msg)
{
  createTestMsg(_msg.header);

  _msg.name = "test_light";
  _msg.type = 1;

  createTestMsg(_msg.pose);
  createTestMsg(_msg.diffuse);
  createTestMsg(_msg.specular);
  _msg.attenuation_constant = 0.2;
  _msg.attenuation_linear = 0.4;
  _msg.attenuation_quadratic = 0.6;
  createTestMsg(_msg.direction);
  _msg.range = 25.0;
  _msg.cast_shadows = true;
  _msg.spot_inner_angle = 0.3;
  _msg.spot_outer_angle = 0.6;
  _msg.spot_falloff = 10.0;

  _msg.id = 24;

  _msg.parent_id = 6;

  _msg.intensity = 125.0;
}

void compareTestMsg(const std::shared_ptr<ros_ign_interfaces::msg::Light> & _msg)
{
  ros_ign_interfaces::msg::Light expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);

  EXPECT_EQ(expected_msg.name, _msg->name);
  EXPECT_EQ(expected_msg.type, _msg->type);

  compareTestMsg(std::make_shared<geometry_msgs::msg::Pose>(_msg->pose));
  compareTestMsg(std::make_shared<std_msgs::msg::ColorRGBA>(_msg->diffuse));
  compareTestMsg(std::make_shared<std_msgs::msg::ColorRGBA>(_msg->specular));
  EXPECT_FLOAT_EQ(expected_msg.attenuation_constant, _msg->attenuation_constant);
  EXPECT_FLOAT_EQ(expected_msg.attenuation_linear, _msg->attenuation_linear);
  EXPECT_FLOAT_EQ(expected_msg.attenuation_quadratic, _msg->attenuation_quadratic);
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->direction));
  EXPECT_FLOAT_EQ(expected_msg.range, _msg->range);
  EXPECT_EQ(expected_msg.cast_shadows, _msg->cast_shadows);
  EXPECT_FLOAT_EQ(expected_msg.spot_inner_angle, _msg->spot_inner_angle);
  EXPECT_FLOAT_EQ(expected_msg.spot_outer_angle, _msg->spot_outer_angle);
  EXPECT_FLOAT_EQ(expected_msg.spot_falloff, _msg->spot_falloff);

  EXPECT_EQ(expected_msg.id, _msg->id);

  EXPECT_EQ(expected_msg.parent_id, _msg->parent_id);

  EXPECT_FLOAT_EQ(expected_msg.intensity, _msg->intensity);
}

void createTestMsg(ros_ign_interfaces::msg::JointWrench & _msg)
{
  createTestMsg(_msg.header);
  _msg.body_1_name.data = "body1";
  _msg.body_2_name.data = "body2";
  _msg.body_1_id.data = 1;
  _msg.body_2_id.data = 2;
  createTestMsg(_msg.body_1_wrench);
  createTestMsg(_msg.body_2_wrench);
}

void compareTestMsg(const std::shared_ptr<ros_ign_interfaces::msg::JointWrench> & _msg)
{
  ros_ign_interfaces::msg::JointWrench expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.body_1_name, _msg->body_1_name);
  EXPECT_EQ(expected_msg.body_2_name, _msg->body_2_name);
  EXPECT_EQ(expected_msg.body_1_id, _msg->body_1_id);
  EXPECT_EQ(expected_msg.body_2_id, _msg->body_2_id);

  compareTestMsg(_msg->header);
  compareTestMsg(std::make_shared<geometry_msgs::msg::Wrench>(_msg->body_1_wrench));
  compareTestMsg(std::make_shared<geometry_msgs::msg::Wrench>(_msg->body_2_wrench));
}

void createTestMsg(ros_ign_interfaces::msg::Entity & _msg)
{
  _msg.id = 1;
  _msg.name = "entity";
  _msg.type = ros_ign_interfaces::msg::Entity::VISUAL;
}

void compareTestMsg(const std::shared_ptr<ros_ign_interfaces::msg::Entity> & _msg)
{
  ros_ign_interfaces::msg::Entity expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.id, _msg->id);
  EXPECT_EQ(expected_msg.name, _msg->name);
  EXPECT_EQ(expected_msg.type, _msg->type);
}

void createTestMsg(ros_ign_interfaces::msg::Contact & _msg)
{
  createTestMsg(_msg.collision1);
  createTestMsg(_msg.collision2);

  geometry_msgs::msg::Vector3 vector_msg;
  createTestMsg(vector_msg);

  ros_ign_interfaces::msg::JointWrench joint_wrench_msg;
  createTestMsg(joint_wrench_msg);

  for (int i = 0; i < 10; i++) {
    _msg.depths.push_back(i);
    _msg.positions.push_back(vector_msg);
    _msg.normals.push_back(vector_msg);
    _msg.wrenches.push_back(joint_wrench_msg);
  }
}

void compareTestMsg(const std::shared_ptr<ros_ign_interfaces::msg::Contact> & _msg)
{
  ros_ign_interfaces::msg::Contact expected_msg;
  createTestMsg(expected_msg);
  compareTestMsg(std::make_shared<ros_ign_interfaces::msg::Entity>(_msg->collision1));
  compareTestMsg(std::make_shared<ros_ign_interfaces::msg::Entity>(_msg->collision2));
  EXPECT_EQ(expected_msg.depths.size(), _msg->depths.size());
  EXPECT_EQ(expected_msg.positions.size(), _msg->positions.size());
  EXPECT_EQ(expected_msg.normals.size(), _msg->normals.size());
  EXPECT_EQ(expected_msg.wrenches.size(), _msg->wrenches.size());
  for (size_t i = 0; i < _msg->depths.size(); i++) {
    EXPECT_EQ(expected_msg.depths.at(i), _msg->depths.at(i));
    compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->positions.at(i)));
    compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->normals.at(i)));
    compareTestMsg(std::make_shared<ros_ign_interfaces::msg::JointWrench>(_msg->wrenches.at(i)));
  }
}

void createTestMsg(ros_ign_interfaces::msg::Contacts & _msg)
{
  createTestMsg(_msg.header);

  ros_ign_interfaces::msg::Contact contact_msg;
  createTestMsg(contact_msg);

  for (int i = 0; i < 10; i++) {
    _msg.contacts.push_back(contact_msg);
  }
}

void compareTestMsg(const std::shared_ptr<ros_ign_interfaces::msg::Contacts> & _msg)
{
  ros_ign_interfaces::msg::Contacts expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  EXPECT_EQ(expected_msg.contacts.size(), _msg->contacts.size());
  for (size_t i = 0; i < _msg->contacts.size(); i++) {
    compareTestMsg(std::make_shared<ros_ign_interfaces::msg::Contact>(_msg->contacts.at(i)));
  }
}

void createTestMsg(nav_msgs::msg::Odometry & _msg)
{
  createTestMsg(_msg.header);
  createTestMsg(_msg.pose.pose);
  createTestMsg(_msg.twist.twist);
}

void compareTestMsg(const std::shared_ptr<nav_msgs::msg::Odometry> & _msg)
{
  compareTestMsg(_msg->header);
  compareTestMsg(_msg->pose.pose);
  compareTestMsg(std::make_shared<geometry_msgs::msg::Twist>(_msg->twist.twist));
}

void createTestMsg(sensor_msgs::msg::Image & _msg)
{
  std_msgs::msg::Header header_msg;
  createTestMsg(header_msg);

  _msg.header = header_msg;
  _msg.width = 320;
  _msg.height = 240;
  _msg.encoding = "rgb8";
  _msg.is_bigendian = false;
  _msg.step = _msg.width * 3;
  _msg.data.resize(_msg.height * _msg.step, '1');
}

void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::Image> & _msg)
{
  sensor_msgs::msg::Image expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  EXPECT_EQ(expected_msg.width, _msg->width);
  EXPECT_EQ(expected_msg.height, _msg->height);
  EXPECT_EQ(expected_msg.encoding, _msg->encoding);
  EXPECT_EQ(expected_msg.is_bigendian, _msg->is_bigendian);
  EXPECT_EQ(expected_msg.step, _msg->step);
}

void createTestMsg(sensor_msgs::msg::CameraInfo & _msg)
{
  std_msgs::msg::Header header_msg;
  createTestMsg(header_msg);

  _msg.header = header_msg;
  _msg.width = 320;
  _msg.height = 240;
  _msg.distortion_model = "plumb_bob";
  _msg.d.resize(5);
  _msg.d[0] = 1;
  _msg.d[1] = 2;
  _msg.d[2] = 3;
  _msg.d[3] = 4;
  _msg.d[4] = 5;

  _msg.k[0] = 1;
  _msg.k[1] = 0;
  _msg.k[2] = 0;
  _msg.k[3] = 0;
  _msg.k[4] = 1;
  _msg.k[5] = 0;
  _msg.k[6] = 0;
  _msg.k[7] = 0;
  _msg.k[8] = 1;

  _msg.r[0] = 1;
  _msg.r[1] = 0;
  _msg.r[2] = 0;
  _msg.r[3] = 0;
  _msg.r[4] = 1;
  _msg.r[5] = 0;
  _msg.r[6] = 0;
  _msg.r[7] = 0;
  _msg.r[8] = 1;

  _msg.p[0] = 1;
  _msg.p[1] = 0;
  _msg.p[2] = 0;
  _msg.p[3] = 0;
  _msg.p[4] = 0;
  _msg.p[5] = 1;
  _msg.p[6] = 0;
  _msg.p[7] = 0;
  _msg.p[8] = 0;
  _msg.p[9] = 0;
  _msg.p[10] = 1;
  _msg.p[11] = 0;
}

void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::CameraInfo> & _msg)
{
  sensor_msgs::msg::CameraInfo expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  EXPECT_EQ(expected_msg.width, _msg->width);
  EXPECT_EQ(expected_msg.height, _msg->height);
  EXPECT_EQ(expected_msg.distortion_model, _msg->distortion_model);

  for (auto i = 0; i < 12; ++i) {
    EXPECT_EQ(expected_msg.p[i], _msg->p[i]);

    if (i > 8) {
      continue;
    }

    EXPECT_EQ(expected_msg.k[i], _msg->k[i]);
    EXPECT_EQ(expected_msg.r[i], _msg->r[i]);

    if (i > 4) {
      continue;
    }

    EXPECT_EQ(expected_msg.d[i], _msg->d[i]);
  }
}

void createTestMsg(sensor_msgs::msg::FluidPressure & _msg)
{
  std_msgs::msg::Header header_msg;
  createTestMsg(header_msg);

  _msg.header = header_msg;
  _msg.fluid_pressure = 0.123;
  _msg.variance = 0.456;
}

void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::FluidPressure> & _msg)
{
  sensor_msgs::msg::FluidPressure expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  EXPECT_FLOAT_EQ(expected_msg.fluid_pressure, _msg->fluid_pressure);
  EXPECT_FLOAT_EQ(expected_msg.variance, _msg->variance);
}

void createTestMsg(sensor_msgs::msg::Imu & _msg)
{
  std_msgs::msg::Header header_msg;
  geometry_msgs::msg::Quaternion quaternion_msg;
  geometry_msgs::msg::Vector3 vector3_msg;

  createTestMsg(header_msg);
  createTestMsg(quaternion_msg);
  createTestMsg(vector3_msg);

  _msg.header = header_msg;
  _msg.orientation = quaternion_msg;
  _msg.angular_velocity = vector3_msg;
  _msg.linear_acceleration = vector3_msg;
}

void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::Imu> & _msg)
{
  compareTestMsg(_msg->header);
  compareTestMsg(_msg->orientation);
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->angular_velocity));
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->linear_acceleration));
}

void createTestMsg(sensor_msgs::msg::JointState & _msg)
{
  std_msgs::msg::Header header_msg;
  createTestMsg(header_msg);

  _msg.header = header_msg;
  _msg.name = {"joint_0", "joint_1", "joint_2"};
  _msg.position = {1, 1, 1};
  _msg.velocity = {2, 2, 2};
  _msg.effort = {3, 3, 3};
}

void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::JointState> & _msg)
{
  sensor_msgs::msg::JointState expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);

  ASSERT_EQ(expected_msg.name.size(), _msg->name.size());
  ASSERT_EQ(expected_msg.position.size(), _msg->position.size());
  ASSERT_EQ(expected_msg.velocity.size(), _msg->velocity.size());
  ASSERT_EQ(expected_msg.effort.size(), _msg->effort.size());

  for (auto i = 0u; i < _msg->position.size(); ++i) {
    EXPECT_EQ(expected_msg.name[i], _msg->name[i]);
    EXPECT_FLOAT_EQ(expected_msg.position[i], _msg->position[i]);
    EXPECT_FLOAT_EQ(expected_msg.velocity[i], _msg->velocity[i]);
    EXPECT_FLOAT_EQ(expected_msg.effort[i], _msg->effort[i]);
  }
}

void createTestMsg(sensor_msgs::msg::LaserScan & _msg)
{
  const unsigned int num_readings = 100u;

  std_msgs::msg::Header header_msg;
  createTestMsg(header_msg);

  _msg.header = header_msg;
  _msg.angle_min = -1.57;
  _msg.angle_max = 1.57;
  _msg.angle_increment = 3.14 / num_readings;
  _msg.scan_time = 0;
  _msg.range_min = 1;
  _msg.range_max = 2;
  _msg.ranges.resize(num_readings, 0);
  _msg.intensities.resize(num_readings, 1);
}

void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::LaserScan> & _msg)
{
  sensor_msgs::msg::LaserScan expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  EXPECT_FLOAT_EQ(expected_msg.angle_min, _msg->angle_min);
  EXPECT_FLOAT_EQ(expected_msg.angle_max, _msg->angle_max);
  EXPECT_FLOAT_EQ(expected_msg.angle_increment, _msg->angle_increment);
  EXPECT_FLOAT_EQ(0, _msg->scan_time);
  EXPECT_FLOAT_EQ(expected_msg.range_min, _msg->range_min);
  EXPECT_FLOAT_EQ(expected_msg.range_max, _msg->range_max);

  const unsigned int num_readings =
    (_msg->angle_max - _msg->angle_min) / _msg->angle_increment;
  for (auto i = 0u; i < num_readings; ++i) {
    EXPECT_FLOAT_EQ(expected_msg.ranges[i], _msg->ranges[i]);
    EXPECT_FLOAT_EQ(expected_msg.intensities[i], _msg->intensities[i]);
  }
}

void createTestMsg(sensor_msgs::msg::MagneticField & _msg)
{
  std_msgs::msg::Header header_msg;
  geometry_msgs::msg::Vector3 vector3_msg;

  createTestMsg(header_msg);
  createTestMsg(vector3_msg);

  _msg.header = header_msg;
  _msg.magnetic_field = vector3_msg;
}

void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::MagneticField> & _msg)
{
  compareTestMsg(_msg->header);
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->magnetic_field));
}

void createTestMsg(sensor_msgs::msg::PointCloud2 & _msg)
{
  createTestMsg(_msg.header);

  sensor_msgs::msg::PointField field;
  field.name = "x";
  field.offset = 0;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  _msg.fields.push_back(field);

  uint32_t height = 4;
  uint32_t width = 2;

  _msg.height = height;
  _msg.width = width;
  _msg.is_bigendian = false;
  _msg.point_step = 4;
  _msg.row_step = 4 * width;
  _msg.is_dense = true;

  _msg.data.resize(_msg.row_step * _msg.height);
  uint8_t * msgBufferIndex = _msg.data.data();

  for (uint32_t j = 0; j < height; ++j) {
    for (uint32_t i = 0; i < width; ++i) {
      *reinterpret_cast<float *>(msgBufferIndex + _msg.fields[0].offset) =
        j * width + i;
      msgBufferIndex += _msg.point_step;
    }
  }
}

void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::PointCloud2> & _msg)
{
  compareTestMsg(_msg->header);

  uint32_t height = 4;
  uint32_t width = 2;

  EXPECT_EQ(height, _msg->height);
  EXPECT_EQ(width, _msg->width);
  EXPECT_FALSE(_msg->is_bigendian);
  EXPECT_EQ(4u, _msg->point_step);
  EXPECT_EQ(4U * width, _msg->row_step);
  EXPECT_TRUE(_msg->is_dense);

  unsigned char * msgBufferIndex =
    const_cast<unsigned char *>(_msg->data.data());

  for (uint32_t j = 0; j < height; ++j) {
    for (uint32_t i = 0; i < width; ++i) {
      float * value =
        reinterpret_cast<float *>(msgBufferIndex + _msg->fields[0].offset);

      EXPECT_FLOAT_EQ(static_cast<float>(j * width + i), *value);
      msgBufferIndex += _msg->point_step;
    }
  }
}

void createTestMsg(sensor_msgs::msg::BatteryState & _msg)
{
  std_msgs::msg::Header header_msg;
  createTestMsg(header_msg);

  _msg.header = header_msg;
  _msg.voltage = 123;
  _msg.current = 456;
  _msg.charge = 789;
  _msg.capacity = 321;
  _msg.percentage = 654;
  _msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
}

void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::BatteryState> & _msg)
{
  sensor_msgs::msg::BatteryState expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  EXPECT_EQ(expected_msg.voltage, _msg->voltage);
  EXPECT_EQ(expected_msg.current, _msg->current);
  EXPECT_EQ(expected_msg.charge, _msg->charge);
  EXPECT_EQ(expected_msg.capacity, _msg->capacity);
  EXPECT_EQ(expected_msg.percentage, _msg->percentage);
  EXPECT_EQ(expected_msg.power_supply_status, _msg->power_supply_status);
}

void createTestMsg(trajectory_msgs::msg::JointTrajectoryPoint & _msg)
{
  const auto number_of_joints = 7;

  for (auto i = 0; i < number_of_joints; ++i) {
    _msg.positions.push_back(1.1 * i);
    _msg.velocities.push_back(2.2 * i);
    _msg.accelerations.push_back(3.3 * i);
    _msg.effort.push_back(4.4 * i);
  }
  _msg.time_from_start.sec = 12345;
  _msg.time_from_start.nanosec = 67890;
}

void compareTestMsg(const std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint> & _msg)
{
  trajectory_msgs::msg::JointTrajectoryPoint expected_msg;
  createTestMsg(expected_msg);

  for (auto i = 0u; i < _msg->positions.size(); ++i) {
    EXPECT_EQ(expected_msg.positions[i], _msg->positions[i]);
  }

  for (auto i = 0u; i < _msg->velocities.size(); ++i) {
    EXPECT_EQ(expected_msg.velocities[i], _msg->velocities[i]);
  }

  for (auto i = 0u; i < _msg->accelerations.size(); ++i) {
    EXPECT_EQ(expected_msg.accelerations[i], _msg->accelerations[i]);
  }

  for (auto i = 0u; i < _msg->effort.size(); ++i) {
    EXPECT_EQ(expected_msg.effort[i], _msg->effort[i]);
  }

  EXPECT_EQ(expected_msg.time_from_start.sec, _msg->time_from_start.sec);
  EXPECT_EQ(expected_msg.time_from_start.nanosec, _msg->time_from_start.nanosec);
}

void createTestMsg(trajectory_msgs::msg::JointTrajectory & _msg)
{
  const auto number_of_joints = 7;
  const auto number_of_trajectory_points = 10;

  std_msgs::msg::Header header_msg;
  createTestMsg(header_msg);
  _msg.header = header_msg;

  for (auto i = 0; i < number_of_joints; ++i) {
    _msg.joint_names.push_back("joint_" + std::to_string(i));
  }

  for (auto j = 0; j < number_of_trajectory_points; ++j) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    createTestMsg(point);
    _msg.points.push_back(point);
  }
}

void compareTestMsg(const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & _msg)
{
  trajectory_msgs::msg::JointTrajectory expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);

  for (auto i = 0u; i < _msg->joint_names.size(); ++i) {
    EXPECT_EQ(expected_msg.joint_names[i], _msg->joint_names[i]);
  }

  for (auto i = 0u; i < _msg->points.size(); ++i) {
    compareTestMsg(std::make_shared<trajectory_msgs::msg::JointTrajectoryPoint>(_msg->points[i]));
  }
}

}  // namespace testing
}  // namespace ros_ign_bridge
