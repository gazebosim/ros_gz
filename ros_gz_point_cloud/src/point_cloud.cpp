// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include "point_cloud.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <span>  // NOLINT
#include <string>

#include <gz/common/Event.hh>
#include <gz/math/Helpers.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/DepthCamera.hh>
#include <gz/rendering/GpuRays.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/sim/components/DepthCamera.hh>
#include <gz/sim/components/GpuLidar.hh>
#include <gz/sim/components/RgbdCamera.hh>
#include <gz/sim/Util.hh>

#include <point_cloud_transport/point_cloud_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

GZ_ADD_PLUGIN(
  ros_gz_point_cloud::PointCloud,
  gz::sim::System,
  ros_gz_point_cloud::PointCloud::ISystemConfigure,
  ros_gz_point_cloud::PointCloud::ISystemPostUpdate)

using ros_gz_point_cloud::PointCloud;
using ros_gz_point_cloud::PointCloudPrivate;

/// \brief Types of sensors supported by this plugin
enum class SensorType
{
  /// \brief A camera which combines an RGB and a depth camera
  RGBD_CAMERA,

  /// \brief Depth camera
  DEPTH_CAMERA,

  /// \brief GPU lidar rays
  GPU_LIDAR
};

//////////////////////////////////////////////////
class ros_gz_point_cloud::PointCloudPrivate
{
  /// \brief Callback when the depth camera generates a new frame.
  /// This is called in the rendering thread.
  /// \param[in] _scan Depth image data
  /// \param[in] _width Image width in pixels
  /// \param[in] _height Image height in pixels
  /// \param[in] _channels Number of channels in image.
  /// \param[in] _format Image format as string.

public:
  void OnNewDepthFrame(
    const float * _scan,
    unsigned int _width, unsigned int _height,
    unsigned int _channels,
    const std::string & _format);

  /// \brief Callback when an RGBD camera generates a new coloured point cloud.
  /// This is called in the rendering thread.
  /// \param[in] _pointCloud XYZRGBA point cloud data
  /// \param[in] _width Image width in pixels
  /// \param[in] _height Image height in pixels
  /// \param[in] _depth Number of floats per point (X, Y, Z and a packed RGBA).
  /// \param[in] _format Point cloud format as string.

public:
  void OnNewRgbPointCloud(
    const float * _pointCloud,
    unsigned int _width, unsigned int _height,
    unsigned int _depth,
    const std::string & _format);

  /// \brief Get depth camera from rendering.
  /// \param[in] _ecm Immutable reference to ECM.

public:
  void LoadDepthCamera(const gz::sim::EntityComponentManager & _ecm);

  /// \brief Get RGB camera from rendering.
  /// \param[in] _ecm Immutable reference to ECM.

public:
  void LoadRgbCamera(const gz::sim::EntityComponentManager & _ecm);

  /// \brief Get GPU rays from rendering.
  /// \param[in] _ecm Immutable reference to ECM.

public:
  void LoadGpuRays(const gz::sim::EntityComponentManager & _ecm);

  /// \brief Rendering scene which manages the cameras.

public:
  gz::rendering::ScenePtr scene_;

  /// \brief Entity ID for sensor within Gazebo.

public:
  gz::sim::Entity entity_;

  /// \brief Rendering depth camera

public:
  std::shared_ptr<gz::rendering::DepthCamera> depth_camera_;

  /// \brief Rendering GPU lidar

public:
  std::shared_ptr<gz::rendering::GpuRays> gpu_rays_;

  /// \brief Connection to depth frame event.

public:
  gz::common::ConnectionPtr depth_connection_;

  /// \brief Connection to GPU rays frame event.

public:
  gz::common::ConnectionPtr gpu_rays_connection_;

  /// \brief Connection to the RGBD camera's coloured point cloud event.

public:
  gz::common::ConnectionPtr rgb_pc_connection_;

  /// \brief Node to publish ROS messages.

public:
  rclcpp::Node::SharedPtr rosnode_;

  /// \brief Point cloud transport instance used to advertise the publisher.

public:
  std::shared_ptr<point_cloud_transport::PointCloudTransport> pc_transport_;

  /// \brief Point cloud transport publisher.

public:
  point_cloud_transport::Publisher pc_pub_;

  /// \brief Current simulation time.

public:
  std::chrono::steady_clock::duration current_time_;

  /// \brief Frame ID to put in message header. Defaults to sensor scoped name.

public:
  std::string frame_id_;

  /// \brief Render engine name

public:
  std::string engine_name_;

  /// \brief Render scene name

public:
  std::string scene_name_;

  /// \brief Type of sensor which this plugin is attached to.

public:
  SensorType type_;
};

//////////////////////////////////////////////////
PointCloud::PointCloud()
: dataPtr(std::make_unique<PointCloudPrivate>())
{
}

//////////////////////////////////////////////////
void PointCloud::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager &)
{
  this->dataPtr->entity_ = _entity;

  using enum SensorType;
  if (_ecm.Component<gz::sim::components::RgbdCamera>(_entity) != nullptr) {
    this->dataPtr->type_ = RGBD_CAMERA;
  } else if (_ecm.Component<gz::sim::components::DepthCamera>(_entity) != nullptr) {
    this->dataPtr->type_ = DEPTH_CAMERA;
  } else if (_ecm.Component<gz::sim::components::GpuLidar>(_entity) != nullptr) {
    this->dataPtr->type_ = GPU_LIDAR;
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("ros_gz_point_cloud"),
      "Point cloud plugin must be attached to an RGBD camera, depth camera or GPU lidar.");
    return;
  }

  // Initialize ROS
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
    RCLCPP_INFO(rclcpp::get_logger("ros_gz_point_cloud"), "Initialized ROS");
  }

  // Sensor scoped name
  auto scoped_name = gz::sim::scopedName(this->dataPtr->entity_, _ecm, "/", false);

  // ROS node. A ROS 2 node name cannot contain '/', so derive a valid name
  // from the scoped name while the namespace is used to place the topics.
  auto ns = _sdf->Get<std::string>("namespace", scoped_name).first;
  std::string node_name = scoped_name;
  std::ranges::replace(node_name, '/', '_');
  this->dataPtr->rosnode_ = std::make_shared<rclcpp::Node>(node_name, ns);

  // Publisher
  auto topic = _sdf->Get<std::string>("topic", "points").first;
  this->dataPtr->pc_transport_ =
    std::make_shared<point_cloud_transport::PointCloudTransport>(*this->dataPtr->rosnode_);
  this->dataPtr->pc_pub_ = this->dataPtr->pc_transport_->advertise(topic, 1);

  // TF frame ID
  this->dataPtr->frame_id_ = _sdf->Get<std::string>("frame_id", scoped_name).first;

  // Rendering engine and scene
  this->dataPtr->engine_name_ = _sdf->Get<std::string>("engine", "ogre2").first;
  this->dataPtr->scene_name_ = _sdf->Get<std::string>("scene", "scene").first;
}

//////////////////////////////////////////////////
void PointCloud::PostUpdate(
  const gz::sim::UpdateInfo & _info,
  const gz::sim::EntityComponentManager & _ecm)
{
  this->dataPtr->current_time_ = _info.simTime;

  using enum SensorType;

  // Find engine / scene
  if (!this->dataPtr->scene_) {
    auto loadedEngNames = gz::rendering::loadedEngines();
    if (loadedEngNames.empty()) {
      RCLCPP_INFO(rclcpp::get_logger("ros_gz_point_cloud"), "No rendering engines loaded yet");
      return;
    }

    auto engine = gz::rendering::engine(this->dataPtr->engine_name_);
    if (!engine) {
      return;
    }

    this->dataPtr->scene_ = engine->SceneByIndex(0);
    if (!this->dataPtr->scene_) {
      return;
    }
    if (!this->dataPtr->scene_->IsInitialized()) {
      return;
    }
  }

  // Get rendering objects
  if (!this->dataPtr->depth_camera_ &&
    this->dataPtr->type_ == DEPTH_CAMERA)
  {
    this->dataPtr->LoadDepthCamera(_ecm);
  }
  if (!this->dataPtr->rgb_pc_connection_ &&
    this->dataPtr->type_ == RGBD_CAMERA)
  {
    this->dataPtr->LoadRgbCamera(_ecm);
  }
  if (!this->dataPtr->gpu_rays_ &&
    this->dataPtr->type_ == GPU_LIDAR)
  {
    this->dataPtr->LoadGpuRays(_ecm);
  }
}

//////////////////////////////////////////////////
void PointCloudPrivate::LoadDepthCamera(
  const gz::sim::EntityComponentManager & _ecm)
{
  // Sensor name scoped from the model
  auto sensor_name =
    gz::sim::scopedName(this->entity_, _ecm, "::", false);
  sensor_name = sensor_name.substr(sensor_name.find("::") + 2);

  // Get sensor
  auto sensor = this->scene_->SensorByName(sensor_name + "_depth");
  if (!sensor) {
    sensor = this->scene_->SensorByName(sensor_name);
    if (!sensor) {
      return;
    }
  }

  this->depth_camera_ =
    std::dynamic_pointer_cast<gz::rendering::DepthCamera>(sensor);
  if (!this->depth_camera_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ros_gz_point_cloud"),
      "Rendering sensor named [%s] is not a depth camera", sensor_name.c_str());
    return;
  }

  this->depth_connection_ = this->depth_camera_->ConnectNewDepthFrame(
    [this](
      const float * _scan, unsigned int _width, unsigned int _height,
      unsigned int _channels, const std::string & _format)
    {
      this->OnNewDepthFrame(_scan, _width, _height, _channels, _format);
    });
}

//////////////////////////////////////////////////
void PointCloudPrivate::LoadRgbCamera(
  const gz::sim::EntityComponentManager & _ecm)
{
  // Sensor name scoped from the model
  auto sensor_name =
    gz::sim::scopedName(this->entity_, _ecm, "::", false);
  sensor_name = sensor_name.substr(sensor_name.find("::") + 2);

  // Get sensor
  auto sensor = this->scene_->SensorByName(sensor_name + "_depth");
  if (!sensor) {
    sensor = this->scene_->SensorByName(sensor_name);
    if (!sensor) {
      return;
    }
  }

  this->depth_camera_ =
    std::dynamic_pointer_cast<gz::rendering::DepthCamera>(sensor);
  if (!this->depth_camera_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ros_gz_point_cloud"),
      "Rendering sensor named [%s] is not a depth camera", sensor_name.c_str());
    return;
  }

  this->rgb_pc_connection_ = this->depth_camera_->ConnectNewRgbPointCloud(
    [this](
      const float * _pointCloud, unsigned int _width, unsigned int _height,
      unsigned int _depth, const std::string & _format)
    {
      this->OnNewRgbPointCloud(_pointCloud, _width, _height, _depth, _format);
    });
}

//////////////////////////////////////////////////
void PointCloudPrivate::OnNewRgbPointCloud(
  const float * _pointCloud,
  unsigned int _width, unsigned int _height,
  unsigned int _depth,
  const std::string &)
{
  if (this->pc_pub_.getNumSubscribers() == 0 || _height == 0 || _width == 0) {
    return;
  }

  const auto sec_nsec = gz::math::durationToSecNsec(this->current_time_);

  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = this->frame_id_;
  msg.header.stamp.sec = sec_nsec.first;
  msg.header.stamp.nanosec = sec_nsec.second;
  msg.width = _width;
  msg.height = _height;
  msg.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(static_cast<std::size_t>(_width) * _height);

  sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(msg, "b");

  // gz delivers each point as _depth floats: X, Y, Z and a packed RGBA float.
  const std::span<const float> cloud(
    _pointCloud, static_cast<std::size_t>(_width) * _height * _depth);

  for (std::size_t i = 0; i + _depth <= cloud.size();
    i += _depth, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
  {
    *iter_x = cloud[i + 0];
    *iter_y = cloud[i + 1];
    *iter_z = cloud[i + 2];

    // The 4th float packs the colour; its bytes are R, G, B, A.
    const float rgba = cloud[i + 3];
    const auto * color = reinterpret_cast<const uint8_t *>(&rgba);
    *iter_r = color[0];
    *iter_g = color[1];
    *iter_b = color[2];
  }

  this->pc_pub_.publish(msg);
}

//////////////////////////////////////////////////
void PointCloudPrivate::LoadGpuRays(
  const gz::sim::EntityComponentManager & _ecm)
{
  // Sensor name scoped from the model
  auto sensor_name =
    gz::sim::scopedName(this->entity_, _ecm, "::", false);
  sensor_name = sensor_name.substr(sensor_name.find("::") + 2);

  // Get sensor
  auto sensor = this->scene_->SensorByName(sensor_name);
  if (!sensor) {
    return;
  }

  this->gpu_rays_ =
    std::dynamic_pointer_cast<gz::rendering::GpuRays>(sensor);
  if (!this->gpu_rays_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ros_gz_point_cloud"),
      "Rendering sensor named [%s] is not a GPU rays sensor", sensor_name.c_str());
    return;
  }

  this->gpu_rays_connection_ = this->gpu_rays_->ConnectNewGpuRaysFrame(
    [this](
      const float * _scan, unsigned int _width, unsigned int _height,
      unsigned int _channels, const std::string & _format)
    {
      this->OnNewDepthFrame(_scan, _width, _height, _channels, _format);
    });
}

//////////////////////////////////////////////////
void PointCloudPrivate::OnNewDepthFrame(
  const float * _scan,
  unsigned int _width, unsigned int _height,
  unsigned int _channels,
  const std::string & _format)
{
  using enum SensorType;

  if (this->pc_pub_.getNumSubscribers() == 0 || _height == 0 || _width == 0) {
    return;
  }

  // Just sanity check, but don't prevent publishing
  if (this->type_ == RGBD_CAMERA && _channels != 1) {
    RCLCPP_WARN(
      rclcpp::get_logger("ros_gz_point_cloud"),
      "Expected depth image to have 1 channel, but it has [%i]", _channels);
  }
  if (this->type_ == GPU_LIDAR && _channels != 3) {
    RCLCPP_WARN(
      rclcpp::get_logger("ros_gz_point_cloud"),
      "Expected GPU rays to have 3 channels, but it has [%i]", _channels);
  }
  if ((this->type_ == RGBD_CAMERA ||
    this->type_ == DEPTH_CAMERA) && _format != "FLOAT32")
  {
    RCLCPP_WARN(
      rclcpp::get_logger("ros_gz_point_cloud"),
      "Expected depth image to have [FLOAT32] format, but it has [%s]", _format.c_str());
  }
  if (this->type_ == GPU_LIDAR && _format != "PF_FLOAT32_RGB") {
    RCLCPP_WARN(
      rclcpp::get_logger("ros_gz_point_cloud"),
      "Expected GPU rays to have [PF_FLOAT32_RGB] format, but it has [%s]", _format.c_str());
  }

  // Fill message
  // Logic borrowed from
  // https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_depth_camera.cpp
  auto sec_nsec = gz::math::durationToSecNsec(this->current_time_);

  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = this->frame_id_;
  msg.header.stamp.sec = sec_nsec.first;
  msg.header.stamp.nanosec = sec_nsec.second;
  msg.width = _width;
  msg.height = _height;
  msg.row_step = msg.point_step * _width;
  msg.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(_width * _height);

  sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(msg, "b");

  // For depth calculation from image
  double fl {0.0};
  if (nullptr != this->depth_camera_) {
    auto hfov = this->depth_camera_->HFOV().Radian();
    fl = _width / (2.0 * tan(hfov / 2.0));
  }

  // For depth calculation from laser scan
  double angle_step {0.0};
  double vertical_angle_step {0.0};
  double inclination {0.0};
  double azimuth {0.0};
  if (nullptr != this->gpu_rays_) {
    angle_step = (this->gpu_rays_->AngleMax() - this->gpu_rays_->AngleMin()).Radian() /
      (this->gpu_rays_->RangeCount() - 1);
    vertical_angle_step = (this->gpu_rays_->VerticalAngleMax() -
      this->gpu_rays_->VerticalAngleMin()).Radian() / (this->gpu_rays_->VerticalRangeCount() - 1);

    // Angles of ray currently processing, azimuth is horizontal, inclination is vertical
    inclination = this->gpu_rays_->VerticalAngleMin().Radian();
    azimuth = this->gpu_rays_->AngleMin().Radian();
  }

  // View the raw scan buffer as a bounds-aware span instead of a bare pointer.
  const std::span<const float> scan(
    _scan, static_cast<std::size_t>(_width) * _height * _channels);

  // Iterate over scan and populate point cloud
  for (uint32_t j = 0; j < _height; ++j) {
    double p_angle {0.0};
    if (fl > 0 && _height > 1) {
      p_angle = atan2(static_cast<double>(j) - 0.5 * static_cast<double>(_height - 1), fl);
    }

    if (nullptr != this->gpu_rays_) {
      azimuth = this->gpu_rays_->AngleMin().Radian();
    }
    for (uint32_t i = 0; i < _width;
      ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
    {
      // Index of current point
      auto index = j * _width * _channels + i * _channels;
      double depth = scan[index];

      double y_angle {0.0};
      if (fl > 0 && _width > 1) {
        y_angle = atan2(static_cast<double>(i) - 0.5 * static_cast<double>(_width - 1), fl);
      }

      if (nullptr != this->depth_camera_) {
        // in optical frame
        // hardcoded rotation rpy(-M_PI/2, 0, -M_PI/2) is built-in
        // to urdf, where the *_optical_frame should have above relative
        // rotation from the physical camera *_frame
        *iter_x = depth * tan(y_angle);
        *iter_y = depth * tan(p_angle);
        *iter_z = depth;

        // Clamp according to REP 117
        if (depth > this->depth_camera_->FarClipPlane()) {
          *iter_z = gz::math::INF_D;
          msg.is_dense = false;
        }
        if (depth < this->depth_camera_->NearClipPlane()) {
          *iter_z = -gz::math::INF_D;
          msg.is_dense = false;
        }
      } else if (nullptr != this->gpu_rays_) {
        // Convert spherical coordinates to Cartesian for pointcloud
        // See https://en.wikipedia.org/wiki/Spherical_coordinate_system
        *iter_x = depth * cos(inclination) * cos(azimuth);
        *iter_y = depth * cos(inclination) * sin(azimuth);
        *iter_z = depth * sin(inclination);
      }

      // Depth and lidar clouds carry no colour information.
      *iter_r = 0;
      *iter_g = 0;
      *iter_b = 0;
      azimuth += angle_step;
    }
    inclination += vertical_angle_step;
  }

  this->pc_pub_.publish(msg);
}
