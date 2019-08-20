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

#include "point_cloud.hh"
#include <ignition/common/Event.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/DepthCamera.hh>
#include <ignition/gazebo/components/GpuLidar.hh>
#include <ignition/gazebo/components/RgbdCamera.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/DepthCamera.hh>
#include <ignition/rendering/GpuRays.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>

#include <ros/ros.h>
#include <ros/advertise_options.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/point_cloud2_iterator.h>

IGNITION_ADD_PLUGIN(
    ros_ign_point_cloud::PointCloud,
    ignition::gazebo::System,
    ros_ign_point_cloud::PointCloud::ISystemConfigure,
    ros_ign_point_cloud::PointCloud::ISystemPostUpdate)

using namespace ros_ign_point_cloud;

/// \brief Types of sensors supported by this plugin
enum class SensorType {
  /// \brief A camera which combines an RGB and a depth camera
  RGBD_CAMERA,

  /// \brief Depth camera
  DEPTH_CAMERA,

  /// \brief GPU lidar rays
  GPU_LIDAR
};

//////////////////////////////////////////////////
class ros_ign_point_cloud::PointCloudPrivate
{
  /// \brief Callback when the depth camera generates a new frame.
  /// This is called in the rendering thread.
  /// \param[in] _scan Depth image data
  /// \param[in] _width Image width in pixels
  /// \param[in] _height Image height in pixels
  /// \param[in] _channels Number of channels in image.
  /// \param[in] _format Image format as string.
  public: void OnNewDepthFrame(const float *_scan,
            unsigned int _width, unsigned int _height,
            unsigned int _channels,
            const std::string &_format);

  /// \brief Get depth camera from rendering.
  /// \param[in] _ecm Immutable reference to ECM.
  public: void LoadDepthCamera(const ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Get RGB camera from rendering.
  /// \param[in] _ecm Immutable reference to ECM.
  public: void LoadRgbCamera(const ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Get GPU rays from rendering.
  /// \param[in] _ecm Immutable reference to ECM.
  public: void LoadGpuRays(const ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Rendering scene which manages the cameras.
  public: ignition::rendering::ScenePtr scene_;

  /// \brief Entity ID for sensor within Gazebo.
  public: ignition::gazebo::Entity entity_;

  /// \brief Rendering depth camera
  public: std::shared_ptr<ignition::rendering::DepthCamera> depth_camera_;

  /// \brief Rendering RGB camera
  public: std::shared_ptr<ignition::rendering::Camera> rgb_camera_;

  /// \brief Rendering GPU lidar
  public: std::shared_ptr<ignition::rendering::GpuRays> gpu_rays_;

  /// \brief Keep latest image from RGB camera.
  public: ignition::rendering::Image rgb_image_;

  /// \brief Message populated with latest image from RGB camera.
  public: sensor_msgs::Image rgb_image_msg_;

  /// \brief Connection to depth frame event.
  public: ignition::common::ConnectionPtr depth_connection_;

  /// \brief Connection to GPU rays frame event.
  public: ignition::common::ConnectionPtr gpu_rays_connection_;

  /// \brief Node to publish ROS messages.
  public: std::unique_ptr<ros::NodeHandle> rosnode_;

  /// \brief Point cloud ROS publisher.
  public: ros::Publisher pc_pub_;

  /// \brief Current simulation time.
  public: std::chrono::steady_clock::duration current_time_;

  /// \brief Frame ID to put in message header. Defaults to sensor scoped name.
  public: std::string frame_id_;

  /// \brief Render engine name
  public: std::string engine_name_;

  /// \brief Render scene name
  public: std::string scene_name_;

  /// \brief Type of sensor which this plugin is attached to.
  public: SensorType type_;
};

//////////////////////////////////////////////////
PointCloud::PointCloud() : dataPtr(std::make_unique<PointCloudPrivate>())
{
}

//////////////////////////////////////////////////
void PointCloud::Configure(const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &)
{
  this->dataPtr->entity_ = _entity;

  if (_ecm.Component<ignition::gazebo::components::RgbdCamera>(_entity) != nullptr)
  {
    this->dataPtr->type_ = SensorType::RGBD_CAMERA;
  }
  else if (_ecm.Component<ignition::gazebo::components::DepthCamera>(_entity) != nullptr)
  {
    this->dataPtr->type_ = SensorType::DEPTH_CAMERA;
  }
  else if (_ecm.Component<ignition::gazebo::components::GpuLidar>(_entity) != nullptr)
  {
    this->dataPtr->type_ = SensorType::GPU_LIDAR;
  }
  else
  {
    ROS_ERROR_NAMED("ros_ign_point_cloud",
        "Point cloud plugin must be attached to an RGBD camera, depth camera or GPU lidar.");
    return;
  }

  // Initialize ROS
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "ignition", ros::init_options::NoSigintHandler);
    ROS_INFO_NAMED("ros_ign_point_cloud", "Initialized ROS");
  }

  // Sensor scoped name
  auto scoped_name = ignition::gazebo::scopedName(this->dataPtr->entity_, _ecm, "/", false);

  // ROS node
  auto ns = _sdf->Get<std::string>("namespace", scoped_name).first;
  this->dataPtr->rosnode_ = std::make_unique<ros::NodeHandle>(ns);

  // Publisher
  auto topic = _sdf->Get<std::string>("topic", "points").first;
  this->dataPtr->pc_pub_ = this->dataPtr->rosnode_->advertise<sensor_msgs::PointCloud2>(topic, 1);

  // TF frame ID
  this->dataPtr->frame_id_ = _sdf->Get<std::string>("frame_id", scoped_name).first;

  // Rendering engine and scene
  this->dataPtr->engine_name_ = _sdf->Get<std::string>("engine", "ogre2").first;
  this->dataPtr->scene_name_ = _sdf->Get<std::string>("scene", "scene").first;
}

//////////////////////////////////////////////////
void PointCloud::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  this->dataPtr->current_time_ = _info.simTime;

  // Find engine / scene
  if (!this->dataPtr->scene_)
  {
    auto engine = ignition::rendering::engine(this->dataPtr->engine_name_);
    if (!engine)
      return;

    this->dataPtr->scene_ = engine->SceneByName(this->dataPtr->scene_name_);
    if (!this->dataPtr->scene_)
      return;
  }

  // Get rendering objects
  if (!this->dataPtr->depth_camera_ &&
      (this->dataPtr->type_ == SensorType::RGBD_CAMERA ||
       this->dataPtr->type_ == SensorType::DEPTH_CAMERA))
  {
    this->dataPtr->LoadDepthCamera(_ecm);
  }
  if (!this->dataPtr->rgb_camera_ &&
       this->dataPtr->type_ == SensorType::RGBD_CAMERA)
  {
    this->dataPtr->LoadRgbCamera(_ecm);
  }
  if (!this->dataPtr->gpu_rays_ &&
       this->dataPtr->type_ == SensorType::GPU_LIDAR)
  {
    this->dataPtr->LoadGpuRays(_ecm);
  }
}

//////////////////////////////////////////////////
void PointCloudPrivate::LoadDepthCamera(
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  // Sensor name scoped from the model
  auto sensor_name =
      ignition::gazebo::scopedName(this->entity_, _ecm, "::", false);
  sensor_name = sensor_name.substr(sensor_name.find("::") + 2);

  // Get sensor
  auto sensor = this->scene_->SensorByName(sensor_name + "_depth");
  if (!sensor)
  {
    sensor = this->scene_->SensorByName(sensor_name);
    if (!sensor)
    {
      return;
    }
  }

  this->depth_camera_ =
    std::dynamic_pointer_cast<ignition::rendering::DepthCamera>(sensor);
  if (!this->depth_camera_)
  {
    ROS_ERROR_NAMED("ros_ign_point_cloud",
        "Rendering sensor named [%s] is not a depth camera", sensor_name.c_str());
    return;
  }

  this->depth_connection_ = this->depth_camera_->ConnectNewDepthFrame(
      std::bind(&PointCloudPrivate::OnNewDepthFrame, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5));
}

//////////////////////////////////////////////////
void PointCloudPrivate::LoadRgbCamera(
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  // Sensor name scoped from the model
  auto sensor_name =
      ignition::gazebo::scopedName(this->entity_, _ecm, "::", false);
  sensor_name = sensor_name.substr(sensor_name.find("::") + 2);

  // Get sensor
  auto sensor = this->scene_->SensorByName(sensor_name);
  if (!sensor)
  {
    return;
  }

  this->rgb_camera_ = std::dynamic_pointer_cast<ignition::rendering::Camera>(sensor);
  if (!this->rgb_camera_)
  {
    ROS_ERROR_NAMED("ros_ign_point_cloud",
        "Rendering sensor named [%s] is not an RGB camera", sensor_name.c_str());
    return;
  }

  this->rgb_image_ = this->rgb_camera_->CreateImage();
}

//////////////////////////////////////////////////
void PointCloudPrivate::LoadGpuRays(
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  // Sensor name scoped from the model
  auto sensor_name =
      ignition::gazebo::scopedName(this->entity_, _ecm, "::", false);
  sensor_name = sensor_name.substr(sensor_name.find("::") + 2);

  // Get sensor
  auto sensor = this->scene_->SensorByName(sensor_name);
  if (!sensor)
  {
    return;
  }

  this->gpu_rays_ =
    std::dynamic_pointer_cast<ignition::rendering::GpuRays>(sensor);
  if (!this->gpu_rays_)
  {
    ROS_ERROR_NAMED("ros_ign_point_cloud",
        "Rendering sensor named [%s] is not a depth camera", sensor_name.c_str());
    return;
  }

  this->gpu_rays_connection_ = this->gpu_rays_->ConnectNewGpuRaysFrame(
      std::bind(&PointCloudPrivate::OnNewDepthFrame, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5));
}

//////////////////////////////////////////////////
void PointCloudPrivate::OnNewDepthFrame(const float *_scan,
                    unsigned int _width, unsigned int _height,
                    unsigned int _channels,
                    const std::string &_format)
{
  if (this->pc_pub_.getNumSubscribers() <= 0 || _height == 0 || _width == 0)
    return;

  // Just sanity check, but don't prevent publishing
  if (this->type_ == SensorType::RGBD_CAMERA && _channels != 1)
  {
    ROS_WARN_NAMED("ros_ign_point_cloud",
        "Expected depth image to have 1 channel, but it has [%i]", _channels);
  }
  if (this->type_ == SensorType::GPU_LIDAR && _channels != 3)
  {
    ROS_WARN_NAMED("ros_ign_point_cloud",
        "Expected GPU rays to have 3 channels, but it has [%i]", _channels);
  }
  if ((this->type_ == SensorType::RGBD_CAMERA ||
       this->type_ == SensorType::DEPTH_CAMERA) && _format != "FLOAT32")
  {
    ROS_WARN_NAMED("ros_ign_point_cloud",
        "Expected depth image to have [FLOAT32] format, but it has [%s]", _format.c_str());
  }
  if (this->type_ == SensorType::GPU_LIDAR && _format != "PF_FLOAT32_RGB")
  {
    ROS_WARN_NAMED("ros_ign_point_cloud",
        "Expected GPU rays to have [PF_FLOAT32_RGB] format, but it has [%s]", _format.c_str());
  }

  // Fill message
  // Logic borrowed from
  // https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_depth_camera.cpp
  auto sec_nsec = ignition::math::durationToSecNsec(this->current_time_);

  sensor_msgs::PointCloud2 msg;
  msg.header.frame_id = this->frame_id_;
  msg.header.stamp.sec = sec_nsec.first;
  msg.header.stamp.nsec = sec_nsec.second;
  msg.width = _width;
  msg.height = _height;
  msg.row_step = msg.point_step * _width;
  msg.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(_width*_height);

  sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(msg, "b");

  if (this->rgb_camera_)
  {
    this->rgb_camera_->Capture(this->rgb_image_);
    fillImage(this->rgb_image_msg_, sensor_msgs::image_encodings::RGB8, _height,
        _width, 3 * _width, this->rgb_image_.Data<unsigned char>());
  }

  // For depth calculation from image
  double fl{0.0};
  if (nullptr != this->depth_camera_)
  {
    auto hfov = this->depth_camera_->HFOV().Radian();
    fl = _width / (2.0 * tan(hfov / 2.0));
  }

  // For depth calculation from laser scan
  double angle_step{0.0};
  double vertical_angle_step{0.0};
  double inclination{0.0};
  double azimuth{0.0};
  if (nullptr != this->gpu_rays_)
  {
    angle_step = (this->gpu_rays_->AngleMax() - this->gpu_rays_->AngleMin()).Radian() /
        (this->gpu_rays_->RangeCount()-1);
    vertical_angle_step = (this->gpu_rays_->VerticalAngleMax() -
        this->gpu_rays_->VerticalAngleMin()).Radian() / (this->gpu_rays_->VerticalRangeCount()-1);

    // Angles of ray currently processing, azimuth is horizontal, inclination is vertical
    inclination = this->gpu_rays_->VerticalAngleMin().Radian();
    azimuth = this->gpu_rays_->AngleMin().Radian();
  }

  // For color calculation
  uint8_t * image_src;
  if (nullptr != this->rgb_camera_)
  {
    image_src = (uint8_t *)(&(this->rgb_image_msg_.data[0]));
  }

  // Iterate over scan and populate point cloud
  for (uint32_t j = 0; j < _height; ++j)
  {
    double p_angle{0.0};
    if (fl > 0 && _height > 1)
      p_angle = atan2((double)j - 0.5 * (double)(_height-1), fl);

    if (nullptr != this->gpu_rays_)
    {
      azimuth = this->gpu_rays_->AngleMin().Radian();
    }
    for (uint32_t i = 0; i < _width; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
    {
      // Index of current point
      auto index = j * _width * _channels + i * _channels;
      double depth = _scan[index];

      double y_angle{0.0};
      if (fl > 0 && _width > 1)
        y_angle = atan2((double)i - 0.5 * (double)(_width-1), fl);

      if (nullptr != this->depth_camera_)
      {
        // in optical frame
        // hardcoded rotation rpy(-M_PI/2, 0, -M_PI/2) is built-in
        // to urdf, where the *_optical_frame should have above relative
        // rotation from the physical camera *_frame
        *iter_x = depth * tan(y_angle);
        *iter_y = depth * tan(p_angle);
        *iter_z = depth;

        // Clamp according to REP 117
        if (depth > this->depth_camera_->FarClipPlane())
        {
          *iter_z = ignition::math::INF_D;
          msg.is_dense = false;
        }
        if (depth < this->depth_camera_->NearClipPlane())
        {
          *iter_z = -ignition::math::INF_D;
          msg.is_dense = false;
        }
      }
      else if (nullptr != this->gpu_rays_)
      {
        // Convert spherical coordinates to Cartesian for pointcloud
        // See https://en.wikipedia.org/wiki/Spherical_coordinate_system
        *iter_x = depth * cos(inclination) * cos(azimuth);
        *iter_y = depth * cos(inclination) * sin(azimuth);
        *iter_z = depth * sin(inclination);
      }

      // Put image color data for each point
      if (this->rgb_image_msg_.data.size() == _height * _width * 3)
      {
        // color
        *iter_r = image_src[i*3+j*_width*3+0];
        *iter_g = image_src[i*3+j*_width*3+1];
        *iter_b = image_src[i*3+j*_width*3+2];
      }
      else if (this->rgb_image_msg_.data.size() == _height*_width)
      {
        // mono?
        *iter_r = image_src[i+j*_width];
        *iter_g = image_src[i+j*_width];
        *iter_b = image_src[i+j*_width];
      }
      else
      {
        // no image
        *iter_r = 0;
        *iter_g = 0;
        *iter_b = 0;
      }
      azimuth += angle_step;
    }
    inclination += vertical_angle_step;
  }

  this->pc_pub_.publish(msg);
}

