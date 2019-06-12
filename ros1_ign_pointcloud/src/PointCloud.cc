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

#include <mutex>

#include "PointCloud.hh"
#include <ignition/common/Event.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/RgbdCamera.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/DepthCamera.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/sensors/RgbdCameraSensor.hh>

#include <ros/ros.h>
#include <ros/advertise_options.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/point_cloud2_iterator.h>

IGNITION_ADD_PLUGIN(
    ros1_ign_pointcloud::PointCloud,
    ignition::gazebo::System,
    ros1_ign_pointcloud::PointCloud::ISystemConfigure,
    ros1_ign_pointcloud::PointCloud::ISystemPostUpdate)

using namespace ros1_ign_pointcloud;

//////////////////////////////////////////////////
class ros1_ign_pointcloud::PointCloudPrivate
{
  public: void OnNewDepthFrame(const float *_scan,
            unsigned int _width, unsigned int _height,
            unsigned int /*_channels*/,
            const std::string &/*_format*/);

  public: void LoadDepthCamera(const ignition::gazebo::EntityComponentManager &_ecm);
  public: void LoadRgbCamera(const ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief rendering scene to be managed by the scene manager and used to
  /// generate sensor data
  public: ignition::rendering::ScenePtr scene;

  public: ignition::gazebo::Entity entity;

  public: std::shared_ptr<ignition::rendering::DepthCamera> depthCamera;
  public: std::shared_ptr<ignition::rendering::Camera> rgbCamera;
  public: ignition::rendering::Image image;

  public: ignition::common::ConnectionPtr newDepthFrameConnection;

  /// \brief Depth data buffer.
  public: float *depthBuffer = nullptr;

  public: std::unique_ptr<ros::NodeHandle> rosnode;
  public: ros::Publisher pcPub;
  public: sensor_msgs::Image imageMsg;

  public: std::chrono::steady_clock::duration currentTime;

  /// \brief Mutex to protect performersToAdd list.
  public: std::mutex scanMutex;
};

//////////////////////////////////////////////////
PointCloud::PointCloud() : dataPtr(std::make_unique<PointCloudPrivate>())
{
}

//////////////////////////////////////////////////
void PointCloud::Configure(const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &,
    ignition::gazebo::EntityComponentManager &,
    ignition::gazebo::EventManager &)
{
  this->dataPtr->entity = _entity;

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "ignition", ros::init_options::NoSigintHandler);
    ROS_INFO_NAMED("point_cloud", "Initialized ROS");
  }
}

//////////////////////////////////////////////////
void PointCloud::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  // Find engine / scene
  if (!this->dataPtr->scene)
  {
    auto engine = ignition::rendering::engine("ogre2");
    this->dataPtr->scene = engine->SceneByName("scene");
    if (!this->dataPtr->scene)
      return;
  }

  // Get rendering cameras
  if (!this->dataPtr->depthCamera)
  {
    this->dataPtr->LoadDepthCamera(_ecm);
  }
  if (!this->dataPtr->rgbCamera)
  {
    this->dataPtr->LoadRgbCamera(_ecm);
  }

  this->dataPtr->currentTime = _info.simTime;
}

//////////////////////////////////////////////////
void PointCloudPrivate::LoadDepthCamera(
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  // Sensor name scoped from the model
  auto sensorName =
      ignition::gazebo::scopedName(this->entity, _ecm, "::", false);
  sensorName = sensorName.substr(sensorName.find("::") + 2);

  // Get sensor
  auto sensor = this->scene->SensorByName(sensorName + "_depth");
  if (!sensor)
  {
    return;
  }

  this->depthCamera =
    std::dynamic_pointer_cast<ignition::rendering::DepthCamera>(sensor);
  if (!this->depthCamera)
  {
    return;
  }

  this->newDepthFrameConnection = this->depthCamera->ConnectNewDepthFrame(
      std::bind(&PointCloudPrivate::OnNewDepthFrame, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5));

  this->rosnode = std::make_unique<ros::NodeHandle>(
      ignition::gazebo::scopedName(this->entity, _ecm, "/", false));

  this->pcPub = this->rosnode->advertise<sensor_msgs::PointCloud2>("points", 1);
}

//////////////////////////////////////////////////
void PointCloudPrivate::LoadRgbCamera(
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  // Sensor name scoped from the model
  auto sensorName =
      ignition::gazebo::scopedName(this->entity, _ecm, "::", false);
  sensorName = sensorName.substr(sensorName.find("::") + 2);

  // Get sensor
  auto sensor = this->scene->SensorByName(sensorName);
  if (!sensor)
  {
    return;
  }

  this->rgbCamera = std::dynamic_pointer_cast<ignition::rendering::Camera>(sensor);
  if (!this->rgbCamera)
  {
    return;
  }

  this->image = this->rgbCamera->CreateImage();
}

//////////////////////////////////////////////////
void PointCloudPrivate::OnNewDepthFrame(const float *_scan,
                    unsigned int _width, unsigned int _height,
                    unsigned int /*_channels*/,
                    const std::string &/*_format*/)
{
  if (this->pcPub.getNumSubscribers() <= 0 || _height == 0 || _width == 0)
    return;

  auto secNsec = ignition::math::durationToSecNsec(this->currentTime);

  sensor_msgs::PointCloud2 msg;
  msg.header.frame_id = "map"; // TODO
  msg.header.stamp.sec = secNsec.first;
  msg.header.stamp.nsec = secNsec.second;
  msg.width = _width;
  msg.height = _height;
  msg.row_step = msg.point_step * _width;
  msg.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(_width*_height);

  sensor_msgs::PointCloud2Iterator<float> iterX(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iterY(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iterZ(msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iterRgb(msg, "rgb");

  if (this->rgbCamera)
  {
    this->rgbCamera->Capture(this->image);
    fillImage(this->imageMsg, sensor_msgs::image_encodings::RGB8, _height,
        _width, 3 * _width, this->image.Data<unsigned char>());
  }

  std::lock_guard<std::mutex> lock(this->scanMutex);

  double hfov = this->depthCamera->HFOV().Radian();
  double fl = _width / (2.0 * tan(hfov/2.0));
  int index{0};
  uint8_t *image_src = (uint8_t *)(&(this->imageMsg.data[0]));

  // Convert depth to point cloud
  for (uint32_t j = 0; j < _height; ++j)
  {
    double pAngle;
    if (_height>1)
      pAngle = atan2( (double)j - 0.5*(double)(_height-1), fl);
    else
      pAngle = 0.0;

    for (uint32_t i=0; i<_width; i++, ++iterX, ++iterY, ++iterZ, ++iterRgb)
    {
      double yAngle;
      if (_width>1)
        yAngle = atan2( (double)i - 0.5*(double)(_width-1), fl);
      else
        yAngle = 0.0;

      double depth = _scan[index++];

      // in optical frame
      // hardcoded rotation rpy(-M_PI/2, 0, -M_PI/2) is built-in
      // to urdf, where the *_optical_frame should have above relative
      // rotation from the physical camera *_frame
      *iterX = depth * tan(yAngle);
      *iterY = depth * tan(pAngle);
      if (depth > this->depthCamera->FarClipPlane())
      {
        *iterZ = ignition::math::INF_D;
        msg.is_dense = false;
      }
      if (depth < this->depthCamera->NearClipPlane())
      {
        *iterZ = -ignition::math::INF_D;
        msg.is_dense = false;
      }
      else
      {
        *iterZ = depth;
      }

      // put image color data for each point
      if (this->imageMsg.data.size() == _height*_width*3)
      {
        // color
        // TODO why does BGR work instead of RGB?!?!
        iterRgb[2] = image_src[i*3+j*_width*3+0];
        iterRgb[1] = image_src[i*3+j*_width*3+1];
        iterRgb[0] = image_src[i*3+j*_width*3+2];
      }
      else if (this->imageMsg.data.size() == _height*_width)
      {
        // mono?
        iterRgb[0] = image_src[i+j*_width];
        iterRgb[1] = image_src[i+j*_width];
        iterRgb[2] = image_src[i+j*_width];
      }
      else
      {
        // no image
        iterRgb[0] = 0;
        iterRgb[1] = 0;
        iterRgb[2] = 0;
      }
    }
  }

  this->pcPub.publish(msg);
}

