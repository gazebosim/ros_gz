# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest

from launch import LaunchDescription

from launch_ros.actions import Node

import launch_testing


def generate_test_description():

    publisher = Node(
        package='ros_ign_bridge',
        executable='test_ros_publisher',
        output='screen'
    )
    process_under_test = Node(
        package='ros_ign_bridge',
        executable='test_ign_subscriber',
        output='screen'
    )

    # Bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
          '/time@builtin_interfaces/msg/Time@ignition.msgs.Time',
          '/bool@std_msgs/msg/Bool@ignition.msgs.Boolean',
          '/color@std_msgs/msg/ColorRGBA@ignition.msgs.Color',
          '/empty@std_msgs/msg/Empty@ignition.msgs.Empty',
          '/float@std_msgs/msg/Float32@ignition.msgs.Float',
          '/double@std_msgs/msg/Float64@ignition.msgs.Double',
          '/uint32@std_msgs/msg/UInt32@ignition.msgs.UInt32',
          '/header@std_msgs/msg/Header@ignition.msgs.Header',
          '/string@std_msgs/msg/String@ignition.msgs.StringMsg',
          '/quaternion@geometry_msgs/msg/Quaternion@ignition.msgs.Quaternion',
          '/vector3@geometry_msgs/msg/Vector3@ignition.msgs.Vector3d',
          '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
          '/point@geometry_msgs/msg/Point@ignition.msgs.Vector3d',
          '/pose@geometry_msgs/msg/Pose@ignition.msgs.Pose',
          '/pose_with_covariance@geometry_msgs/msg/PoseWithCovariance@ignition.msgs.PoseWithCovariance',
          '/pose_stamped@geometry_msgs/msg/PoseStamped@ignition.msgs.Pose',
          '/transform@geometry_msgs/msg/Transform@ignition.msgs.Pose',
          '/tf2_message@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
          '/transform_stamped@geometry_msgs/msg/TransformStamped@ignition.msgs.Pose',
          '/twist@geometry_msgs/msg/Twist@ignition.msgs.Twist',
          '/twist_with_covariance@geometry_msgs/msg/TwistWithCovariance@ignition.msgs.TwistWithCovariance',
          '/wrench@geometry_msgs/msg/Wrench@ignition.msgs.Wrench',
          '/joint_wrench@ros_ign_interfaces/msg/JointWrench@ignition.msgs.JointWrench',
          '/entity@ros_ign_interfaces/msg/Entity@ignition.msgs.Entity',
          '/contact@ros_ign_interfaces/msg/Contact@ignition.msgs.Contact',
          '/contacts@ros_ign_interfaces/msg/Contacts@ignition.msgs.Contacts',
          '/light@ros_ign_interfaces/msg/Light@ignition.msgs.Light',
          '/gui_camera@ros_ign_interfaces/msg/GuiCamera@ignition.msgs.GUICamera',
          '/stringmsg_v@ros_ign_interfaces/msg/StringVec@ignition.msgs.StringMsg_V',
          '/track_visual@ros_ign_interfaces/msg/TrackVisual@ignition.msgs.TrackVisual',
          '/video_record@ros_ign_interfaces/msg/VideoRecord@ignition.msgs.VideoRecord',
          '/image@sensor_msgs/msg/Image@ignition.msgs.Image',
          '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
          '/fluid_pressure@sensor_msgs/msg/FluidPressure@ignition.msgs.FluidPressure',
          '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
          '/laserscan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
          '/magnetic@sensor_msgs/msg/MagneticField@ignition.msgs.Magnetometer',
          # '/actuators@mav_msgs/msg/Actuators@ignition.msgs.Actuators',
          '/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
          '/pointcloud2@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
          '/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model',
          '/battery_state@sensor_msgs/msg/BatteryState@ignition.msgs.BatteryState',
          '/joint_trajectory@trajectory_msgs/msg/JointTrajectory@ignition.msgs.JointTrajectory'
        ],
        output='screen'
    )
    return LaunchDescription([
        bridge,
        publisher,
        process_under_test,
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest(),
    ]), locals()


class IgnSubscriberTest(unittest.TestCase):

    def test_termination(self, process_under_test, proc_info):
        proc_info.assertWaitForShutdown(process=process_under_test, timeout=200)


@launch_testing.post_shutdown_test()
class IgnSubscriberTestAfterShutdown(unittest.TestCase):

    def test_exit_code(self, process_under_test, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            [launch_testing.asserts.EXIT_OK],
            process_under_test
        )
