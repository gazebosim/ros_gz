# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-r sensors.sdf'
        }.items(),
    )

    # RQt
    rqt = Node(
        package='rqt_topic',
        executable='rqt_topic',
        arguments=['-t'],
        condition=IfCondition(LaunchConfiguration('rqt'))
    )

    # RViz
    # FIXME: Add once there's an IMU display for RViz2
    # pkg_ros_gz_sim_demos = get_package_share_directory('ros_gz_sim_demos')
    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     # arguments=['-d', os.path.join(pkg_ros_gz_sim_demos, 'rviz', 'imu.rviz')],
    #     condition=IfCondition(LaunchConfiguration('rviz'))
    # )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument(
            'rqt', default_value='true', description='Open RQt.'
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true', description='Open RViz.'
        ),
        bridge,
        rqt
        # rviz
    ])
