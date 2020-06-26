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

    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')

    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
        #launch_arguments={
        #    'ign_args': '-r gpu_lidar.sdf'
        #}.items(),
    )

    # RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       # FIXME: Generate new RViz config once this demo is usable again
       # arguments=['-d', os.path.join(pkg_ros_ign_gazebo_demos, 'rviz', 'gpu_lidar.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan/'],
        output='screen'
    )

    # FIXME: need a SDF file (gpu_lidar.sdf) inside ros_ign_point_cloud/
    return LaunchDescription([
        ign_gazebo,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        bridge,
        rviz
    ])
