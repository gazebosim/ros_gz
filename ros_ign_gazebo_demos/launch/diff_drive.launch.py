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

    pkg_ros_ign_gazebo_demos = get_package_share_directory('ros_ign_gazebo_demos')
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')

    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={
            'ign_args': '-r diff_drive.sdf'
        }.items(),
    )

    # RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_ros_ign_gazebo_demos, 'rviz', 'diff_drive.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                   '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                   '/model/vehicle_green/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                   '/model/vehicle_green/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry'],
        parameters=[{'qos_overrides./model/vehicle_blue.subscriber.reliability' : 'reliable',
                     'qos_overrides./model/vehicle_green.subscriber.reliability' : 'reliable'}],
        output='screen'
    )

    return LaunchDescription([
        ign_gazebo,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        bridge,
        rviz
    ])
