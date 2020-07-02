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

    # RQt
    rqt = Node(
        package='rqt_plot',
        executable='rqt_plot',
        # FIXME: Why isn't the topic being populated on the UI? RQt issue?
        arguments=['--force-discover',
                   '/model/vehicle_blue/battery/linear_battery/state/percentage'],
        condition=IfCondition(LaunchConfiguration('rqt'))
    )

    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={
            'ign_args': '-r -z 1000000 linear_battery_demo.sdf'
        }.items(),
    )

    # Bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                   '/model/vehicle_blue/battery/linear_battery/state@sensor_msgs/msg/BatteryState@ignition.msgs.BatteryState'],
        output='screen'
    )

    return LaunchDescription([
        ign_gazebo,
        DeclareLaunchArgument('rqt', default_value='true',
                              description='Open RQt.'),
        bridge,
        rqt
    ])
