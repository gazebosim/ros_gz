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

from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import Shutdown
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    ros_ign_gazebo = get_package_prefix('ros_ign_gazebo')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ignition_gui_args', default_value='',
            description='Extra arguments to be passed to Ignition'
        ),
        ExecuteProcess(
            cmd=[[ros_ign_gazebo + '/lib/ros_ign_gazebo/ign_gazebo', ' ', '-g', ' ',
                  LaunchConfiguration('ignition_gui_args')]],
            output='screen',
            on_exit=Shutdown(),
            shell=True,
        ),
    ])
