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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import Shutdown
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'ignition_server_args', default_value='',
            description='Extra arguments to be passed to Ignition'
        ),
        ExecuteProcess(
            cmd=[['ign', ' ', 'gazebo', ' ', '-s', ' ',
                  LaunchConfiguration('ignition_server_args')]],
            output='screen',
            on_exit=Shutdown(),
            shell=True,
        ),
    ])
