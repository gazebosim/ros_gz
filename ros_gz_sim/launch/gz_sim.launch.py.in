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

"""Launch Gazebo Sim with command line arguments."""

from os import environ

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, AndSubstitution, PythonExpression
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals, IfCondition


def generate_launch_description():
    env = {'GZ_SIM_SYSTEM_PLUGIN_PATH':
           ':'.join([environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
                     environ.get('LD_LIBRARY_PATH', default='')]),
           'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':  # TODO(CH3): To support pre-garden. Deprecated.
                      ':'.join([environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                                environ.get('LD_LIBRARY_PATH', default='')])}


    return LaunchDescription([
        DeclareLaunchArgument('gz_args', default_value='',
                              description='Arguments to be passed to Gazebo Sim'),
        # Gazebo Sim's major version
        DeclareLaunchArgument('gz_version', default_value='@GZ_SIM_VER@',
                              description='Gazebo Sim\'s major version'),

        # TODO(CH3): Deprecated. Remove on tock.
        DeclareLaunchArgument(
            'ign_args', default_value='',
            description='Deprecated: Arguments to be passed to Gazebo Sim'
        ),
        # Gazebo Sim's major version
        DeclareLaunchArgument(
            'ign_version', default_value='',
            description='Deprecated: Gazebo Sim\'s major version'
        ),
        DeclareLaunchArgument(
            'debugger', default_value='false',
            description='Run in Debugger'),

        ExecuteProcess(
            condition=IfCondition(
                AndSubstitution(
                    PythonExpression(["'", LaunchConfiguration('ign_version'), "' == ''"]),
                    PythonExpression(["'", LaunchConfiguration('gz_version'), "' < '7'"])
                )
            ),
            cmd=['ruby $(which ign) gazebo',
                 LaunchConfiguration('gz_args'),
                 '--force-version',
                 LaunchConfiguration('gz_version'),
                 ],
            output='screen',
            additional_env=env,
            shell=True,
            prefix=PythonExpression([
                "'x-terminal-emulator -e gdb -ex run --args' if '", LaunchConfiguration('debugger'), "'=='true' else ''"])
        ),

        ExecuteProcess(
            condition=IfCondition(
                AndSubstitution(
                    PythonExpression(["'", LaunchConfiguration('ign_version'), "' == ''"]),
                    PythonExpression(["'", LaunchConfiguration('gz_version'), "' >= '7'"])
                )
            ),
            cmd=['ruby $(which gz) sim',
                 LaunchConfiguration('gz_args'),
                 '--force-version',
                 LaunchConfiguration('gz_version'),
                 ],
            output='screen',
            additional_env=env,
            shell=True,
            prefix=PythonExpression([
                "'x-terminal-emulator -e gdb -ex run --args' if '", LaunchConfiguration('debugger'), "'=='true' else ''"])
        ),
    ])
