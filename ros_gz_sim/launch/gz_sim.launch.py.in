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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import ExecuteProcess, Shutdown
from launch.substitutions import LaunchConfiguration

def launch_gz(context, *args, **kwargs):
    env = {'GZ_SIM_SYSTEM_PLUGIN_PATH':
           ':'.join([environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
                     environ.get('LD_LIBRARY_PATH', default='')]),
           'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':  # TODO(CH3): To support pre-garden. Deprecated.
                      ':'.join([environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                                environ.get('LD_LIBRARY_PATH', default='')])}

    gz_args = LaunchConfiguration('gz_args').perform(context)
    gz_version = LaunchConfiguration('gz_version').perform(context)
    ign_args = LaunchConfiguration('ign_args').perform(context)
    ign_version = LaunchConfiguration('ign_version').perform(context)
    debugger = LaunchConfiguration('debugger').perform(context)
    on_exit_shutdown = LaunchConfiguration('on_exit_shutdown').perform(context)

    if not len(gz_args) and len(ign_args):
        print("ign_args is deprecated, migrate to gz_args!")
        exec_args = ign_args
    else:
        exec_args = gz_args

    if len(ign_version) or (ign_version == '' and int(gz_version) < 7):
        exec = 'ruby $(which ign) gazebo'

        if len(ign_version):
            gz_version = ign_version
    else:
        exec = 'ruby $(which gz) sim'

    if debugger != 'false':
        debug_prefix = 'x-terminal-emulator -e gdb -ex run --args'
    else:
        debug_prefix = None

    if on_exit_shutdown:
        on_exit = Shutdown()
    else:
        on_exit = None

    return [ExecuteProcess(
            cmd=[exec, exec_args, '--force-version', gz_version],
            output='screen',
            additional_env=env,
            shell=True,
            prefix=debug_prefix,
            on_exit=on_exit
        )]


def generate_launch_description():
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
        DeclareLaunchArgument(
            'on_exit_shutdown', default_value='false',
            description='Shutdown on gz-sim exit'),
        OpaqueFunction(function = launch_gz),
    ])
