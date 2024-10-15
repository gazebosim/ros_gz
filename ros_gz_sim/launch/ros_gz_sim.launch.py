# Copyright 2024 Open Source Robotics Foundation, Inc.
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

"""Launch gzsim + ros_gz_bridge in a component container."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    bridge_name = LaunchConfiguration('bridge_name')
    config_file = LaunchConfiguration('config_file')
    container_name = LaunchConfiguration('container_name')
    create_own_container = LaunchConfiguration('create_own_container')
    namespace = LaunchConfiguration('namespace')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    bridge_log_level = LaunchConfiguration('bridge_log_level')

    world_sdf_file = LaunchConfiguration('world_sdf_file')
    world_sdf_string = LaunchConfiguration('world_sdf_string')

    declare_bridge_name_cmd = DeclareLaunchArgument(
        'bridge_name', description='Name of the bridge'
    )

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', description='YAML config file'
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='ros_gz_container',
        description='Name of container that nodes will load in if use composition',
    )

    declare_create_own_container_cmd = DeclareLaunchArgument(
        'create_own_container',
        default_value='False',
        description='Whether we should start a ROS container when using composition.',
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False', description='Use composed bringup if True'
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_bridge_log_level_cmd = DeclareLaunchArgument(
        'bridge_log_level', default_value='info', description='Bridge log level'
    )

    declare_world_sdf_file_cmd = DeclareLaunchArgument(
        'world_sdf_file', default_value=TextSubstitution(text=''),
        description='Path to the SDF world file'
    )

    declare_world_sdf_string_cmd = DeclareLaunchArgument(
        'world_sdf_string', default_value=TextSubstitution(text=''),
        description='SDF world string'
    )

    bridge_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_bridge'),
                                   'launch',
                                   'ros_gz_bridge.launch.py'])]),
        launch_arguments=[('bridge_name', bridge_name),
                          ('config_file', config_file),
                          ('container_name', container_name),
                          ('namespace', namespace),
                          ('use_composition', use_composition),
                          ('use_respawn', use_respawn),
                          ('bridge_log_level', bridge_log_level)])

    gz_server_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                   'launch',
                                   'gz_server.launch.py'])]),
        launch_arguments=[('world_sdf_file', world_sdf_file),
                          ('world_sdf_string', world_sdf_string),
                          ('container_name', container_name),
                          ('create_own_container', create_own_container),
                          ('use_composition', use_composition), ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_bridge_name_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_create_own_container_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_bridge_log_level_cmd)
    ld.add_action(declare_world_sdf_file_cmd)
    ld.add_action(declare_world_sdf_string_cmd)
    # Add the actions to launch all of the bridge + gz_server nodes
    ld.add_action(bridge_description)
    ld.add_action(gz_server_description)

    return ld
