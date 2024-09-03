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

"""Launch ros_gz bridge in a component container."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    name = LaunchConfiguration('name')
    config_file = LaunchConfiguration('config_file')
    container_name = LaunchConfiguration('container_name')
    namespace = LaunchConfiguration('namespace')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    declare_name_cmd = DeclareLaunchArgument(
        'name', description='Name of ros_gz_bridge node'
    )

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='', description='YAML config file'
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='ros_gz_container',
        description='Name of container that nodes will load in if use composition',
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True', description='Use composed bringup if True'
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='bridge_node',
                name=name,
                namespace=namespace,
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[{'config_file': config_file}],
                arguments=['--ros-args', '--log-level', log_level],
            ),
        ],
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name,
        composable_node_descriptions=[
            ComposableNode(
                package='ros_gz_bridge',
                plugin='ros_gz_bridge::RosGzBridge',
                name=name,
                namespace=namespace,
                parameters=[{'config_file': config_file}],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_name_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    # Add the actions to launch all of the bridge nodes
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
