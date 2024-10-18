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

"""Launch gz_server in a component container."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    declare_world_sdf_file_cmd = DeclareLaunchArgument(
        'world_sdf_file', default_value=TextSubstitution(text=''),
        description='Path to the SDF world file')
    declare_world_sdf_string_cmd = DeclareLaunchArgument(
        'world_sdf_string', default_value=TextSubstitution(text=''),
        description='SDF world string')
    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='ros_gz_container',
        description='Name of container that nodes will load in if use composition',)
    declare_create_own_container_cmd = DeclareLaunchArgument(
        'create_own_container', default_value='False',
        description='Whether we should start our own ROS container when using composition.',)
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup if True')

    load_nodes = Node(
        condition=IfCondition(PythonExpression(['not ', LaunchConfiguration('use_composition')])),
        package='ros_gz_sim',
        executable='gzserver',
        output='screen',
        parameters=[{'world_sdf_file': LaunchConfiguration('world_sdf_file'),
                     'world_sdf_string': LaunchConfiguration('world_sdf_string')}],
    )

    load_composable_nodes_with_container = ComposableNodeContainer(
        condition=IfCondition(
            PythonExpression([LaunchConfiguration('use_composition'), ' and ',
                              LaunchConfiguration('create_own_container')])),
        name=LaunchConfiguration('container_name'),
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='ros_gz_sim',
                plugin='ros_gz_sim::GzServer',
                name='gz_server',
                parameters=[{'world_sdf_file': LaunchConfiguration('world_sdf_file'),
                             'world_sdf_string': LaunchConfiguration('world_sdf_string')}],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    load_composable_nodes_without_container = LoadComposableNodes(
        condition=IfCondition(
            PythonExpression([LaunchConfiguration('use_composition'), ' and not ',
                              LaunchConfiguration('create_own_container')])),
        target_container=LaunchConfiguration('container_name'),
        composable_node_descriptions=[
            ComposableNode(
                package='ros_gz_sim',
                plugin='ros_gz_sim::GzServer',
                name='gz_server',
                parameters=[{'world_sdf_file': LaunchConfiguration('world_sdf_file'),
                             'world_sdf_string': LaunchConfiguration('world_sdf_string')}],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_world_sdf_file_cmd)
    ld.add_action(declare_world_sdf_string_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_create_own_container_cmd)
    ld.add_action(declare_use_composition_cmd)
    # Add the actions to launch all of the gz_server nodes
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes_with_container)
    ld.add_action(load_composable_nodes_without_container)

    return ld
