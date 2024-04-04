
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

"""Launch gzsim in a component container."""

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    world_sdf_file_param = LaunchConfiguration('world_sdf_file')

    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='ros_gz_sim',
                    plugin='ros_gz_sim::GzServer',
                    name='gzserver',
                    parameters=[{'world_sdf_file': world_sdf_file_param}],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='screen',
    )

    return launch.LaunchDescription([container,])
