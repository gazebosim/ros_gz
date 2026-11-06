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

"""Launch create to spawn models in gz sim."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    world = LaunchConfiguration('world')
    file = LaunchConfiguration('file')
    model_string = LaunchConfiguration('model_string')
    topic = LaunchConfiguration('topic')
    entity_name = LaunchConfiguration('entity_name')
    entity_namespace = LaunchConfiguration('entity_namespace')
    allow_renaming = LaunchConfiguration('allow_renaming')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    declare_world_cmd = DeclareLaunchArgument(
        'world', default_value=TextSubstitution(text=''),
        description='World name')
    declare_file_cmd = DeclareLaunchArgument(
        'file', default_value=TextSubstitution(text=''),
        description='SDF filename')
    declare_model_string_cmd = DeclareLaunchArgument(
        'model_string',
        default_value='',
        description='XML(SDF) string',
    )
    declare_topic_cmd = DeclareLaunchArgument(
        'topic', default_value=TextSubstitution(text=''),
        description='Get XML from this topic'
    )
    declare_entity_name_cmd = DeclareLaunchArgument(
        'entity_name', default_value=TextSubstitution(text=''),
        description='Name of the entity'
    )
    declare_entity_namespace_cmd = DeclareLaunchArgument(
        'entity_namespace', default_value=TextSubstitution(text=''),
        description='Namespace for the spawned entity.'
    )
    declare_allow_renaming_cmd = DeclareLaunchArgument(
        'allow_renaming', default_value='False',
        description='Whether the entity allows renaming or not'
    )
    declare_x_cmd = DeclareLaunchArgument(
        'x', default_value='0.0',
        description='X position of the entity')
    declare_y_cmd = DeclareLaunchArgument(
        'y', default_value='0.0',
        description='Y position of the entity')
    declare_z_cmd = DeclareLaunchArgument(
        'z', default_value='0.0',
        description='Z position of the entity')
    # 'R', 'P' and 'Y' are the deprecated names, kept for backwards compatibility.
    declare_roll_cmd = DeclareLaunchArgument(
        'roll', default_value=LaunchConfiguration('R', default='0.0'),
        description='Roll orientation of the entity')
    declare_pitch_cmd = DeclareLaunchArgument(
        'pitch', default_value=LaunchConfiguration('P', default='0.0'),
        description='Pitch orientation of the entity')
    declare_yaw_cmd = DeclareLaunchArgument(
        'yaw', default_value=LaunchConfiguration('Y', default='0.0'),
        description='Yaw orientation of the entity')

    load_nodes = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[{'world': world,
                     'file': file,
                     'string': model_string,
                     'topic': topic,
                     'name': entity_name,
                     'entity_namespace': entity_namespace,
                     'allow_renaming': allow_renaming,
                     'x': x,
                     'y': y,
                     'z': z,
                     'R': roll,
                     'P': pitch,
                     'Y': yaw,
                     }],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_file_cmd)
    ld.add_action(declare_model_string_cmd)
    ld.add_action(declare_topic_cmd)
    ld.add_action(declare_entity_name_cmd)
    ld.add_action(declare_entity_namespace_cmd)
    ld.add_action(declare_allow_renaming_cmd)
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)
    # Add the actions to launch all of the create nodes
    ld.add_action(load_nodes)

    return ld
