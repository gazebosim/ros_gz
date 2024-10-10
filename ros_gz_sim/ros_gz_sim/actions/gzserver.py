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

"""Module for the GzServer action."""

from typing import List
from typing import Optional

from launch.action import Action
from launch.actions import IncludeLaunchDescription
from launch.frontend import Entity, expose_action, Parser
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


@expose_action('gz_server')
class GzServer(Action):
    """Action that executes a gz_server ROS [composable] node."""

    def __init__(
        self,
        *,
        world_sdf_file: Optional[SomeSubstitutionsType] = '',
        world_sdf_string: Optional[SomeSubstitutionsType] = '',
        container_name: Optional[SomeSubstitutionsType] = 'ros_gz_container',
        create_own_container: Optional[SomeSubstitutionsType] = 'False',
        use_composition: Optional[SomeSubstitutionsType] = 'False',
        **kwargs
    ) -> None:
        """
        Construct a gz_server action.

        All arguments are forwarded to `ros_gz_sim.launch.gz_server.launch.py`,
        so see the documentation of that class for further details.

        :param: world_sdf_file Path to the SDF world file.
        :param: world_sdf_string SDF world string.
        :param: container_name Name of container that nodes will load in if use composition.
        :param: create_own_container Whether to start a ROS container when using composition.
        :param: use_composition Use composed bringup if True.
        """
        super().__init__(**kwargs)
        self.__world_sdf_file = world_sdf_file
        self.__world_sdf_string = world_sdf_string
        self.__container_name = container_name
        self.__create_own_container = create_own_container
        self.__use_composition = use_composition

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Parse gz_server."""
        _, kwargs = super().parse(entity, parser)

        world_sdf_file = entity.get_attr(
            'world_sdf_file', data_type=str,
            optional=True)

        world_sdf_string = entity.get_attr(
            'world_sdf_string', data_type=str,
            optional=True)

        container_name = entity.get_attr(
            'container_name', data_type=str,
            optional=True)

        create_own_container = entity.get_attr(
            'create_own_container', data_type=str,
            optional=True)

        use_composition = entity.get_attr(
            'use_composition', data_type=str,
            optional=True)

        if isinstance(world_sdf_file, str):
            world_sdf_file = parser.parse_substitution(world_sdf_file)
            kwargs['world_sdf_file'] = world_sdf_file

        if isinstance(world_sdf_string, str):
            world_sdf_string = parser.parse_substitution(world_sdf_string)
            kwargs['world_sdf_string'] = world_sdf_string

        if isinstance(container_name, str):
            container_name = parser.parse_substitution(container_name)
            kwargs['container_name'] = container_name

        if isinstance(create_own_container, str):
            create_own_container = \
                parser.parse_substitution(create_own_container)
            kwargs['create_own_container'] = create_own_container

        if isinstance(use_composition, str):
            use_composition = parser.parse_substitution(use_composition)
            kwargs['use_composition'] = use_composition

        return cls, kwargs

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        """Execute the action."""
        gz_server_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_server.launch.py'])]),
            launch_arguments=[('world_sdf_file', self.__world_sdf_file),
                              ('world_sdf_string', self.__world_sdf_string),
                              ('container_name',   self.__container_name),
                              ('create_own_container', self.__create_own_container),
                              ('use_composition',  self.__use_composition), ])

        return [gz_server_description]
