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

"""Module for the ros_gz bridge action."""

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


@expose_action('ros_gz_bridge')
class RosGzBridge(Action):
    """Action that executes a ros_gz bridge ROS [composable] node."""

    def __init__(
        self,
        *,
        bridge_name: SomeSubstitutionsType,
        config_file: SomeSubstitutionsType,
        container_name: Optional[SomeSubstitutionsType] = 'ros_gz_container',
        create_own_container: Optional[SomeSubstitutionsType] = 'False',
        namespace: Optional[SomeSubstitutionsType] = '',
        use_composition: Optional[SomeSubstitutionsType] = 'False',
        use_respawn: Optional[SomeSubstitutionsType] = 'False',
        log_level: Optional[SomeSubstitutionsType] = 'info',
        **kwargs
    ) -> None:
        """
        Construct a ros_gz bridge action.

        All arguments are forwarded to `ros_gz_bridge.launch.ros_gz_bridge.launch.py`,
        so see the documentation of that class for further details.

        :param: bridge_name Name of ros_gz_bridge  node
        :param: config_file YAML config file.
        :param: container_name Name of container that nodes will load in if use composition.
        :param: create_own_container Whether to start a ROS container when using composition.
        :param: namespace Top-level namespace.
        :param: use_composition Use composed bringup if True.
        :param: use_respawn Whether to respawn if a node crashes (when composition is disabled).
        :param: log_level Log level.
        """
        super().__init__(**kwargs)
        self.__bridge_name = bridge_name
        self.__config_file = config_file
        self.__container_name = container_name
        self.__create_own_container = create_own_container
        self.__namespace = namespace
        self.__use_composition = use_composition
        self.__use_respawn = use_respawn
        self.__log_level = log_level

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Parse ros_gz_bridge."""
        _, kwargs = super().parse(entity, parser)

        bridge_name = entity.get_attr(
            'bridge_name', data_type=str,
            optional=False)

        config_file = entity.get_attr(
            'config_file', data_type=str,
            optional=False)

        container_name = entity.get_attr(
            'container_name', data_type=str,
            optional=True)

        create_own_container = entity.get_attr(
            'create_own_container', data_type=str,
            optional=True)

        namespace = entity.get_attr(
            'namespace', data_type=str,
            optional=True)

        use_composition = entity.get_attr(
            'use_composition', data_type=str,
            optional=True)

        use_respawn = entity.get_attr(
            'use_respawn', data_type=str,
            optional=True)

        log_level = entity.get_attr(
            'log_level', data_type=str,
            optional=True)

        if isinstance(bridge_name, str):
            bridge_name = parser.parse_substitution(bridge_name)
            kwargs['bridge_name'] = bridge_name

        if isinstance(config_file, str):
            config_file = parser.parse_substitution(config_file)
            kwargs['config_file'] = config_file

        if isinstance(container_name, str):
            container_name = parser.parse_substitution(container_name)
            kwargs['container_name'] = container_name

        if isinstance(create_own_container, str):
            create_own_container = \
                parser.parse_substitution(create_own_container)
            kwargs['create_own_container'] = create_own_container

        if isinstance(namespace, str):
            namespace = parser.parse_substitution(namespace)
            kwargs['namespace'] = namespace

        if isinstance(use_composition, str):
            use_composition = parser.parse_substitution(use_composition)
            kwargs['use_composition'] = use_composition

        if isinstance(use_respawn, str):
            use_respawn = parser.parse_substitution(use_respawn)
            kwargs['use_respawn'] = use_respawn

        if isinstance(log_level, str):
            log_level = parser.parse_substitution(log_level)
            kwargs['log_level'] = log_level

        return cls, kwargs

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        """Execute the action."""
        ros_gz_bridge_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_bridge'),
                                       'launch',
                                       'ros_gz_bridge.launch.py'])]),
            launch_arguments=[('bridge_name', self.__bridge_name),
                              ('config_file', self.__config_file),
                              ('container_name', self.__container_name),
                              ('create_own_container', self.__create_own_container),
                              ('namespace',   self.__namespace),
                              ('use_composition',  self.__use_composition),
                              ('use_respawn',  self.__use_respawn),
                              ('log_level',   self.__log_level), ])

        return [ros_gz_bridge_description]
