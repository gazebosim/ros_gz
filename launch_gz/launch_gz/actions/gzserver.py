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
from launch.frontend import Entity, expose_action, Parser
from launch.launch_context import LaunchContext
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.some_substitutions_type import SomeSubstitutionsType

@expose_action('gzserver')
class GzServer(ComposableNodeContainer):
    """Action that executes a container ROS node for composable ROS nodes."""

    def __init__(
        self,
        *,
        name: SomeSubstitutionsType,
        namespace: SomeSubstitutionsType,
        composable_node_descriptions: Optional[List[ComposableNode]] = None,
        **kwargs
    ) -> None:
        """
        Construct a ComposableNodeContainer action.

        Most arguments are forwarded to :class:`launch_ros.actions.Node`, so see the documentation
        of that class for further details.

        :param: name the name of the node, mandatory for full container node name resolution
        :param: namespace the ROS namespace for this Node, mandatory for full container node
             name resolution
        :param composable_node_descriptions: optional descriptions of composable nodes to be loaded
        """
        super().__init__(name=name, namespace=namespace,
                         composable_node_descriptions=composable_node_descriptions,
                         **kwargs)

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Parse node_container."""
        _, kwargs = super().parse(entity, parser)

        return cls, kwargs

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        """
        Execute the action.

        Most work is delegated to :meth:`launch_ros.actions.Node.execute`, except for the
        composable nodes load action if it applies.
        """

        return super().execute(context)
