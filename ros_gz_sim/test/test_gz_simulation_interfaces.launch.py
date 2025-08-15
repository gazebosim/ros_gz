# Copyright 2025 Open Source Robotics Foundation, Inc.
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

import unittest

import launch_testing
import rclpy
import simulation_interfaces.srv as si
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_test_description():

    test_dir = Path(__file__).parent
    test_sdf_file = str(test_dir / "sdf" / "gz_simulation_interfaces.sdf")

    return LaunchDescription(
        [
            Node(
                package="ros_gz_sim",
                executable="gzserver",
                parameters=[{"world_sdf_file": test_sdf_file}],
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestGzSimulationInterfaces(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self) -> None:
        self.node = rclpy.create_node("test_interfaces")

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_get_entities(self, proc_output) -> None:
        client = self.node.create_client(si.GetEntities, "get_entities")
        self.assertTrue(client.wait_for_service(timeout_sec=5))

    def test_get_entity_state(self, proc_output) -> None:
        client = self.node.create_client(si.GetEntityState, "get_entity_state")
        self.assertTrue(client.wait_for_service(timeout_sec=5))


# NOTE: If we don't have this test, unittest will report "NO TESTS RAN" at the end of the test
# See https://github.com/colcon/colcon-core/issues/678
@launch_testing.post_shutdown_test()
class TestGzserverShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
