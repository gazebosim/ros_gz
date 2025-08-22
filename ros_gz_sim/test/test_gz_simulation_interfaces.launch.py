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

from pathlib import Path
import re
import time
from typing import Any
import unittest

from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
from launch_testing.actions import ReadyToTest
from launch_testing.asserts import assertExitCodes
import rclpy
from simulation_interfaces.msg import Result
import simulation_interfaces.srv as si

# Match name used in launch files
GZ_SERVER_NODE_NAME = 'gz_server'


def generate_test_description():
    test_dir = Path(__file__).parent
    test_sdf_file = str(test_dir / 'sdf' / 'gz_simulation_interfaces.sdf')

    server_node = Node(
        package='ros_gz_sim',
        executable='gzserver',
        name=GZ_SERVER_NODE_NAME,
        output='screen',
        parameters=[{'world_sdf_file': test_sdf_file}],
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        emulate_tty=True,
        parameters=[
            {'bridge_names': ['cmd_vel']},
            {
                'bridges': {
                    'cmd_vel': {
                        'ros_topic_name': '/cmd_vel',
                        'gz_topic_name': '/model/vehicle/cmd_vel',
                        'ros_type_name': 'geometry_msgs/msg/Twist',
                        'gz_type_name': 'gz.msgs.Twist',
                        'direction': 'ROS_TO_GZ',
                    }
                }
            },
        ],
    )

    return (
        LaunchDescription(
            [
                server_node,
                bridge_node,
                ReadyToTest(),
            ]
        ),
        locals(),
    )


class TestGzSimulationInterfaces(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self) -> None:
        self.node = rclpy.create_node('test_interfaces')

    def tearDown(self) -> None:
        self.node.destroy_node()

    def setup_client(self, srv_type, srv_name):
        client = self.node.create_client(
            srv_type,
            f'{GZ_SERVER_NODE_NAME}/{srv_name}',
        )
        self.assertTrue(client.wait_for_service(timeout_sec=5))
        return client, srv_type.Request()

    def call_and_spin(self, client, request, timeout_sec=5) -> Any:
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        self.assertTrue(future.done())
        return future.result()

    def test_get_entities_with_no_filters(self) -> None:
        get_entities, request = self.setup_client(si.GetEntities, 'get_entities')
        response = self.call_and_spin(get_entities, request)
        self.assertIsNotNone(response)
        self.assertEqual(response.result.result, Result.RESULT_OK)
        self.assertEqual(response.result.error_message, '')

    def get_entity_state(self, entity_name) -> si.GetEntityState.Response:
        get_entity_state, request = self.setup_client(si.GetEntityState, 'get_entity_state')
        request.entity = entity_name
        response = self.call_and_spin(get_entity_state, request)
        self.assertIsNotNone(response)
        self.assertEqual(response.result.result, Result.RESULT_OK)
        self.assertEqual(response.result.error_message, '')
        return response

    def test_get_entity_state(self, proc_output, bridge_node) -> None:
        state = self.get_entity_state('vehicle').state
        self.assertAlmostEqual(state.twist.linear.x, 0, delta=1e-4)

        # Send a velocity command to vehicle
        publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)
        msg = Twist()
        msg.linear.x = 0.25
        pattern = re.compile('Passing message.*geometry_msgs/msg/Twist')
        for _ in range(10):
            publisher.publish(msg)
            if proc_output.waitFor(pattern, process=bridge_node, timeout=1):
                break
            rclpy.spin_once(self.node)

        time.sleep(2)
        state = self.get_entity_state('vehicle').state
        self.assertAlmostEqual(state.twist.linear.x, 0.25, delta=1e-2)

    def test_get_entity_state_on_spawned_entity(self) -> None:
        sdf_string = """
            <sdf version='1.12'>
                <model name="sphere">
                    <link name="sphere_link">
                        <inertial auto="true"><mass>1.0</mass></inertial>
                        <collision name="sphere_collision">
                            <geometry>
                                <sphere> <radius>1</radius> </sphere>
                            </geometry>
                        </collision>
                        <visual name="sphere_visual">
                            <geometry>
                                <sphere> <radius>1</radius> </sphere>
                            </geometry>
                        </visual>
                    </link>
                </model>
            </sdf>
            """
        spawn_entity, request = self.setup_client(si.SpawnEntity, 'spawn_entity')
        request.name = 'test_sphere'
        request.entity_resource.resource_string = sdf_string
        # spawn at 4, -20, 2 so that when the ball falls on the incline and start
        # rolling without any external commands
        request.initial_pose.pose.position.x = 4.0
        request.initial_pose.pose.position.y = -20.0
        request.initial_pose.pose.position.z = 4.0

        self.assertTrue(self.call_and_spin(spawn_entity, request))
        time.sleep(2)
        state = self.get_entity_state('test_sphere').state
        self.assertGreater(state.twist.linear.x, 0.1)


# NOTE: If we don't have this test, unittest will report "NO TESTS RAN" at the end of the test
# See https://github.com/colcon/colcon-core/issues/678
@launch_testing.post_shutdown_test()
class TestGzserverShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        assertExitCodes(proc_info)
