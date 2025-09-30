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

import os
from pathlib import Path
import re
import time
from typing import Any
import unittest

from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
import launch_testing
from launch_testing.actions import ReadyToTest
from launch_testing.asserts import assertExitCodes
import rclpy
import rclpy.action
from simulation_interfaces.action import SimulateSteps
from simulation_interfaces.msg import EntityCategory, Result, SimulationState, SimulatorFeatures
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
                # Keep gz-transport contained to localhost to make the test more deterministic
                SetEnvironmentVariable(name='GZ_IP', value='127.0.0.1'),
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
        expected_gz_ip = '127.0.0.1'
        gz_ip = os.environ['GZ_IP']
        if gz_ip != expected_gz_ip:
            raise RuntimeError(f'GZ_IP is expected to be {expected_gz_ip}, but is set to {gz_ip}')
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self) -> None:
        self.node = rclpy.create_node('test_interfaces')

    def tearDown(self) -> None:
        self.node.destroy_node()

    # helpers

    def setup_client(self, srv_type, srv_name):
        client = self.node.create_client(
            srv_type,
            f'{GZ_SERVER_NODE_NAME}/{srv_name}')
        self.assertTrue(client.wait_for_service(timeout_sec=5))
        return client, srv_type.Request()

    def call_and_spin(self, client, request, timeout_sec=5) -> Any:
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=timeout_sec)
        self.assertTrue(future.done())
        return future.result()

    def assert_result_ok(self, response) -> None:
        self.assertIsNotNone(response)
        self.assertEqual(
            response.result.result,
            Result.RESULT_OK,
            msg=response.result.error_message)
        self.assertEqual(response.result.error_message, '')

    def get_simulation_state(self) -> si.GetSimulationState.Response:
        get_simulation_state, request = self.setup_client(
            si.GetSimulationState, 'get_simulation_state')
        response = self.call_and_spin(get_simulation_state, request)
        self.assert_result_ok(response)
        return response

    def set_simulation_state(self, simulation_state) -> None:
        set_simulation_state, request = self.setup_client(
            si.SetSimulationState, 'set_simulation_state')
        request.state.state = simulation_state
        response = self.call_and_spin(set_simulation_state, request)
        self.assert_result_ok(response)

    def delete_entity(self, entity_name) -> None:
        delete_entity, request = self.setup_client(
            si.DeleteEntity, 'delete_entity')
        request.entity = entity_name
        response = self.call_and_spin(delete_entity, request)
        self.assert_result_ok(response)

    def get_entity_state(self, entity_name) -> si.GetEntityState.Response:
        get_entity_state, request = self.setup_client(
            si.GetEntityState, 'get_entity_state')
        request.entity = entity_name
        response = self.call_and_spin(get_entity_state, request)
        self.assert_result_ok(response)
        return response

    def reset_simulation(self) -> si.ResetSimulation.Response:
        reset_simulation, request = self.setup_client(
            si.ResetSimulation, 'reset_simulation')
        request.scope = si.ResetSimulation.Request.SCOPE_DEFAULT
        return self.call_and_spin(reset_simulation, request)

    # tests

    def test_get_entities_with_no_filters(self) -> None:
        get_entities, request = self.setup_client(
            si.GetEntities, 'get_entities')
        response = self.call_and_spin(get_entities, request)
        self.assert_result_ok(response)

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
        self.assertAlmostEqual(state.twist.linear.x, 0.25, delta=1e-1)

    def test_get_entity_state_on_spawned_entity(self) -> None:
        sdf_string = """
            <sdf version='1.12'>
                <model name="sphere2">
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
        spawn_entity, request = self.setup_client(
            si.SpawnEntity, 'spawn_entity')
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

    def test_set_entity_state(self) -> None:
        self.assert_result_ok(self.reset_simulation())
        set_entity_state, request = self.setup_client(
            si.SetEntityState, 'set_entity_state')
        self.set_simulation_state(SimulationState.STATE_PLAYING)
        test_entity = 'sphere'
        request.entity = test_entity
        request.state.pose.position.z = 100.0
        request.state.twist.linear.x = 5.0
        self.assert_result_ok(self.call_and_spin(set_entity_state, request))
        state = self.get_entity_state(test_entity).state
        self.assertAlmostEqual(state.twist.linear.x, 5.0, delta=1e-1)

    def test_set_entity_state_preserves_sim_state(self) -> None:
        set_entity_state, request = self.setup_client(
            si.SetEntityState, 'set_entity_state')
        request.entity = 'sphere'
        request.state.pose.position.z = 10.0
        for test_state in [SimulationState.STATE_PLAYING, SimulationState.STATE_PAUSED]:
            self.set_simulation_state(test_state)
            self.assertEqual(self.get_simulation_state().state.state, test_state)
            self.assert_result_ok(self.call_and_spin(set_entity_state, request))
            self.assertEqual(self.get_simulation_state().state.state, test_state)

    def test_spawn_entity_duplicate_name(self) -> None:
        sdf_string = """
            <sdf version='1.12'>
                <model name="test_empty">
                    <link name="link">
                    </link>
                </model>
            </sdf>
        """
        spawn_entity, request = self.setup_client(
            si.SpawnEntity, 'spawn_entity')
        request.name = 'test_duplicate'
        request.entity_resource.resource_string = sdf_string

        self.assert_result_ok(self.call_and_spin(spawn_entity, request))
        time.sleep(2)

        # Try to spawn the same entity again
        result = self.call_and_spin(spawn_entity, request).result.result
        self.assertTrue(result, Result.RESULT_OPERATION_FAILED)

    def test_delete_entity(self) -> None:
        self.delete_entity('box')

    def test_delete_on_spawned_entity(self) -> None:
        sdf_string = """
            <sdf version='1.12'>
                <model name="test_empty">
                    <link name="link">
                    </link>
                </model>
            </sdf>
        """
        spawn_entity, request = self.setup_client(
            si.SpawnEntity, 'spawn_entity')
        request.name = 'test_empty'
        request.entity_resource.resource_string = sdf_string

        self.assert_result_ok(self.call_and_spin(spawn_entity, request))
        time.sleep(2)

        self.delete_entity('test_empty')

    def test_step_simulation(self) -> None:
        step_simulation, request = self.setup_client(
            si.StepSimulation, 'step_simulation')
        request.steps = 5
        response = self.call_and_spin(step_simulation, request)
        self.assert_result_ok(response)

    def test_reset_simulation(self) -> None:
        self.assert_result_ok(self.reset_simulation())

    def test_toggle_simulation_state(self) -> None:
        initial_state = self.get_simulation_state().state.state
        if initial_state == SimulationState.STATE_PAUSED:
            target_state = SimulationState.STATE_PLAYING
        else:
            target_state = SimulationState.STATE_PAUSED

        self.set_simulation_state(target_state)

        final_state = self.get_simulation_state().state.state
        self.assertEqual(final_state, target_state)

        # Return the simulation to its initial state
        self.set_simulation_state(initial_state)
        restored_state = self.get_simulation_state().state.state
        self.assertEqual(restored_state, initial_state)

    def test_playing_when_already_playing(self) -> None:
        # Try to set it to the same state twice
        self.set_simulation_state(SimulationState.STATE_PLAYING)
        self.set_simulation_state(SimulationState.STATE_PLAYING)

    def test_simulate_steps_action(self) -> None:

        goal_msg = SimulateSteps.Goal()
        goal_msg.steps = 200

        self.feedback_cb_count = 0

        def feedback_cb(feedback_msg) -> None:
            self.feedback_cb_count += 1
            feedback = feedback_msg.feedback
            self.assertGreater(feedback.completed_steps, 0)

        simulation_state = self.get_simulation_state().state.state
        if simulation_state != SimulationState.STATE_PAUSED:
            self.set_simulation_state(SimulationState.STATE_PAUSED)

        # Create a new node for testing the action.
        # A warning is printed when the existing node is used.
        test_node = rclpy.create_node('test_action')
        action_client = rclpy.action.ActionClient(
            test_node,
            SimulateSteps,
            f'{GZ_SERVER_NODE_NAME}/simulate_steps')
        self.assertTrue(action_client.wait_for_server(timeout_sec=30))

        # TODO(azeey): Need to wait on the result future here so the test is ran to completion.
        def send_goal_and_check_results():
            self.feedback_cb_count = 0
            send_goal_future = action_client.send_goal_async(
                goal_msg, feedback_callback=feedback_cb)
            rclpy.spin_until_future_complete(test_node, send_goal_future)
            goal_handle = send_goal_future.result()
            assert goal_handle is not None
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(test_node, result_future)
            result = result_future.result()
            assert result is not None
            self.assert_result_ok(result.result)

        send_goal_and_check_results()
        # Assert that the feedback callback is called at least once.
        self.assertGreater(self.feedback_cb_count, 1)
        # Do it again to make sure subsequent actions are handled properly
        send_goal_and_check_results()
        self.assertGreater(self.feedback_cb_count, 1)

    def test_get_entity_info(self) -> None:
        get_entity_info, request = self.setup_client(
            si.GetEntityInfo, 'get_entity_info')
        request.entity = 'vehicle'

        response = self.call_and_spin(get_entity_info, request)
        self.assert_result_ok(response)

        self.assertEqual(response.info.category.category,
                         EntityCategory.CATEGORY_ROBOT)

    def test_get_entities_states(self) -> None:
        get_entities_state, request = self.setup_client(
            si.GetEntitiesStates, 'get_entities_states')
        request.filters.filter = 'vehicle'

        response = self.call_and_spin(get_entities_state, request)
        self.assert_result_ok(response)

        self.assertEqual(len(response.entities), 1)
        self.assertEqual(response.entities[0], 'vehicle')

    def test_get_simulator_features(self) -> None:
        get_simulator_features, request = self.setup_client(
            si.GetSimulatorFeatures, 'get_simulator_features')

        response = self.call_and_spin(get_simulator_features, request)
        returned_features = response.features.features

        existing_features = [
            SimulatorFeatures.SPAWNING,
            SimulatorFeatures.DELETING,
            SimulatorFeatures.ENTITY_TAGS,
            # SimulatorFeatures.ENTITY_BOUNDS,
            # SimulatorFeatures.ENTITY_BOUNDS_BOX,
            SimulatorFeatures.ENTITY_CATEGORIES,
            SimulatorFeatures.SPAWNING_RESOURCE_STRING,
            SimulatorFeatures.ENTITY_STATE_GETTING,
            SimulatorFeatures.ENTITY_STATE_SETTING,
            SimulatorFeatures.ENTITY_INFO_GETTING,
            # SimulatorFeatures.ENTITY_INFO_SETTING,
            SimulatorFeatures.SIMULATION_RESET,
            # SimulatorFeatures.SIMULATION_RESET_TIME,
            # SimulatorFeatures.SIMULATION_RESET_STATE,
            # SimulatorFeatures.SIMULATION_RESET_SPAWNED,
            SimulatorFeatures.SIMULATION_STATE_GETTING,
            SimulatorFeatures.SIMULATION_STATE_SETTING,
            SimulatorFeatures.SIMULATION_STATE_PAUSE,
            SimulatorFeatures.STEP_SIMULATION_SINGLE,
            SimulatorFeatures.STEP_SIMULATION_MULTIPLE,
            SimulatorFeatures.STEP_SIMULATION_ACTION
        ]

        # Check if each feature exists in the response
        for feature in existing_features:
            self.assertIn(feature, returned_features)

        # Check sizes of both feature lists
        self.assertEqual(len(returned_features), len(existing_features))


# NOTE: If we don't have this test, unittest will report "NO TESTS RAN" at the end of the test
# See https://github.com/colcon/colcon-core/issues/678
@launch_testing.post_shutdown_test()
class TestGzserverShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        assertExitCodes(proc_info)
