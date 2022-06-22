# Copyright 2022 Open Source Robotics Foundation, Inc.
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

from launch import LaunchDescription

from launch_ros.actions import Node

import launch_testing


def generate_test_description():

    server = Node(
        package='ros_gz_bridge',
        executable='test_gz_server',
        output='screen'
    )
    process_under_test = Node(
        package='ros_gz_bridge',
        executable='test_ros_client',
        output='screen'
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
          '/gz_ros/test/serviceclient/world_control@ros_gz_interfaces/srv/ControlWorld',
        ],
        output='screen'
    )
    return LaunchDescription([
        bridge,
        server,
        process_under_test,
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest(),
    ]), locals()


class ROSSubscriberTest(unittest.TestCase):

    def test_termination(self, process_under_test, proc_info):
        proc_info.assertWaitForShutdown(process=process_under_test, timeout=200)


@launch_testing.post_shutdown_test()
class ROSSubscriberTestAfterShutdown(unittest.TestCase):

    def test_exit_code(self, process_under_test, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            [launch_testing.asserts.EXIT_OK],
            process_under_test
        )
