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
import sys
import unittest

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource

from launch_ros.actions import Node

import launch_testing

# add ros_gz_bridge to the Python path
ros_gz_bridge_root = os.path.join(os.path.dirname(__file__), '..', '..')
sys.path.insert(0, os.path.abspath(ros_gz_bridge_root))


def generate_test_description():

    publisher = Node(
        package='ros_gz_bridge',
        executable='test_gz_publisher',
        output='screen'
    )
    server = Node(
        package='ros_gz_bridge',
        executable='test_gz_server',
        output='screen'
    )
    process_under_test = Node(
        package='ros_gz_bridge',
        executable='test_launch_action_subscriber',
        output='screen'
    )

    include_cmd = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(os.path.dirname(__file__), 'test_launch_action_classic.launch')))

    return LaunchDescription([
        DeclareLaunchArgument(
            'gz_msgs_ver',
            default_value=['0.0.0'],
            description='Gazebo messages version to test against'
        ),
        publisher,
        server,
        process_under_test,
        include_cmd,
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest(),
    ]), locals()


class LaunchActionTest(unittest.TestCase):

    def test_termination(self, process_under_test, proc_info):
        proc_info.assertWaitForShutdown(process=process_under_test, timeout=200)


@launch_testing.post_shutdown_test()
class LaunchActionTestAfterShutdown(unittest.TestCase):

    def test_exit_code(self, process_under_test, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            [launch_testing.asserts.EXIT_OK],
            process_under_test
        )
