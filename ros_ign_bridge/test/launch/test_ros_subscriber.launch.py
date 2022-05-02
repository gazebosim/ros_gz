# Copyright 2020 Open Source Robotics Foundation, Inc.
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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import launch_testing

# add ros_ign_bridge to the Python path
ros_ign_bridge_root = os.path.join(os.path.dirname(__file__), '..', '..')
sys.path.insert(0, os.path.abspath(ros_ign_bridge_root))

from ros_ign_bridge import mappings  # noqa: E402


def bridge_setup(context, *args, **kwargs):
    gz_msgs_ver = LaunchConfiguration('gz_msgs_ver').perform(context)
    gz_msgs_ver = tuple(map(int, gz_msgs_ver.split('.')))

    # Bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
          f'/{m.unique()}@{m.ros2_string()}[{m.ign_string()}'
          for m in mappings(gz_msgs_ver)
        ],
        output='screen'
    )
    return [bridge]


def generate_test_description():

    publisher = Node(
        package='ros_ign_bridge',
        executable='test_ign_publisher',
        output='screen'
    )
    process_under_test = Node(
        package='ros_ign_bridge',
        executable='test_ros_subscriber',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gz_msgs_ver',
            default_value=['0.0.0'],
            description='Gazebo messages version to test against'
        ),
        publisher,
        process_under_test,
        OpaqueFunction(function=bridge_setup),
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
