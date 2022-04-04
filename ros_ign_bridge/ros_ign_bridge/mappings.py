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

from collections import namedtuple
from dataclasses import dataclass

Mapping = namedtuple('Mapping', ('ros_type', 'ign_type'))

# List of known mappings
#
# The pattern for adding a new mapping
#
#   'ros2_package_name': [
#       Mapping('ros2_message_name', 'ignition_message_name'),
#   ],
MAPPINGS = {
    'builtin_interfaces': [
        Mapping('Time', 'Time'),
    ],
    'geometry_msgs': [
        Mapping('Quaternion', 'Quaternion'),
        Mapping('Vector3', 'Vector3d'),
        Mapping('Point', 'Vector3d'),
        Mapping('Pose', 'Pose'),
        Mapping('PoseStamped', 'Pose'),
        Mapping('Transform', 'Pose'),
        Mapping('TransformStamped', 'Pose'),
        Mapping('Twist', 'Twist'),
        Mapping('Wrench', 'Wrench'),
    ],
    'nav_msgs': [
        Mapping('Odometry', 'Odometry'),
    ],
    'ros_ign_interfaces': [
        Mapping('JointWrench', 'JointWrench'),
        Mapping('Entity', 'Entity'),
        Mapping('Contact', 'Contact'),
        Mapping('Contacts', 'Contacts'),
        Mapping('GuiCamera', 'GUICamera'),
        Mapping('Light', 'Light'),
        Mapping('StringVec', 'StringMsg_V'),
        Mapping('TrackVisual', 'TrackVisual'),
        Mapping('VideoRecord', 'VideoRecord'),
        Mapping('WorldControl', 'WorldControl'),
    ],
    'rosgraph_msgs': [
        Mapping('Clock', 'Clock'),
    ],
    'sensor_msgs': [
        Mapping('FluidPressure', 'FluidPressure'),
        Mapping('Image', 'Image'),
        Mapping('CameraInfo', 'CameraInfo'),
        Mapping('Imu', 'IMU'),
        Mapping('JointState', 'Model'),
        Mapping('LaserScan', 'LaserScan'),
        Mapping('MagneticField', 'Magnetometer'),
        Mapping('PointCloud2', 'PointCloudPacked'),
        Mapping('BatteryState', 'BatteryState'),
    ],
    'std_msgs': [
        Mapping('Bool', 'Boolean'),
        Mapping('ColorRGBA', 'Color'),
        Mapping('Empty', 'Empty'),
        Mapping('Float32', 'Float'),
        Mapping('Float64', 'Double'),
        Mapping('Header', 'Header'),
        Mapping('Int32', 'Int32'),
        Mapping('UInt32', 'UInt32'),
        Mapping('String', 'StringMsg'),
    ],
    'tf2_msgs': [
        Mapping('TFMessage', 'Pose_V'),
    ],
    'trajectory_msgs': [
        Mapping('JointTrajectory', 'JointTrajectory'),
    ],
}


@dataclass
class MessageMapping:
    # Class to represent mapping between ROS2 and Ignition types
    ros2_package_name: str
    ros2_message_name: str
    ign_message_name: str

    def ros2_string(self):
        # Return ROS2 string version of a message (eg std_msgs/msg/Bool)
        return f'{self.ros2_package_name}/msg/{self.ros2_message_name}'

    def ros2_type(self):
        # Return ROS2 type of a message (eg std_msgs::msg::Bool)
        return f'{self.ros2_package_name}::msg::{self.ros2_message_name}'

    def ign_string(self):
        # Return IGN string version of a message (eg ignition.msgs.Bool)
        return f'ignition.msgs.{self.ign_message_name}'

    def ign_type(self):
        # Return IGN type of a message (eg ignition::msgs::Bool)
        return f'ignition::msgs::{self.ign_message_name}'


def mappings():
    # Generate MessageMapping object for all known mappings
    data = []
    for (ros2_package_name, mappings) in MAPPINGS.items():
        for mapping in sorted(mappings):
            data.append(MessageMapping(
                ros2_package_name=ros2_package_name,
                ros2_message_name=mapping.ros_type,
                ign_message_name=mapping.ign_type
            ))
    return data


if __name__ == '__main__':
    # Print the markdown table used in the README.md
    rows = []
    rows.append(f'| {"ROS type":32}| {"Ignition Transport Type":32}|')
    rows.append(f'|{"-":-<33}|:{"-":-<31}:|')

    for mapping in mappings():
        rows.append('| {:32}| {:32}|'.format(
            mapping.ros2_package_name + '/' + mapping.ros2_message_name,
            mapping.ign_string()))

    print('\n'.join(rows))
