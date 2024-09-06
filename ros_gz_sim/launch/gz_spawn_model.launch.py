from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ros_gz_sim.actions import GzServer
from ros_gz_bridge.actions import RosGzBridge


def generate_launch_description():

    return LaunchDescription([
        GzServer(
            world_sdf_file='empty.sdf'),
        RosGzBridge(
            name='bridge1',
            config_file=PathJoinSubstitution([FindPackageShare('ros_gz_bridge'), 'config/full.yaml'])),
    ])
