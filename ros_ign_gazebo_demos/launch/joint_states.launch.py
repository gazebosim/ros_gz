import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # Package Directories
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg_ros_ign_gazebo_demos = get_package_share_directory('ros_ign_gazebo_demos')

    # Parse robot description from urdf
    robot_description_file =  os.path.join(pkg_ros_ign_gazebo_demos, "models", "rrbot.urdf")
    robot_description_config = open(robot_description_file).read()
    robot_description = {"robot_description": robot_description_config}

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Ignition gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_ros_ign_gazebo_demos, 'rviz', 'joint_states.rviz')],
        parameters=[]
    )

    # Spawn
    rrbot_sdf_path =  os.path.join(pkg_ros_ign_gazebo_demos, "models", "rrbot.sdf")
    print(rrbot_sdf_path)
    spawn = Node(package='ros_ign_gazebo', executable='create',
                arguments=[
                    '-name', 'rrbot',
                    '-file', rrbot_sdf_path,
                    ],
                output='screen',
                )

    # Ign Bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
                # Clock (IGN -> ROS2)
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                # Joint states (IGN -> ROS2)
                '/world/default/model/rrbot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
                ],
        remappings=[
            ("/world/default/model/rrbot/joint_state", "joint_states"),
        ],
        output='screen'
    )

    # Static TF between world and robot
    world_static_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='world_static_tf',
                        output='log',
                        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'link1', 'world'])

    return LaunchDescription(
        [
            # Nodes and Launches
            gazebo,
            spawn,
            bridge,
            robot_state_publisher,
            world_static_tf,
            rviz,
        ]
    )
