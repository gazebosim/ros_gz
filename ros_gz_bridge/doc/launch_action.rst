Launch Action
=============

``ros_gz_bridge`` ships with a Python launch action plugin that declaratively
starts the bridge from either XML or Python launch files.

The action is registered under the name ``ros_gz_bridge`` and implemented by
:class:`ros_gz_bridge.actions.RosGzBridge`.

XML launch file
---------------

.. code-block:: xml

   <launch>
     <ros_gz_bridge
         bridge_name="ros_gz_bridge"
         config_file="$(find-pkg-share my_pkg)/config/bridge.yaml"
         use_composition="true"
         use_respawn="false"
         log_level="info"/>
   </launch>

Python launch file
------------------

.. code-block:: python

   from launch import LaunchDescription
   from launch.substitutions import PathJoinSubstitution
   from launch_ros.substitutions import FindPackageShare

   from ros_gz_bridge.actions import RosGzBridge


   def generate_launch_description():
       return LaunchDescription([
           RosGzBridge(
               bridge_name='ros_gz_bridge',
               config_file=PathJoinSubstitution([
                   FindPackageShare('my_pkg'),
                   'config',
                   'bridge.yaml',
               ]),
               use_composition='true',
           ),
       ])

Arguments
---------

``bridge_name``
    Node name used by the bridge.

``config_file``
    Path to a YAML file describing the bridge configuration
    (same schema used by the ``parameter_bridge`` executable).

``container_name``
    Name of the component container to start (when ``create_own_container=true``)
    or attach to (when ``create_own_container=false``). Defaults to
    ``ros_gz_container``.

``create_own_container``
    When ``true`` and ``use_composition=true``, the action starts a fresh
    component container with the name specified by ``container_name``. When
    ``false`` and ``use_composition=true``, the bridge loads into an existing
    container with that name. Ignored when ``use_composition=false``.

``namespace``
    ROS namespace to push the bridge node into.

``use_composition``
    When ``true``, the bridge runs as a composable node.  When ``false``, a
    standalone process is launched instead.

``use_respawn``
    Whether to respawn the node if it crashes. Only applies when
    ``use_composition=false`` (respawn is not supported for composable nodes).

``log_level``
    Log level for the bridge node (e.g. ``"info"``, ``"debug"``).

``bridge_params``
    Extra key-value parameters forwarded to the bridge node.

Module Reference
----------------

Full API pages:

- :doc:`/ros_gz_bridge`
- :doc:`/ros_gz_bridge.mappings`
- :doc:`/ros_gz_bridge.actions.ros_gz_bridge`
