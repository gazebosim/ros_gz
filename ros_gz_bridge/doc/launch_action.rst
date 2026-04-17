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
    When composing the bridge into an existing component container, the name
    of that container.

``create_own_container``
    When ``true``, the action starts a fresh component container and loads the
    bridge into it.  Ignored when ``container_name`` is set.

``namespace``
    ROS namespace to push the bridge node into.

``use_composition``
    When ``true``, the bridge runs as a composable node.  When ``false``, a
    standalone process is launched instead.

``use_respawn``
    Passes through to the underlying ``Node`` / ``ComposableNode`` action.

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
