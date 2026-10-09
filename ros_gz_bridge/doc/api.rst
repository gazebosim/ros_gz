API Reference
=============

``ros_gz_bridge`` exposes both a C++ and a Python API:

- :doc:`user_api` — core C++ classes and free functions.
- :doc:`conversions` — per-message conversion headers.
- :doc:`launch_action` — Python launch action plugin.
- :doc:`tutorials/adding_a_message` — walkthrough for adding a new
  ROS ↔ Gazebo message pair.

C++ Classes and Structs
-----------------------

- :cpp:class:`ros_gz_bridge::RosGzBridge` —
  :ref:`exhale_class_classros__gz__bridge_1_1RosGzBridge`
- :cpp:struct:`ros_gz_bridge::BridgeConfig` —
  :ref:`exhale_struct_structros__gz__bridge_1_1BridgeConfig`
- :cpp:enum:`ros_gz_bridge::BridgeDirection`

Free Functions
--------------

- :cpp:func:`ros_gz_bridge::parseQoS`
- :cpp:func:`ros_gz_bridge::readFromYamlString`
- :cpp:func:`ros_gz_bridge::readFromYamlFile`
- :cpp:func:`ros_gz_bridge::convert_ros_to_gz` (templates)
- :cpp:func:`ros_gz_bridge::convert_gz_to_ros` (templates)

Python Modules
--------------

- :doc:`/ros_gz_bridge` — top-level package (``MessageMapping``, generators).
- :doc:`/ros_gz_bridge.mappings` — ROS ↔ GZ message mapping table.
- :doc:`/ros_gz_bridge.actions.ros_gz_bridge` — launch action.
