User API
========

This page documents the C++ types users of ``ros_gz_bridge`` interact with
most often.  For the full auto-generated Doxygen listing of every class,
function, and file, follow the cross-reference links into the ``Full C++ API``
tree.

RosGzBridge
-----------

:cpp:class:`ros_gz_bridge::RosGzBridge` is the composable node that owns the
Gazebo transport node and all active bridge handles.

Full API page:
:ref:`exhale_class_classros__gz__bridge_1_1RosGzBridge`.

.. code-block:: cpp

   #include <ros_gz_bridge/ros_gz_bridge.hpp>

   auto node = std::make_shared<ros_gz_bridge::RosGzBridge>();
   rclcpp::spin(node);

Topic bridges are added via
:cpp:func:`~ros_gz_bridge::RosGzBridge::add_bridge`, which takes a
:cpp:struct:`~ros_gz_bridge::BridgeConfig`:

.. code-block:: cpp

   ros_gz_bridge::BridgeConfig config;
   config.ros_type_name  = "std_msgs/msg/String";
   config.ros_topic_name = "/chatter";
   config.gz_type_name   = "gz.msgs.StringMsg";
   config.gz_topic_name  = "/chatter";
   config.direction      = ros_gz_bridge::BridgeDirection::BIDIRECTIONAL;
   node->add_bridge(config);

Service bridges are added via
:cpp:func:`~ros_gz_bridge::RosGzBridge::add_service_bridge`:

.. code-block:: cpp

   node->add_service_bridge(
     "ros_gz_interfaces/srv/ControlWorld",
     "gz.msgs.WorldControl",
     "gz.msgs.Boolean",
     "/world/shapes/control");

BridgeConfig
------------

:cpp:struct:`ros_gz_bridge::BridgeConfig` captures everything needed to create
one topic or service bridge.

Full API page:
:ref:`exhale_struct_structros__gz__bridge_1_1BridgeConfig`.

Key fields:

- ``ros_type_name`` / ``ros_topic_name`` — ROS 2 side.
- ``gz_type_name`` / ``gz_topic_name`` — Gazebo side.
- ``direction`` — one of :cpp:enum:`~ros_gz_bridge::BridgeDirection`.
- ``subscriber_queue_size`` / ``publisher_queue_size`` — optional per-side
  queue depths.  When unset, the node-level default is used.
- ``is_lazy`` — when ``true``, the bridge only forwards messages if there is
  at least one subscriber on the destination side.
- ``qos_profile`` — optional ROS QoS profile; see :cpp:func:`ros_gz_bridge::parseQoS`.
- ``service_name``, ``gz_req_type_name``, ``gz_rep_type_name`` — used for
  service bridges instead of topic fields.
- ``frame_id`` — when non-empty, overrides the ``frame_id`` of bridged ROS
  messages that carry a ``Header``.

YAML loaders
------------

Configuration files are parsed via:

- :cpp:func:`ros_gz_bridge::readFromYamlString` — parse a YAML string.
- :cpp:func:`ros_gz_bridge::readFromYamlFile` — parse a YAML file on disk.

Both return a ``std::vector<BridgeConfig>`` that can be fed into
:cpp:func:`~ros_gz_bridge::RosGzBridge::add_bridge` one entry at a time.

QoS parsing
-----------

:cpp:func:`ros_gz_bridge::parseQoS` converts an uppercase QoS profile name
(e.g. ``"SENSOR_DATA"``, ``"SYSTEM_DEFAULT"``) into an ``rclcpp::QoS``
object.  Throws ``std::invalid_argument`` if the profile is unknown.

Constants
---------

- :cpp:var:`ros_gz_bridge::kDefaultSubscriberQueue`
- :cpp:var:`ros_gz_bridge::kDefaultPublisherQueue`
- :cpp:var:`ros_gz_bridge::kDefaultLazy`
- :cpp:var:`ros_gz_bridge::kDefaultDirection`
