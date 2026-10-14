Overview
========

``ros_gz_bridge`` is a bidirectional message bridge between ROS 2 and Gazebo
Transport.  Applications running against either middleware can exchange messages
transparently, provided a conversion exists for the message type.

The package ships with:

- A **core C++ library** (``libros_gz_bridge``) exposing
  :cpp:class:`~ros_gz_bridge::RosGzBridge` and the
  :cpp:struct:`~ros_gz_bridge::BridgeConfig` configuration structure.
- Three **executables**: ``parameter_bridge`` (configurable via CLI / YAML),
  ``static_bridge`` (hard-coded example), and ``bridge_types`` (lists the
  available message mappings).
- A **composable node** (``ros_gz_bridge::RosGzBridge``, exposed as
  ``bridge_node``) that can be loaded into any ``rclcpp_components`` container.
- A **Python launch action** (:class:`ros_gz_bridge.actions.RosGzBridge`) for
  declarative use from XML and Python launch files.
- A **message mapping registry** (:mod:`ros_gz_bridge.mappings`) used by the
  build-time code generators.

Architecture
------------

.. code-block:: none

   ros_gz_bridge/
   ├── include/ros_gz_bridge/
   │   ├── ros_gz_bridge.hpp   ← RosGzBridge composable node
   │   ├── bridge_config.hpp   ← BridgeConfig struct + YAML loader
   │   ├── convert.hpp         ← aggregate of per-message conversions
   │   └── convert/*.hpp       ← ROS ↔ GZ message-type specializations
   ├── src/
   │   ├── parameter_bridge.cpp, static_bridge.cpp, bridge_types.cpp
   │   ├── bridge_handle*.cpp  ← per-direction bridge handles
   │   └── factory*.{hpp,cpp}  ← hand-written factory interfaces + templates
   ├── ros_gz_bridge/          ← Python package
   │   ├── mappings.py         ← ROS ↔ GZ type mapping table
   │   └── actions/ros_gz_bridge.py  ← launch action plugin
   └── launch/                 ← reference launch files

The per-message factories are not checked into ``src/``.  They are emitted at
build time from ``mappings.py`` into
``${CMAKE_BINARY_DIR}/generated/factories/<pkg>.cpp``; ``src/`` holds only the
hand-written ``factory_interface.cpp`` / ``service_factory_interface.cpp`` and
the ``factory.hpp`` / ``service_factory.hpp`` templates they build on.  See
:doc:`conversions` for the generation pipeline.

A single :cpp:class:`~ros_gz_bridge::RosGzBridge` node owns one ``gz::transport::Node``
plus a set of ``BridgeHandle`` instances (one per bridged topic) and zero or
more ``rclcpp::Service`` entries (one per bridged service).  ``BridgeHandle`` is
internal to the implementation (``src/bridge_handle.hpp``); the public headers
only forward-declare it, so Doxygen never indexes it and there is no generated
API page to link to.

Bridge Directions
-----------------

Topic bridges are always configured with a direction, encoded by
:cpp:enum:`~ros_gz_bridge::BridgeDirection`:

- ``BIDIRECTIONAL`` — default; forwards in both directions.
- ``ROS_TO_GZ`` — forward ROS messages to Gazebo only.
- ``GZ_TO_ROS`` — forward Gazebo messages to ROS only.
- ``NONE`` — disables the bridge (used for dynamic configuration).

Running the parameter bridge
-----------------------------

The simplest form passes mappings as CLI arguments.  Each mapping is of the
form ``<topic>@<ros_type>[@<gz_type>]``:

.. code-block:: bash

   ros2 run ros_gz_bridge parameter_bridge \
     /chatter@std_msgs/msg/String@gz.msgs.StringMsg

Direction is controlled by the separator between the two type names:

- ``@`` — bidirectional
- ``[`` — Gazebo → ROS only
- ``]`` — ROS → Gazebo only

For larger deployments, pass a YAML configuration file via the
``config_file`` parameter:

.. code-block:: bash

   ros2 run ros_gz_bridge parameter_bridge \
     --ros-args -p config_file:=bridge.yaml

.. code-block:: yaml
   :caption: bridge.yaml

   - topic_name: /chatter
     ros_type_name: std_msgs/msg/String
     gz_type_name: gz.msgs.StringMsg
     direction: BIDIRECTIONAL
     qos_profile: SENSOR_DATA
   - service_name: /world/shapes/control
     ros_type_name: ros_gz_interfaces/srv/ControlWorld
     gz_req_type_name: gz.msgs.WorldControl
     gz_rep_type_name: gz.msgs.Boolean

Supported QoS profiles are:
``CLOCK``, ``SENSOR_DATA``, ``PARAMETERS``, ``SERVICES``,
``PARAMETER_EVENTS``, ``ROSOUT``, ``SYSTEM_DEFAULT``, or ``BEST_AVAILABLE``.

See :doc:`user_api` for the corresponding C++ types and
:doc:`launch_action` for the Python ``RosGzBridge`` launch action.

Composable Node
---------------

:cpp:class:`~ros_gz_bridge::RosGzBridge` is registered as an
``rclcpp_components`` plugin, so it can be loaded into a shared component
container:

.. code-block:: bash

   ros2 component load /ComponentManager ros_gz_bridge ros_gz_bridge::RosGzBridge \
     -p config_file:=bridge.yaml

Node Parameters
---------------

The parameter bridge recognizes the following ROS parameters.  The package
``README.md`` covers the same parameters and the full YAML schema; keep the two
in sync when adding or changing one.

- ``config_file`` (string, default: ``""``) — path to a YAML config file.
- ``bridge_names`` (array of strings, default: ``[]``) — list of bridge
  configuration names to load via parameter namespaces (e.g., ``bridges.<name>.ros_type_name``).
- ``lazy`` (bool, default: ``false``) — enable lazy subscription mode, where the
  bridge subscribes on the *source* side only while at least one subscriber is
  present on the *destination* side.  For a ``GZ_TO_ROS`` bridge the Gazebo
  subscription starts once a ROS subscriber appears; for ``ROS_TO_GZ`` the ROS
  subscription starts once a Gazebo subscriber connects.  Note that Gazebo
  transport can only report whether a publisher *has* connections, not how many
  or whether they are remote, so a bidirectional bridge always counts as having
  a destination-side subscriber and never goes idle.
- ``subscription_heartbeat`` (int, default: ``1000``) — period, in
  milliseconds, for the liveliness / lazy-subscription heartbeat.
- ``expand_gz_topic_names`` (bool, default: ``false``) — when ``true``, Gazebo
  topic names receive the node namespace.
- ``override_timestamps_with_wall_time`` (bool, default: ``false``) — overwrite
  message timestamps with the wall clock at bridge time.
- ``override_frame_id`` (string, default: ``""``) — if non-empty, replaces the
  ``frame_id`` of each bridged ROS header.

Documentation Layout
--------------------

- :doc:`user_api` — C++ classes, structs, and free functions.
- :doc:`conversions` — reference listing of conversion headers.
- :doc:`launch_action` — Python launch-action plugin.
- :doc:`tutorials/adding_a_message` — step-by-step guide to bridging a new
  message type, including tests.
- :doc:`api` — index of the full API listing.
