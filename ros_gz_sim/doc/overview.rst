Overview
========

``ros_gz_sim`` provides the ROS 2 ↔ Gazebo Sim glue that is not message-level
bridging: a composable ROS 2 wrapper around ``gz sim server``, a suite of
command-line tools for spawning / removing / posing entities, a
:cpp:class:`~ros_gz_sim::Stopwatch` utility that honours the ROS simulated
clock, and a set of launch-file actions (``gz_server``, ``gz_spawn_model``)
for declarative use from XML and Python launch files.

For ROS ↔ Gazebo **message** bridging, see the companion
``ros_gz_bridge`` package.

Package Contents
----------------

.. code-block:: none

   ros_gz_sim/
   ├── include/ros_gz_sim/
   │   ├── Stopwatch.hpp               ← sim-clock-aware stopwatch
   │   ├── gzserver.hpp                ← GzServer composable node
   │   ├── gz_simulation_interfaces.hpp← REP-2116 simulation service hub
   │   ├── spawn_entity.hpp            ← EntitySpawner node + CLI parser
   │   ├── set_entity_pose.hpp         ← EntityPoseSetter node
   │   └── delete_entity.hpp           ← EntityDeleter node
   ├── src/
   │   ├── create.cpp / remove.cpp     ← gflags CLI tools
   │   ├── spawn_entity.cpp, delete_entity.cpp, set_entity_pose.cpp
   │   ├── gzserver.cpp                ← gzserver executable + component
   │   └── gz_simulation_interfaces/   ← service / action implementations
   ├── launch/                         ← reference launch files (py + xml)
   └── ros_gz_sim/                     ← Python launch-action plugins
       └── actions/
           ├── gzserver.py             ← GzServer launch action
           └── gz_spawn_model.py       ← GzSpawnModel launch action

Main Components
---------------

GzServer
~~~~~~~~

:cpp:class:`ros_gz_sim::GzServer` is a composable ROS 2 node that starts a
``gz sim`` server.  It accepts either an SDF file path or an SDF string via
node parameters:

.. code-block:: bash

   ros2 run ros_gz_sim gzserver --ros-args -p world_sdf_file:=shapes.sdf

The same node can be loaded into any ``rclcpp_components`` container, which is
how the :class:`ros_gz_sim.actions.GzServer` launch action runs it.

Entity management CLI tools
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Three CLI executables wrap the equivalent ``ros_gz_interfaces`` services.
The matching ROS node classes (:cpp:class:`EntitySpawner`,
:cpp:class:`EntityDeleter`, :cpp:class:`EntityPoseSetter`) are public so
that other nodes can invoke the same services programmatically:

- ``create`` / ``spawn_entity`` — spawn an SDF/URDF entity from a file, a
  string, or a Gazebo Fuel URI.
- ``remove`` / ``delete_entity`` — remove an entity by name, id, or type.
- ``set_entity_pose`` — move an entity to a new pose.

.. code-block:: bash

   ros2 run ros_gz_sim create --world default \
       --file https://fuel.gazebosim.org/1.0/openrobotics/models/Gazebo
   ros2 run ros_gz_sim remove -world default -entity Gazebo
   ros2 run ros_gz_sim set_entity_pose --entity Gazebo --x 1 --y 2 --z 0.5

See :doc:`user_api` for the corresponding C++ node classes.

Stopwatch
~~~~~~~~~

:cpp:class:`ros_gz_sim::Stopwatch` measures wall-clock or simulated-clock
durations.  Inject a clock via
:cpp:func:`~ros_gz_sim::Stopwatch::SetClock` to follow ``/clock``:

.. code-block:: cpp

   #include <ros_gz_sim/Stopwatch.hpp>

   ros_gz_sim::Stopwatch sw;
   sw.SetClock(node->get_clock());   // /clock when sim time is enabled
   sw.Start();
   do_work();
   sw.Stop();
   RCLCPP_INFO_STREAM(node->get_logger(), "elapsed: " << sw.ElapsedRunTime().seconds() << " s");

GzSimulationInterfaces
~~~~~~~~~~~~~~~~~~~~~~

:cpp:class:`ros_gz_sim::gz_simulation_interfaces::GzSimulationInterfaces`
exposes the REP-2116 ``simulation_interfaces`` service surface against a
running Gazebo Sim server: ``get_entities``, ``reset_simulation``,
``step_simulation``, ``get_simulation_state``, and so on.  Construct it with
the node that should host the services:

.. code-block:: cpp

   auto node = std::make_shared<rclcpp::Node>("sim_services");
   ros_gz_sim::gz_simulation_interfaces::GzSimulationInterfaces sim_if(node);
   rclcpp::spin(node);

Resource Path Configuration
---------------------------

Gazebo's asset lookup is controlled by ``GZ_SIM_RESOURCE_PATH`` and
``GZ_SIM_SYSTEM_PLUGIN_PATH``.  ``ros_gz_sim`` reads ``<gazebo_ros>`` export
blocks from every installed ROS 2 package and folds their ``gazebo_model_path``
and ``plugin_path`` entries into those environment variables at launch time:

.. code-block:: xml
   :caption: your_pkg/package.xml

   <export>
     <gazebo_ros gazebo_model_path="${prefix}/models"/>
     <gazebo_ros plugin_path="${prefix}/plugins"/>
   </export>

``${prefix}`` expands to the package's install prefix.  See
:class:`ros_gz_sim.actions.gzserver.GazeboRosPaths` for the implementation.

Launch Files
------------

``ros_gz_sim`` ships reference launch files under ``launch/``.  Each has both
a Python and an XML flavour so you can pick whichever fits your project:

- ``gz_sim.launch.py`` — full Gazebo Sim (server + GUI) with argument
  passthrough.
- ``gz_server.launch.py`` — server only, useful for headless tests.
- ``gz_spawn_model.launch.py`` — spawn an entity from a file, string, or
  topic.
- ``gz_remove_model.launch.py`` — remove an entity.
- ``ros_gz_sim.launch.py`` — ``gz_server`` + ``ros_gz_bridge``
  side-by-side.
- ``ros_gz_spawn_model.launch.py`` — spawn + bridge in one invocation.

Documentation Layout
--------------------

- :doc:`user_api` — C++ classes and node types.
- :doc:`launch_actions` — Python launch-action plugins.
- :doc:`executables` — command-line tools reference.
- :doc:`api` — index of the full API listing.
