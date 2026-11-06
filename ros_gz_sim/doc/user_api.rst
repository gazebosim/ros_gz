User API
========

The public C++ API is declared under ``include/ros_gz_sim/``.  This page
documents the types that other packages should build against; follow the
cross-reference links to jump to the full auto-generated Doxygen pages.

GzServer
--------

:cpp:class:`ros_gz_sim::GzServer` is a composable ROS 2 node that starts a
``gz sim`` server.  It accepts one of two node parameters:

- ``world_sdf_file`` (string) — path to an SDF world file.
- ``world_sdf_string`` (string) — inline SDF document.

Full API page:
:ref:`exhale_class_classros__gz__sim_1_1GzServer`.

.. code-block:: cpp

   #include <ros_gz_sim/gzserver.hpp>

   rclcpp::NodeOptions opts;
   opts.parameter_overrides({{"world_sdf_file", "shapes.sdf"}});
   auto gz = std::make_shared<ros_gz_sim::GzServer>(opts);
   gz->OnStart();
   rclcpp::spin(gz);

The same class is registered as an ``rclcpp_components`` plugin (``gzserver``
executable alias), so it can be loaded into any shared component container.

EntitySpawner
-------------

:cpp:class:`EntitySpawner` wraps the
``ros_gz_interfaces/srv/SpawnEntity`` service call.

Full API page:
:ref:`exhale_class_classEntitySpawner`.

.. code-block:: cpp

   #include <ros_gz_sim/spawn_entity.hpp>

   auto spawner = std::make_shared<EntitySpawner>();
   geometry_msgs::msg::Pose pose;  // identity
   spawner->spawn_entity("my_model", "/path/to/model.sdf", pose);

The free function :cpp:func:`parse_arguments` turns argv into a
:cpp:struct:`CommandLineArgs` struct that accepts orientation as
either a quaternion or Euler angles.

EntityDeleter
-------------

:cpp:class:`EntityDeleter` wraps the
``ros_gz_interfaces/srv/DeleteEntity`` service call.

Full API page:
:ref:`exhale_class_classEntityDeleter`.

.. code-block:: cpp

   #include <ros_gz_sim/delete_entity.hpp>

   auto deleter = std::make_shared<EntityDeleter>();
   deleter->delete_entity("my_model", /*id=*/0, /*type=*/0);

An entity can be identified by any of ``name``, ``id``, or ``type``; unused
identifiers should be left at their default value.

EntityPoseSetter
----------------

:cpp:class:`EntityPoseSetter` wraps the
``ros_gz_interfaces/srv/SetEntityPose`` service call.

Full API page:
:ref:`exhale_class_classEntityPoseSetter`.

.. code-block:: cpp

   #include <ros_gz_sim/set_entity_pose.hpp>

   auto setter = std::make_shared<EntityPoseSetter>();
   // Position + quaternion form
   setter->set_entity_pose("my_model", 0, 0,
                           1.0, 2.0, 0.5,       // x y z
                           0.0, 0.0, 0.0, 1.0,  // qx qy qz qw
                           /*use_quaternion=*/true);

Passing ``use_quaternion = false`` reinterprets the final four arguments as
Euler angles (roll, pitch, yaw, ignored).

Stopwatch
---------

:cpp:class:`ros_gz_sim::Stopwatch` is a minimal stopwatch that can follow
either the wall clock or a ROS simulated clock.  It tracks separate
"run" (running) and "stop" (paused) durations.

Full API page:
:ref:`exhale_class_classros__gz__sim_1_1Stopwatch`.

Typical usage:

.. code-block:: cpp

   #include <ros_gz_sim/Stopwatch.hpp>

   ros_gz_sim::Stopwatch sw;
   sw.SetClock(node->get_clock());   // honour /clock when use_sim_time is true
   sw.Start();
   do_work();
   sw.Stop();
   auto elapsed = sw.ElapsedRunTime();

The default constructor uses a wall-clock source; call
:cpp:func:`~ros_gz_sim::Stopwatch::SetClock` once to switch to sim time.

GzSimulationInterfaces
----------------------

:cpp:class:`ros_gz_sim::gz_simulation_interfaces::GzSimulationInterfaces`
exposes the REP-2116 ``simulation_interfaces`` surface on top of a running
Gazebo Sim server.  It advertises one ROS service per simulation operation:

- ``get_entities`` — list the entities currently in the world.
- ``get_entity_info`` / ``get_entities_states`` / ``get_entity_state`` —
  query per-entity information.
- ``set_entity_state`` — teleport / re-pose an entity.
- ``delete_entity`` — remove an entity.
- ``get_simulation_state`` / ``set_simulation_state`` — inspect or toggle
  the play / pause state.
- ``get_simulator_features`` — capability negotiation.
- ``reset_simulation`` — reset the world.
- ``step_simulation`` — step the simulation forward by N iterations.

Full API page:
:ref:`exhale_class_classros__gz__sim_1_1gz__simulation__interfaces_1_1GzSimulationInterfaces`.

Constructor:

.. code-block:: cpp

   #include <ros_gz_sim/gz_simulation_interfaces.hpp>

   auto node = std::make_shared<rclcpp::Node>("sim_services");
   ros_gz_sim::gz_simulation_interfaces::GzSimulationInterfaces sim_if(node);
   rclcpp::spin(node);
