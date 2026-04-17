Launch Actions
==============

``ros_gz_sim`` ships two Python launch action plugins that make it easy to
declare Gazebo Sim and entity-spawning steps from XML or Python launch files.
Both are discovered by ``launch`` via the ``@expose_action`` decorator.

GzServer
--------

:class:`ros_gz_sim.actions.GzServer` starts the ``gz sim`` server, either as
a standalone node or inside a shared component container.

Registered name: ``gz_server``.

XML form
~~~~~~~~

.. code-block:: xml

   <launch>
     <gz_server
         world_sdf_file="$(find-pkg-share my_pkg)/worlds/shapes.sdf"
         use_composition="true"
         verbosity_level="3"/>
   </launch>

Python form
~~~~~~~~~~~

.. code-block:: python

   from launch import LaunchDescription
   from launch_ros.substitutions import FindPackageShare
   from launch.substitutions import PathJoinSubstitution

   from ros_gz_sim.actions import GzServer


   def generate_launch_description():
       return LaunchDescription([
           GzServer(
               world_sdf_file=PathJoinSubstitution([
                   FindPackageShare('my_pkg'), 'worlds', 'shapes.sdf',
               ]),
               use_composition='true',
               initial_sim_time='0.0',
           ),
       ])

Arguments
~~~~~~~~~

``world_sdf_file``
    Path to an SDF world file.  Mutually exclusive with ``world_sdf_string``.

``world_sdf_string``
    Inline SDF document.

``container_name``
    Name of the component container to load the server into (when
    ``use_composition`` is true and ``create_own_container`` is false).

``create_own_container``
    When ``true``, the action launches a dedicated component container.

``use_composition``
    When ``true``, runs ``GzServer`` as a composable node; otherwise as a
    standalone process.

``initial_sim_time``
    Value (seconds) to initialise the simulated clock at.

``verbosity_level``
    Gazebo verbosity 0–4; forwarded to ``gz sim --verbose``.

GzSpawnModel
------------

:class:`ros_gz_sim.actions.GzSpawnModel` calls the ``create`` executable to
spawn an entity into a running simulation.

Registered name: ``gz_spawn_model``.

XML form
~~~~~~~~

.. code-block:: xml

   <launch>
     <gz_spawn_model
         world="default"
         file="$(find-pkg-share my_pkg)/models/robot.sdf"
         entity_name="robot"
         x="0.0" y="0.0" z="0.5"
         yaw="1.5707"/>
   </launch>

Python form
~~~~~~~~~~~

.. code-block:: python

   from ros_gz_sim.actions import GzSpawnModel

   spawn = GzSpawnModel(
       world='default',
       file='/path/to/robot.sdf',
       entity_name='robot',
       x='0.0', y='0.0', z='0.5', yaw='1.5707',
   )

Arguments
~~~~~~~~~

``world``
    Target Gazebo world name.

``file``
    Path or Fuel URI of the SDF/URDF file to spawn.  Mutually exclusive with
    ``model_string`` and ``topic``.

``model_string``
    Inline SDF/URDF document.

``topic``
    Topic carrying the model description (``robot_description`` style).

``entity_name``
    Desired entity name in the simulation.

``allow_renaming``
    If ``true``, Gazebo appends a suffix when ``entity_name`` is already
    taken.

``x``, ``y``, ``z``
    Spawn position in metres.

``roll``, ``pitch``, ``yaw``
    Spawn orientation in radians (Euler).

GazeboRosPaths utility
----------------------

:class:`ros_gz_sim.actions.gzserver.GazeboRosPaths` discovers the combined
``GZ_SIM_RESOURCE_PATH`` and ``GZ_SIM_SYSTEM_PLUGIN_PATH`` values by scanning
every installed ROS 2 package for ``<gazebo_ros>`` export entries.  It is
used internally by :class:`~ros_gz_sim.actions.GzServer` but is available
for downstream launch code that needs the same behaviour:

.. code-block:: python

   from ros_gz_sim.actions.gzserver import GazeboRosPaths

   model_path, plugin_path = GazeboRosPaths.get_paths()

Module Reference
----------------

Auto-generated module pages (from sphinx-apidoc):

- :doc:`/ros_gz_sim`
- :doc:`/ros_gz_sim.actions`
- :doc:`/ros_gz_sim.actions.gzserver`
- :doc:`/ros_gz_sim.actions.gz_spawn_model`
