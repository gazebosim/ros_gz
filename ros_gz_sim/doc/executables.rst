Executables
===========

All executables are invoked through ``ros2 run ros_gz_sim <name>`` or the
equivalent launch actions.

gzserver
--------

Runs :cpp:class:`ros_gz_sim::GzServer` either standalone or as a composable
node.  Accepts ROS parameters:

- ``world_sdf_file`` — path to an SDF world.
- ``world_sdf_string`` — inline SDF.
- ``initial_sim_time`` — seconds the simulated clock starts at.

.. code-block:: bash

   ros2 run ros_gz_sim gzserver --ros-args -p world_sdf_file:=shapes.sdf

Use the :class:`ros_gz_sim.actions.GzServer` launch action to embed this in a
larger launch file.

create
------

Spawns an entity using the Gazebo transport protocol directly.  Uses ``gflags``
for argument parsing (``--helpshort`` for the full list).

.. code-block:: bash

   ros2 run ros_gz_sim create \
       --world default \
       --file https://fuel.gazebosim.org/1.0/openrobotics/models/Gazebo \
       --name my_gazebo \
       --x 0 --y 0 --z 0.5

``create`` is preferred over ``spawn_entity`` when you already have a running
Gazebo transport channel and do not want the ROS service layer.

spawn_entity
------------

Invokes :cpp:class:`EntitySpawner` to spawn via the
``ros_gz_interfaces/srv/SpawnEntity`` ROS service.  Prefer this in launch
files because it plays nicely with ROS event handlers and parameters.

.. code-block:: bash

   ros2 run ros_gz_sim spawn_entity \
       --name my_robot --file /path/to/robot.sdf \
       --x 0 --y 0 --z 0.1

Orientation may be passed either as a quaternion (``--qx --qy --qz --qw``) or
Euler angles (``--roll --pitch --yaw``).  See
:cpp:func:`parse_arguments`.

remove
------

Removes an entity via Gazebo transport.  Mirrors ``create``:

.. code-block:: bash

   ros2 run ros_gz_sim remove -world default -entity my_gazebo

delete_entity
-------------

Invokes :cpp:class:`EntityDeleter` against
``ros_gz_interfaces/srv/DeleteEntity``:

.. code-block:: bash

   ros2 run ros_gz_sim delete_entity --name my_robot

An entity can be identified by name, numeric id, or type; unused identifiers
may be omitted.

set_entity_pose
---------------

Invokes :cpp:class:`EntityPoseSetter` against
``ros_gz_interfaces/srv/SetEntityPose``:

.. code-block:: bash

   ros2 run ros_gz_sim set_entity_pose \
       --entity my_robot --x 1 --y 2 --z 0.5

Pass ``--use-quaternion=false`` to interpret the orientation arguments as
Euler angles.

Reference Launch Files
----------------------

``ros_gz_sim`` installs reference launch files that compose the executables
above.  Each one ships in both Python (``.launch.py``) and frontend XML
(``.launch``) formats:

- ``gz_sim.launch.py`` — full server + GUI.
- ``gz_server.launch.py`` — server only.
- ``gz_spawn_model.launch.py`` — wrap ``create``.
- ``gz_remove_model.launch.py`` — wrap ``remove``.
- ``ros_gz_sim.launch.py`` — ``gz_server`` plus ``ros_gz_bridge``.
- ``ros_gz_spawn_model.launch.py`` — spawn an entity and bridge it.

They are intended as copy-paste templates — point your own launch file at the
one that matches your setup and override arguments from the outside.
