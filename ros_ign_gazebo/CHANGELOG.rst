^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_ign_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.5 (2021-03-30)
------------------

0.9.4 (2020-12-10)
------------------
* Add dome + melodic to CI (`#126 <https://github.com/ignitionrobotics/ros_ign/issues/126>`_)
  Co-authored-by: Nate Koenig <nate@openrobotics.org>
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
* Choose collection based on environment variable (`#72 <https://github.com/ignitionrobotics/ros_ign/issues/72>`_)
  * Choose collection based on the environment variable IGNITION_VERSION
* Contributors: Louise Poubel, Nate Koenig

0.9.2 (2020-05-14)
------------------
* Merge pull request `#77 <https://github.com/osrf/ros_ign/issues/77>`_ from ignitionrobotics/j-rivero-patch-2
  Change ignition-gazebo2 from exec_depend to build_depend in ros_ign_gazebo
* Change ignition-gazebo2 from exec_depend to build_depend in ros_ign_gazebo
* Contributors: Jose Luis Rivero

0.9.1 (2020-05-13)
------------------
* Merge pull request `#71 <https://github.com/osrf/ros_ign/issues/71>`_ from ignitionrobotics/0_9_0gazebo2_fix
  Fix gazebo version in package.xml files
* Contributors: Nate Koenig

0.9.0 (2020-05-13)
------------------
* ros_ign_gazebo package, with launch and spawn (`#60 <https://github.com/ignitionrobotics/ros_ign/issues/60>`_)
  * create ros_ign_gazebo package and move ign_gazebo exec and launch files there
  * Port create executable from create branch - ROS interface to spawn entities
  * use correct gflags key
  * PR feedback
* Contributors: chapulina
