^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros1_ign_image
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.5 (2021-03-30)
------------------

0.9.4 (2020-12-10)
------------------
* Add dome + melodic to CI (`#126 <https://github.com/ignitionrobotics/ros_ign/issues/126>`_)
  Co-authored-by: Nate Koenig <nate@openrobotics.org>
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
* Install only what's necessary, rename builtin_interfaces (`#96 <https://github.com/ignitionrobotics/ros_ign/issues/96>`_)
* Choose collection based on environment variable (`#72 <https://github.com/ignitionrobotics/ros_ign/issues/72>`_)
  * Choose collection based on the environment variable IGNITION_VERSION
* Contributors: Louise Poubel, Nate Koenig

0.9.2 (2020-05-14)
------------------

0.9.1 (2020-05-13)
------------------

0.9.0 (2020-05-13)
------------------
* [Citadel] Citadel support (`#48 <https://github.com/ignitionrobotics/ros_ign/issues/48>`_)
  * Citadel support
  * more citadel deps
  * addressing feedback, fix typos and better find logic
  * fix CI
* Contributors: chapulina

0.8.0 (2019-11-22)
------------------
* Add replaces for each package (`#46 <https://github.com/osrf/ros_ign/issues/46>`_)
* Make all API and comments ROS-version agnostic
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* Rename packages and fix compilation + tests
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* Move files ros1 -> ros
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* Contributors: Louise Poubel, chapulina

0.7.0 (2019-08-15)
------------------

0.6.3 (2019-08-04)
------------------
* Merge pull request `#36 <https://github.com/osrf/ros1_ign_bridge/issues/36>`_ from osrf/restest_depen_image
  Missing rostest dependency in image package
* Contributors: Jose Luis Rivero

0.6.2 (2019-08-04)
------------------

0.6.1 (2019-08-04)
------------------

0.6.0 (2019-08-02)
------------------
* Image bridge using image_transport (`#34 <https://github.com/osrf/ros1_ign_bridge/issues/34>`_)
  * Image bridge using image_transport
  * tests for image
  * correct metapackage
  * tests with catkin
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
  * Revert changes from `#32 <https://github.com/osrf/ros1_ign_bridge/issues/32>`_
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* Contributors: chapulina
