^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros1_ign_image
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.221.0 (2020-07-23)
--------------------
* Install only what's necessary, rename builtin_interfaces (`#95 <https://github.com/osrf/ros_ign/issues/95>`_)
* Add CI for Eloquent (`#86 <https://github.com/osrf/ros_ign/issues/86>`_)
* Avoid the use of --ros-args arguments outside ros (`#84 <https://github.com/osrf/ros_ign/issues/84>`_)
* [WIP] Port ign_ros_gazebo_demos to ROS2 (`#58 <https://github.com/osrf/ros_ign/issues/58>`_)
  Port ros_ign_image to ROS2
  Port ros_ign_gazebo_demos to ROS2
* Enable ROS2 CI for Dashing branch (`#43 <https://github.com/osrf/ros_ign/issues/43>`_)
* Make all API and comments ROS-version agnostic
* Rename packages and fix compilation + tests
* Move files ros1 -> ros
* Contributors: Alejandro Hernández Cordero, Jose Luis Rivero, Louise Poubel, chapulina

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
