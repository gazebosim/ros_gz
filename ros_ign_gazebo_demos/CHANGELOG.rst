^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros1_ign_gazebo_demos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.3 (2020-05-14)
------------------

0.9.2 (2020-05-14)
------------------

0.9.1 (2020-05-13)
------------------
* Merge pull request `#71 <https://github.com/osrf/ros_ign/issues/71>`_ from ignitionrobotics/0_9_0gazebo2_fix
  Fix gazebo version in package.xml files
* Contributors: Nate Koenig

0.9.0 (2020-05-13)
------------------
* Corrected error in launch (`#66 <https://github.com/ignitionrobotics/ros_ign/issues/66>`_)
* ros_ign_gazebo package, with launch and spawn (`#60 <https://github.com/ignitionrobotics/ros_ign/issues/60>`_)
  * create ros_ign_gazebo package and move ign_gazebo exec and launch files there
  * Port create executable from create branch - ROS interface to spawn entities
  * use correct gflags key
  * PR feedback
* [Citadel] Citadel support (`#48 <https://github.com/ignitionrobotics/ros_ign/issues/48>`_)
  * Citadel support
  * more citadel deps
  * addressing feedback, fix typos and better find logic
  * fix CI
* Correct demo frame names (`#54 <https://github.com/ignitionrobotics/ros_ign/issues/54>`_)
* Contributors: John, RDaneelOlivav, chapulina

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
* Revert "Pointcloud bridge demo for depth camera"
  This reverts commit 094cd40f21aed734d59c204172ad5afd7a26c8d6.
* Pointcloud bridge demo for depth camera
* Contributors: Louise Poubel, chapulina

* 0.5.0
* Battery state (`#30 <https://github.com/osrf/ros1_ign_bridge/issues/30>`_)
* Packed demo (`#29 <https://github.com/osrf/ros1_ign_bridge/issues/29>`_)
  * adding demo for point cloud packed bridge
  * correct rviz file
  * RGBD bridged cloud demo
* Merge pull request `#28 <https://github.com/osrf/ros1_ign_bridge/issues/28>`_ from osrf/pointcloudpacked
  Bridge point cloud packed
* Contributors: Nate Koenig, chapulina

* Battery state (`#30 <https://github.com/osrf/ros1_ign_bridge/issues/30>`_)
* Packed demo (`#29 <https://github.com/osrf/ros1_ign_bridge/issues/29>`_)
  * adding demo for point cloud packed bridge
  * correct rviz file
  * RGBD bridged cloud demo
* Merge pull request `#28 <https://github.com/osrf/ros1_ign_bridge/issues/28>`_ from osrf/pointcloudpacked
  Bridge point cloud packed
* Contributors: Nate Koenig, chapulina

0.4.0 (2019-07-16)
------------------

0.3.1 (2019-07-01)
------------------
* Merge pull request `#24 <https://github.com/osrf/ros1_ign_bridge/issues/24>`_ from osrf/fix_dep
  ignition-gazebo2 needed at build time
* ignition-gazebo2 needed at build time
* Contributors: Jose Luis Rivero

0.3.0 (2019-06-28)
------------------
* 0.2.0
* Merge pull request `#21 <https://github.com/osrf/ros1_ign_bridge/issues/21>`_ from osrf/lidar
  Point clouds from lidars
* Conversion between nav_msgs/Odometry and ignition::msgs::Odometry (`#22 <https://github.com/osrf/ros1_ign_bridge/issues/22>`_)
  * Conversion between nav_msgs/Odometry and ignition::msgs::Odometry.
  * Update documentation.
  * More time to run tests
  * Cleaning test_utils.
  * Remove explicit ROS dependencies for Travis.
  * diff drive demo with cmd_vel and odom
  * process child frame id
* final tweaks
* PC2 for gpu_lidar, 1 vertical sample
* Start of lidar PC
* Fluid pressure (`#20 <https://github.com/osrf/ros1_ign_bridge/issues/20>`_)
  * screenshots
  * missing IMU
  * Fluid pressure
  * Fix tests.
* Demos package (`#19 <https://github.com/osrf/ros1_ign_bridge/issues/19>`_)
  * Start of demos package: camera
  * IMU
  * depth camera
  * magnetometer
  * lidar, base launch
  * READMEs, RGBD camera
  * screenshots
  * missing IMU
  * set plugin path env
  * It's best to always set it
* Contributors: Carlos Agüero, Nate Koenig, chapulina

0.2.2 (2019-05-20)
------------------

0.2.1 (2019-05-11)
------------------

0.2.0 (2019-05-09)
------------------

0.1.0 (2019-03-20)
------------------
