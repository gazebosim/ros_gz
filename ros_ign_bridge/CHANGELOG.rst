^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros1_ign_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.3 (2020-05-14)
------------------
* Revert msgs5 and transport8 dependencies to keep compatibility
* Contributors: Jose Luis Rivero

0.9.2 (2020-05-14)
------------------

0.9.1 (2020-05-13)
------------------

0.9.0 (2020-05-13)
------------------
* Add ignition::msgs::Pose_V to tf2_msgs::TFMessage conversion (`#67 <https://github.com/ignitionrobotics/ros_ign/issues/67>`_)
  * add ign pose_v to ros tf2_message bridge
  * add tf2 msgs dependency
* Add Float64 to ignition Double conversions.  This is needed for suppo… (`#64 <https://github.com/ignitionrobotics/ros_ign/issues/64>`_)
  * Add Float64 to ignition Double conversions.  This is needed for supporting joint controller (such as for a pan/tilt gimbal)
  * Update README to reflect that bridge now supports double message conversions.
* Update Melodic docs (`#61 <https://github.com/ignitionrobotics/ros_ign/issues/61>`_)
* Patches for Citadel release (`#56 <https://github.com/ignitionrobotics/ros_ign/issues/56>`_)
  * Patches for Citadel release
  * Mention Citadel or Blueprint deps
* Merge pull request `#55 <https://github.com/ignitionrobotics/ros_ign/issues/55>`_ from osrf/fix_repo_url
  Update repo URL in README install isntructions
* Update repo URL in README install isntructions
* [Citadel] Citadel support (`#48 <https://github.com/ignitionrobotics/ros_ign/issues/48>`_)
  * Citadel support
  * more citadel deps
  * addressing feedback, fix typos and better find logic
  * fix CI
* Contributors: Jose Luis Rivero, chapulina, iche033, realdealneil

0.8.0 (2019-11-22)
------------------
* Add support for std_msgs/Empty (`#52 <https://github.com/osrf/ros_ign/issues/52>`_)
* Add support for std_msgs/Bool (`#49 <https://github.com/osrf/ros_ign/issues/49>`_)
  Signed-off-by: Michael Carroll <michael@openrobotics.org>
* Add replaces for each package (`#46 <https://github.com/osrf/ros_ign/issues/46>`_)
* Make all API and comments ROS-version agnostic
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* Rename packages and fix compilation + tests
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* Move files ros1 -> ros
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* Contributors: Addisu Taddese, Louise Poubel, Michael Carroll, chapulina

0.7.0 (2019-08-15)
------------------
* Merge pull request `#38 <https://github.com/osrf/ros1_ign_bridge/issues/38>`_ from osrf/unidirectional
  Support unidirectional bridge topics
* More examples
* Merge pull request `#37 <https://github.com/osrf/ros1_ign_bridge/issues/37>`_ from osrf/debug
  Adding debug and error statements
* Switch to characters supported by ros
* Merge branch 'debug' into unidirectional
* More output, and rosconsole depend
* Support specification of bridge direction
* Adding debug and error statements
* Contributors: Nate Koenig

0.6.3 (2019-08-04)
------------------

0.6.2 (2019-08-04)
------------------

0.6.1 (2019-08-04)
------------------
* Update README.md
* Contributors: Carlos Agüero

0.6.0 (2019-08-02)
------------------
* Merge pull request `#33 <https://github.com/osrf/ros1_ign_bridge/issues/33>`_ from osrf/issue_31
  Fix issue `#31 <https://github.com/osrf/ros1_ign_bridge/issues/31>`_
* Image bridge using image_transport (`#34 <https://github.com/osrf/ros1_ign_bridge/issues/34>`_)
  * Image bridge using image_transport
  * tests for image
  * correct metapackage
  * tests with catkin
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
  * Revert changes from `#32 <https://github.com/osrf/ros1_ign_bridge/issues/32>`_
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* Use intra-process field from messageInfo.
* Contributors: Carlos Aguero, Nate Koenig, chapulina

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
* tests and reverse bridge for pointcloud
* Bridge point cloud packed
* Contributors: Nate Koenig

0.3.1 (2019-07-01)
------------------

0.3.0 (2019-06-28)
------------------
* 0.2.0
* Conversion between nav_msgs/Odometry and ignition::msgs::Odometry (`#22 <https://github.com/osrf/ros1_ign_bridge/issues/22>`_)
  * Conversion between nav_msgs/Odometry and ignition::msgs::Odometry.
  * Update documentation.
  * More time to run tests
  * Cleaning test_utils.
  * Remove explicit ROS dependencies for Travis.
  * diff drive demo with cmd_vel and odom
  * process child frame id
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
* Point clouds for RGBD cameras (`#17 <https://github.com/osrf/ros1_ign_bridge/issues/17>`_)
  * Beginning of point cloud package
  * Populating image data, but result is not correct. Must find out where's the source of the problem.
  * RGB -> BGR: why?
  * Cleanup code and example
  * pointcloud -> point_cloud
  * add keys - how was this working before?
  * install wget
  * well, we need ign-gz2 :sweat_smile:
  * README update
  * PR feedback
  * .travis/build: rosdep skip ignition keys (`#18 <https://github.com/osrf/ros1_ign_bridge/issues/18>`_)
  * .travis/build: rosdep skip ignition keys
  * Update build
* Move package to subfolder, add metapackage (`#16 <https://github.com/osrf/ros1_ign_bridge/issues/16>`_)
* Contributors: Carlos Agüero, Nate Koenig, chapulina

0.2.2 (2019-05-20)
------------------

0.2.1 (2019-05-11)
------------------

0.2.0 (2019-05-09)
------------------

0.1.0 (2019-03-20)
------------------
