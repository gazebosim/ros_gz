^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros1_ign
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.5 (2021-03-30)
------------------

0.9.4 (2020-12-10)
------------------

0.9.2 (2020-05-14)
------------------

0.9.1 (2020-05-13)
------------------

0.9.0 (2020-05-13)
------------------
* Update Melodic docs (`#61 <https://github.com/ignitionrobotics/ros_ign/issues/61>`_)
* [Citadel] Citadel support (`#48 <https://github.com/ignitionrobotics/ros_ign/issues/48>`_)
  * Citadel support
  * more citadel deps
  * addressing feedback, fix typos and better find logic
  * fix CI
* Contributors: chapulina

0.8.0 (2019-11-22)
------------------
* Add replaces for each package (`#46 <https://github.com/osrf/ros_ign/issues/46>`_)
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
* Merge pull request `#35 <https://github.com/osrf/ros1_ign_bridge/issues/35>`_ from osrf/image_meta
  Add ros_ign_image to metapackage
* Add ros_ign_image to metapackage
  Signed-off-by: chapulina <louise@openrobotics.org>
  typo
  Signed-off-by: chapulina <louise@openrobotics.org>
* Contributors: Nate Koenig, chapulina

0.6.0 (2019-08-02)
------------------

* 0.5.0 partial
* 0.5.0
* Merge pull request `#28 <https://github.com/osrf/ros1_ign_bridge/issues/28>`_ from osrf/pointcloudpacked
  Bridge point cloud packed
* Contributors: Nate Koenig

* Merge pull request `#28 <https://github.com/osrf/ros1_ign_bridge/issues/28>`_ from osrf/pointcloudpacked
  Bridge point cloud packed
* Contributors: Nate Koenig

0.4.0 (2019-07-16)
------------------

0.3.1 (2019-07-01)
------------------

0.3.0 (2019-06-28)
------------------
* 0.2.0
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
* Contributors: Nate Koenig, chapulina

0.2.2 (2019-05-20)
------------------

0.2.1 (2019-05-11)
------------------

0.2.0 (2019-05-09)
------------------

0.1.0 (2019-03-20)
------------------
