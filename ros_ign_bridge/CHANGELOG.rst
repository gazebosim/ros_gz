^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_ign_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.233.4 (2022-02-07)
--------------------
* [galactic] backport test memory usage improvements (`#215 <https://github.com/ignitionrobotics/ros_ign/issues/215>`_)
  - Improve modularity of ign/ros publisher tests (`#194 <https://github.com/ignitionrobotics/ros_ign/issues/194>`_)
  - Break apart ros_subscriber test translation unit (`#212 <https://github.com/ignitionrobotics/ros_ign/issues/212>`_)
  - Fix deprecated parameter declaration
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
* Contributors: Michael Carroll

0.233.3 (2021-12-30)
--------------------
* Separate galactic branch from ros2 branch (`#201 <https://github.com/osrf/ros_ign/issues/201>`_)
* [ros2] 🏁 Dome EOL (`#199 <https://github.com/osrf/ros_ign/issues/199>`_)
* New Light Message, also bridge Color (`#187 <https://github.com/osrf/ros_ign/issues/187>`_)
  Co-authored-by: ahcorde <ahcorde@gmail.com>
* Statically link each translation unit (`#193 <https://github.com/osrf/ros_ign/issues/193>`_)
* Break apart convert and factories translation unit (`#192 <https://github.com/osrf/ros_ign/issues/192>`_)
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
* Fixed ROS subscriber test in ros_ign_bridge (`#189 <https://github.com/osrf/ros_ign/issues/189>`_)
* Enable QoS overrides (`#181 <https://github.com/osrf/ros_ign/issues/181>`_)
* Fixed ros ign bridge documentation (`#178 <https://github.com/osrf/ros_ign/issues/178>`_)
* Expose Contacts through ROS bridge (`#175 <https://github.com/osrf/ros_ign/issues/175>`_)
* Contributors: Alejandro Hernández Cordero, Louise Poubel, Michael Carroll, Vatan Aksoy Tezer, William Lew

0.233.2 (2021-07-20)
--------------------
* [ros2] Update version docs, add Galactic and Fortress (`#164 <https://github.com/osrf/ros_ign/issues/164>`_)
* Contributors: Louise Poubel

0.233.1 (2021-04-16)
--------------------
* Default to Edifice for Rolling (`#150 <https://github.com/osrf/ros_ign/issues/150>`_)
* Ignore local publications for ROS 2 subscriber (`#146 <https://github.com/osrf/ros_ign/issues/146>`_)
  - Note: Does not work with all rmw implementations (e.g.: FastRTPS)
* Update documentation for installation instructions and bridge examples (`#142 <https://github.com/osrf/ros_ign/issues/142>`_)
* Edifice support (`#140 <https://github.com/osrf/ros_ign/issues/140>`_)
* Add JointTrajectory message conversion (`#121 <https://github.com/osrf/ros_ign/issues/121>`_)
  Conversion between
  - ignition::msgs::JointTrajectory
  - trajectory_msgs::msg::JointTrajectory
* Add TFMessage / Pose_V and Float64 / Double conversions (`#117 <https://github.com/osrf/ros_ign/issues/117>`_)
  Addresses issue `#116 <https://github.com/osrf/ros_ign/issues/116>`_
* Updated prereq & branch name (`#113 <https://github.com/osrf/ros_ign/issues/113>`_)
* Update releases (`#108 <https://github.com/osrf/ros_ign/issues/108>`_)
* Updated README.md (`#104 <https://github.com/osrf/ros_ign/issues/104>`_)
* Add support for Dome (`#103 <https://github.com/osrf/ros_ign/issues/103>`_)
* Contributors: Alejandro Hernández Cordero, Andrej Orsula, Florent Audonnet, Jenn, Louise Poubel, Luca Della Vedova

0.221.1 (2020-08-19)
--------------------
* Add pkg-config as a buildtool dependency (`#102 <https://github.com/osrf/ros_ign/issues/102>`_)
* Port ros_ign_bridge tests to ROS 2 (`#98 <https://github.com/osrf/ros_ign/issues/98>`_)
* Rename test_utils.hpp (`#98 <https://github.com/osrf/ros_ign/issues/98>`_)
* Contributors: Louise Poubel, ahcorde

0.221.0 (2020-07-23)
--------------------
* Install only what's necessary, rename builtin_interfaces (`#95 <https://github.com/osrf/ros_ign/issues/95>`_)
* Move headers to src, rename builtin_interfaces (`#95 <https://github.com/osrf/ros_ign/issues/95>`_)
* Integer support (`#91 <https://github.com/osrf/ros_ign/issues/91>`_)
  Adds Int32 to the bridge.
* [ros2] Fixed CI - Added Foxy (`#89 <https://github.com/osrf/ros_ign/issues/89>`_)
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
* Ignore ros-args in parameter bridge (`#65 <https://github.com/osrf/ros_ign/issues/65>`_)
* Update Dashing docs (`#62 <https://github.com/osrf/ros_ign/issues/62>`_)
* Update dependencies to Citadel (`#57 <https://github.com/osrf/ros_ign/issues/57>`_)
* [WIP] Port ign_ros_gazebo_demos to ROS2 (`#58 <https://github.com/osrf/ros_ign/issues/58>`_)
  Port ros_ign_image to ROS2
  Port ros_ign_gazebo_demos to ROS2
* Add support for std_msgs/Empty (`#53 <https://github.com/osrf/ros_ign/issues/53>`_)
* Add support for std_msgs/Bool (`#50 <https://github.com/osrf/ros_ign/issues/50>`_)
* [ros2] Port ros_ign_bridge to ROS2 (`#45 <https://github.com/osrf/ros_ign/issues/45>`_)
* Enable ROS2 CI for Dashing branch (`#43 <https://github.com/osrf/ros_ign/issues/43>`_)
* Make all API and comments ROS-version agnostic
* Rename packages and fix compilation + tests
* Move files ros1 -> ros
* Contributors: Addisu Taddese, Alejandro Hernández Cordero, Jose Luis Rivero, Louise Poubel, Luca Della Vedova, Michael Carroll, Mohamed Ahmed, Shivesh Khaitan, chapulina

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
