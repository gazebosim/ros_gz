^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros1_gz_sim_demos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.245.0 (2022-10-12)
--------------------
* humble to ros2 (`#311 <https://github.com/gazebosim/ros_gz/issues/311>`_)
  Co-authored-by: Michael Carroll <michael@openrobotics.org>
* Merge remote-tracking branch 'origin/humble' into ahcorde/humble_to_ros2
* Remove all ignition references on ROS 2 branch (`#302 <https://github.com/gazebosim/ros_gz/issues/302>`_)
  * Remove all shims
  * Update CMakeLists and package.xml for garden
  * Complete garden gz renaming
  * Drop fortress CI
* Contributors: Alejandro Hernández Cordero, Michael Carroll, ahcorde

0.244.10 (2023-05-03)
---------------------

0.244.9 (2022-11-03)
--------------------

0.244.8 (2022-10-28)
--------------------
* Fixed ros_gz_sim_demos launch files (`#319 <https://github.com/gazebosim/ros_gz/issues/319>`_) (`#320 <https://github.com/gazebosim/ros_gz/issues/320>`_)
* Contributors: Alejandro Hernández Cordero

0.244.7 (2022-10-12)
--------------------
* Merge pull request `#275 <https://github.com/gazebosim/ros_gz/issues/275>`_ (Galactic to Humble)
  Galactic to Humble
* Merge branch 'ros2' into ports/galactic_to_ros2
* Contributors: Michael Carroll

0.244.6 (2022-09-14)
--------------------

0.244.5 (2022-09-12)
--------------------
* sdformat_urdf parser demo (`#265 <https://github.com/gazebosim/ros_gz/issues/265>`_)
  * parser compatible model and launch framework
  * added ground plane, common gz plugins, demo commands and cleaned install paths
  * unique collision names and cleared flake
  * updating model config
  * building parser from source
  * fix flake and update deb dependency for garden
  * Move packages and files to gz
  * feedback and ign->gz
  * Support ros_ign migration
  Clean up shared libraries, and tick-tock RosGzPointCloud
  Tick-tock launch args
  Hard-tock ign\_ in sources
  Migrate ign, ign\_, IGN\_ for sources, launch, and test files
  Migrate IGN_XXX_VER, IGN_T, header guards
  Migrate launchfile, launchfile args, and test source references
  Migrate ros_ign_XXX and gz_gazebo -> gz_sim
  Migrate ros_ign_XXX project names
  Migrate Ign, ign-, IGN_DEPS, ign-gazebo
  Migrate ignitionrobotics, ignitionrobotics/ros_ign, osrf/ros_ign
  Migrate ignition-version, IGNITION_VERSION, Ignition <LIB>, ros_ign_ci
  * renaming and flake
  * added ros commands
  * gz-version
  * feedback and ci trial
  * removing garden condition
  Co-authored-by: Michael Carroll <michael@openrobotics.org>
  Co-authored-by: methylDragon <methylDragon@gmail.com>
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
* Support ros_ign migration (`#282 <https://github.com/gazebosim/ros_gz/issues/282>`_)
  Clean up shared libraries, and tick-tock RosGzPointCloud
  Tick-tock launch args
  Hard-tock ign\_ in sources
  Migrate ign, ign\_, IGN\_ for sources, launch, and test files
  Migrate IGN_XXX_VER, IGN_T, header guards
  Migrate launchfile, launchfile args, and test source references
  Migrate ros_ign_XXX and gz_gazebo -> gz_sim
  Migrate ros_ign_XXX project names
  Migrate Ign, ign-, IGN_DEPS, ign-gazebo
  Migrate ignitionrobotics, ignitionrobotics/ros_ign, osrf/ros_ign
  Migrate ignition-version, IGNITION_VERSION, Ignition <LIB>, ros_ign_ci
* Move packages and files to gz (`#282 <https://github.com/gazebosim/ros_gz/issues/282>`_)
* Contributors: Dharini Dutia, methylDragon

0.244.3 (2022-05-19)
--------------------
* [ros2] README updates (service bridge, Gazebo rename) (`#252 <https://github.com/gazebosim/ros_gz/issues/252>`_)
* Fix linter tests (`#251 <https://github.com/gazebosim/ros_gz/issues/251>`_)
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
* Joint state publisher and tf bridging demo (`#244 <https://github.com/gazebosim/ros_gz/issues/244>`_)
  * Added joint state publisher and tf bridge demo
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
* Contributors: Aditya Pande, Daisuke Nishimatsu, Louise Poubel

0.244.2 (2022-04-25)
--------------------
* Camera trigger demo (`#223 <https://github.com/gazebosim/ros_gz/issues/223>`_)
* Separate galactic branch from ros2 branch (`#201 <https://github.com/gazebosim/ros_gz/issues/201>`_)
* 🏁 Dome EOL (`#198 <https://github.com/gazebosim/ros_gz/issues/198>`_)
* Joint states tutorial (`#156 <https://github.com/gazebosim/ros_gz/issues/156>`_)
  Adds an rrbot model to demos and shows the usage of joint_states plugin.
* Contributors: Louise Poubel, Michael Carroll, Vatan Aksoy Tezer, William Lew

0.244.1 (2022-01-04)
--------------------

0.244.0 (2021-12-30)
--------------------
* Default to Fortress for Rolling (future Humble) (`#195 <https://github.com/gazebosim/ros_gz/issues/195>`_)
* [ros2] 🏁 Dome EOL (`#199 <https://github.com/gazebosim/ros_gz/issues/199>`_)
* Enable QoS overrides (`#181 <https://github.com/gazebosim/ros_gz/issues/181>`_)
* Contributors: Guillaume Doisy, Louise Poubel

0.233.2 (2021-07-20)
--------------------
* [ros2] Add exec depend on xacro for demos (`#170 <https://github.com/gazebosim/ros_gz/issues/170>`_)
* [ros2] Update version docs, add Galactic and Fortress (`#164 <https://github.com/gazebosim/ros_gz/issues/164>`_)
* Joint states tutorial (`#156 <https://github.com/gazebosim/ros_gz/issues/156>`_)
  Adds an rrbot model to demos and shows the usage of joint_states plugin.
* Contributors: Louise Poubel, Vatan Aksoy Tezer

0.233.1 (2021-04-16)
--------------------
* Default to Edifice for Rolling (`#150 <https://github.com/gazebosim/ros_gz/issues/150>`_)
* Minor updates for demos (`#144 <https://github.com/gazebosim/ros_gz/issues/144>`_)
  * Re-enable air pressure demo
  - Resolves https://github.com/gazebosim/ros_gz/issues/78
  * Add RQt topic viewer to IMU demo
  * Add image_topic argument for image_bridge demo
  * Do not normalize depth image in RViz2
* Edifice support (`#140 <https://github.com/gazebosim/ros_gz/issues/140>`_)
* Add topic flag to create robot  (`#128 <https://github.com/gazebosim/ros_gz/issues/128>`_)
  Now it is possible to run ros_gz_sim create specifying a topic as
  source of the robot description
  Add a launch file starting a ignition gazebo world and spawn a sphere in it.
  Additionally a rviz2 interface is loaded to show that also Rviz can load
  the robot description
  The newly created demo introduce a dependency on the robot_state_publisher package
* [ros2] Update releases (`#108 <https://github.com/gazebosim/ros_gz/issues/108>`_)
* Contributors: Andrej Orsula, Louise Poubel, Valerio Magnago

0.221.1 (2020-08-19)
--------------------

0.221.0 (2020-07-23)
--------------------
* Updated launch file to use ros_gz_sim (`#82 <https://github.com/gazebosim/ros_gz/issues/82>`_)
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
* Use new ros_gz_sim package on ROS 2 demos (`#85 <https://github.com/gazebosim/ros_gz/issues/85>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* [WIP] Port ign_ros_gazebo_demos to ROS2 (`#58 <https://github.com/gazebosim/ros_gz/issues/58>`_)
  Port ros_gz_image to ROS2
  Port ros_gz_sim_demos to ROS2
* Enable ROS2 CI for Dashing branch (`#43 <https://github.com/gazebosim/ros_gz/issues/43>`_)
* Make all API and comments ROS-version agnostic
* Rename packages and fix compilation + tests
* Move files ros1 -> ros
* Contributors: Alejandro Hernández Cordero, Jose Luis Rivero, Louise Poubel, chapulina

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
