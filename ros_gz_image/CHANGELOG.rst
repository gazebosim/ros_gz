^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros1_ign_image
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.244.19 (2025-05-26)
---------------------

0.244.18 (2025-05-23)
---------------------

0.244.17 (2025-05-06)
---------------------
* Added codespell pre-commit hook. (backport `#721 <https://github.com/gazebosim/ros_gz/issues/721>`_) (`#723 <https://github.com/gazebosim/ros_gz/issues/723>`_)
* Add pre commit (backport `#718 <https://github.com/gazebosim/ros_gz/issues/718>`_) (`#720 <https://github.com/gazebosim/ros_gz/issues/720>`_)
* Contributors: mergify[bot]

0.244.16 (2024-07-22)
---------------------

0.244.15 (2024-07-03)
---------------------

0.244.14 (2024-04-08)
---------------------

0.244.13 (2024-01-23)
---------------------

0.244.12 (2023-12-13)
---------------------
* Add support for Harmonic/Humble pairing (`#462 <https://github.com/gazebosim/ros_gz/issues/462>`_)
* Fix double wait in ros_gz_bridge (`#347 <https://github.com/gazebosim/ros_gz/issues/347>`_) (`#450 <https://github.com/gazebosim/ros_gz/issues/450>`_)
* Contributors: Addisu Z. Taddese, Alejandro Hernández Cordero

0.244.11 (2023-05-23)
---------------------

0.244.10 (2023-05-03)
---------------------

0.244.9 (2022-11-03)
--------------------

0.244.8 (2022-10-28)
--------------------

0.244.7 (2022-10-12)
--------------------
* Merge branch 'ros2' into ports/galactic_to_ros2
* Contributors: Michael Carroll

0.244.6 (2022-09-14)
--------------------

0.244.5 (2022-09-12)
--------------------
* Add missing GZ_VERSION ticktocks (`#289 <https://github.com/gazebosim/ros_gz/issues/289>`_)
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
* Contributors: methylDragon

0.244.3 (2022-05-19)
--------------------
* [ros2] README updates (service bridge, Gazebo rename) (`#252 <https://github.com/gazebosim/ros_gz/issues/252>`_)
* Fix linter tests (`#251 <https://github.com/gazebosim/ros_gz/issues/251>`_)
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
* Contributors: Daisuke Nishimatsu, Louise Poubel

0.244.2 (2022-04-25)
--------------------
* Bring ros2 branch up-to-date with Rolling (`#213 <https://github.com/gazebosim/ros_gz/issues/213>`_)
* Separate galactic branch from ros2 branch (`#201 <https://github.com/gazebosim/ros_gz/issues/201>`_)
* 🏁 Dome EOL (`#198 <https://github.com/gazebosim/ros_gz/issues/198>`_)
* Fix Deprecation Warning (`#158 <https://github.com/gazebosim/ros_gz/issues/158>`_)
* Contributors: David V. Lu!!, Louise Poubel, Michael Carroll

0.244.1 (2022-01-04)
--------------------

0.244.0 (2021-12-30)
--------------------
* Default to Fortress for Rolling (future Humble) (`#195 <https://github.com/gazebosim/ros_gz/issues/195>`_)
* [ros2] 🏁 Dome EOL (`#199 <https://github.com/gazebosim/ros_gz/issues/199>`_)
* Statically link each translation unit (`#193 <https://github.com/gazebosim/ros_gz/issues/193>`_)
* Contributors: Guillaume Doisy, Louise Poubel, Michael Carroll

0.233.2 (2021-07-20)
--------------------
* [ros2] Update version docs, add Galactic and Fortress (`#164 <https://github.com/gazebosim/ros_gz/issues/164>`_)
* Fix Deprecation Warning (`#158 <https://github.com/gazebosim/ros_gz/issues/158>`_)
* Contributors: David V. Lu!!, Louise Poubel

0.233.1 (2021-04-16)
--------------------
* Default to Edifice for Rolling (`#150 <https://github.com/gazebosim/ros_gz/issues/150>`_)
* Edifice support (`#140 <https://github.com/gazebosim/ros_gz/issues/140>`_)
* Update releases (`#108 <https://github.com/gazebosim/ros_gz/issues/108>`_)
* Add support for Dome (`#103 <https://github.com/gazebosim/ros_gz/issues/103>`_)
* Contributors: Louise Poubel, Luca Della Vedova

0.221.1 (2020-08-19)
--------------------
* Add pkg-config as a buildtool dependency (`#102 <https://github.com/gazebosim/ros_gz/issues/102>`_)
* Contributors: Louise Poubel

0.221.0 (2020-07-23)
--------------------
* Install only what's necessary, rename builtin_interfaces (`#95 <https://github.com/gazebosim/ros_gz/issues/95>`_)
* Add CI for Eloquent (`#86 <https://github.com/gazebosim/ros_gz/issues/86>`_)
* Avoid the use of --ros-args arguments outside ros (`#84 <https://github.com/gazebosim/ros_gz/issues/84>`_)
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
