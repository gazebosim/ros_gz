^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_gz_sim
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.246.0 (2023-08-31)
--------------------
* Add harmonic CI (`#447 <https://github.com/gazebosim/ros_gz/issues/447>`_)
  * Add harmonic CI
  * Include garden options
  * Add harmonic stanza
  * Additional message headers
  ---------
* Replace deprecated ign_find_package with gz_find_package (`#432 <https://github.com/gazebosim/ros_gz/issues/432>`_)
  Co-authored-by: jmackay2 <jmackay@gmail.com>
* Port: humble to ros2 (`#386 <https://github.com/gazebosim/ros_gz/issues/386>`_)
* Merge branch 'humble' into mjcarroll/humble_to_ros2
* Update maintainers (`#376 <https://github.com/gazebosim/ros_gz/issues/376>`_)
* set on_exit_shutdown argument for gz-sim ExecuteProcess (`#355 <https://github.com/gazebosim/ros_gz/issues/355>`_)
* Humble ➡️ ROS2 (`#323 <https://github.com/gazebosim/ros_gz/issues/323>`_)
* Remove all ignition references on ROS 2 branch (`#302 <https://github.com/gazebosim/ros_gz/issues/302>`_)
  * Remove all shims
  * Update CMakeLists and package.xml for garden
  * Complete garden gz renaming
  * Drop fortress CI
* Contributors: Aditya Pande, Alejandro Hernández Cordero, Michael Carroll, ahcorde, andermi, jmackay2

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
* Export ROS Stopwatch library (`#299 <https://github.com/gazebosim/ros_gz/issues/299>`_) (`#322 <https://github.com/gazebosim/ros_gz/issues/322>`_)
  New Stopwatch library needs to be exported and built as shared
  Co-authored-by: Michael Anderson <anderson@mbari.org>
* Contributors: Michael Carroll

0.244.8 (2022-10-28)
--------------------

0.244.7 (2022-10-12)
--------------------
* Fix launch substitutions for ign_args (`#309 <https://github.com/gazebosim/ros_gz/issues/309>`_)
  * Fix launch substitutions for ign_args
* Merge pull request `#275 <https://github.com/gazebosim/ros_gz/issues/275>`_ (Galactic to Humble)
  Galactic to Humble
* Merge branch 'ros2' into ports/galactic_to_ros2
* Contributors: Michael Carroll

0.244.6 (2022-09-14)
--------------------

0.244.5 (2022-09-12)
--------------------
* Fix missing msgs include and packages.xml deps (`#292 <https://github.com/gazebosim/ros_gz/issues/292>`_)
  * Fix missing msgs include and packages.xml deps
  * Add additional conditions to support gz sim invocation
  * Fix cpplint
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
* Support bridging services (`#211 <https://github.com/gazebosim/ros_gz/issues/211>`_)
* Add std_msgs as dependency of ros_gz_sim (`#242 <https://github.com/gazebosim/ros_gz/issues/242>`_)
* Fixed ros_gz_sim launch file install directory (`#229 <https://github.com/gazebosim/ros_gz/issues/229>`_) (`#230 <https://github.com/gazebosim/ros_gz/issues/230>`_)
* Added ign_version launch argument to set ignition gazebo version (`#226 <https://github.com/gazebosim/ros_gz/issues/226>`_)
* Bring ros2 branch up-to-date with Rolling (`#213 <https://github.com/gazebosim/ros_gz/issues/213>`_)
* create.cpp usage message fixed for ros2 branch (`#207 <https://github.com/gazebosim/ros_gz/issues/207>`_)
* Separate galactic branch from ros2 branch (`#201 <https://github.com/gazebosim/ros_gz/issues/201>`_)
* 🏁 Dome EOL (`#198 <https://github.com/gazebosim/ros_gz/issues/198>`_)
* Contributors: Alejandro Hernández Cordero, Aryaman Shardul, Ivan Santiago Paunovic, Kenji Brameld, Louise Poubel, Michael Carroll, ahcorde

0.244.1 (2022-01-04)
--------------------

0.244.0 (2021-12-30)
--------------------
* Default to Fortress for Rolling (future Humble) (`#195 <https://github.com/gazebosim/ros_gz/issues/195>`_)
* [ros2] 🏁 Dome EOL (`#199 <https://github.com/gazebosim/ros_gz/issues/199>`_)
* Contributors: Guillaume Doisy, Louise Poubel

0.233.2 (2021-07-20)
--------------------
* [ros2] Update version docs, add Galactic and Fortress (`#164 <https://github.com/gazebosim/ros_gz/issues/164>`_)
* Contributors: Louise Poubel

0.233.1 (2021-04-16)
--------------------
* Default to Edifice for Rolling (`#150 <https://github.com/gazebosim/ros_gz/issues/150>`_)
* Edifice support (`#140 <https://github.com/gazebosim/ros_gz/issues/140>`_)
  Co-authored-by: Alejandro Hernández <ahcorde@gmail.com>
* Add topic flag to create robot  (`#128 <https://github.com/gazebosim/ros_gz/issues/128>`_)
  Now it is possible to run ros_gz_sim create specifying a topic as
  source of the robot description
  Add a launch file starting a ignition gazebo world and spawn a sphere in it.
  Additionally a rviz2 interface is loaded to show that also Rviz can load
  the robot description
  The newly created demo introduce a dependency on the robot_state_publisher package
* Add default value for plugin path in launch script (`#125 <https://github.com/gazebosim/ros_gz/issues/125>`_)
* Fix overwriting of plugin path in launch script (`#122 <https://github.com/gazebosim/ros_gz/issues/122>`_)
  - GZ_SIM_SYSTEM_PLUGIN_PATH was overwritten by LD_LIBRARY_PATH
  - Now it is instead extended by LD_LIBRARY_PATH
  - This allows use of gz_sim.launch.py with custom gazebo plugins
* Changed for loading xml from ROS param(`#119 <https://github.com/gazebosim/ros_gz/issues/119>`_) (`#120 <https://github.com/gazebosim/ros_gz/issues/120>`_)
* ros_gz_sim exec depend on gz-sim (`#110 <https://github.com/gazebosim/ros_gz/issues/110>`_)
* Update releases (`#108 <https://github.com/gazebosim/ros_gz/issues/108>`_)
* Add support for Dome (`#103 <https://github.com/gazebosim/ros_gz/issues/103>`_)
* Contributors: Andrej Orsula, Louise Poubel, Luca Della Vedova, Valerio Magnago, chama1176

0.221.1 (2020-08-19)
--------------------
* Add pkg-config as a buildtool dependency (`#102 <https://github.com/gazebosim/ros_gz/issues/102>`_)
* Contributors: Louise Poubel

0.221.0 (2020-07-23)
--------------------
* [ros2] Fixed CI - Added Foxy (`#89 <https://github.com/gazebosim/ros_gz/issues/89>`_)
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
* Added ros_gz_sim for ros2 (`#80 <https://github.com/gazebosim/ros_gz/issues/80>`_)
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
* Update Dashing docs (`#62 <https://github.com/gazebosim/ros_gz/issues/62>`_)
* Contributors: Alejandro Hernández Cordero, Louise Poubel, chapulina
