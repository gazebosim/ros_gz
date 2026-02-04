^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_gz_sim
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.244.22 (2026-02-04)
---------------------

0.244.21 (2026-01-20)
---------------------
* Add dependency to ros2pkg (backport `#816 <https://github.com/gazebosim/ros_gz/issues/816>`_) (`#819 <https://github.com/gazebosim/ros_gz/issues/819>`_)
* Contributors: mergify[bot]

0.244.20 (2025-06-12)
---------------------
* OS agnostic 'which' command (`#762 <https://github.com/gazebosim/ros_gz/issues/762>`_) (`#764 <https://github.com/gazebosim/ros_gz/issues/764>`_)
* Contributors: mergify[bot]

0.244.19 (2025-05-26)
---------------------
* Fix debug_env (`#747 <https://github.com/gazebosim/ros_gz/issues/747>`_) (`#750 <https://github.com/gazebosim/ros_gz/issues/750>`_)
* Contributors: mergify[bot]

0.244.18 (2025-05-23)
---------------------
* Log environment variables with which gazebo was launched (backport `#680 <https://github.com/gazebosim/ros_gz/issues/680>`_) (`#745 <https://github.com/gazebosim/ros_gz/issues/745>`_)
* Contributors: mergify[bot]

0.244.17 (2025-05-06)
---------------------

0.244.16 (2024-07-22)
---------------------

0.244.15 (2024-07-03)
---------------------

0.244.14 (2024-04-08)
---------------------
* Support `<gazebo_ros>` in `package.xml` exports (`#492 <https://github.com/gazebosim/ros_gz/issues/492>`_)
  This copies the implementation from `gazebo_ros_paths.py` to provide a
  way for packages to set resource paths from `package.xml`.
  ```
  e.g.  <export>
  <gazebo_ros gazebo_model_path="${prefix}/models"/>
  <gazebo_ros gazebo_media_path="${prefix}/models"/>
  </export>
  ```
  The value of `gazebo_model_path` and `gazebo_media_path` is appended to `GZ_SIM_RESOURCE_PATH`
  The value of `plugin_path` appended to `GZ_SIM_SYSTEM_PLUGIN_PATH`
  ---------
* Contributors: Addisu Z. Taddese

0.244.13 (2024-01-23)
---------------------

0.244.12 (2023-12-13)
---------------------
* Add support for Harmonic/Humble pairing (`#462 <https://github.com/gazebosim/ros_gz/issues/462>`_)
* Set on_exit_shutdown argument for gz-sim ExecuteProcess (`#355 <https://github.com/gazebosim/ros_gz/issues/355>`_) (`#451 <https://github.com/gazebosim/ros_gz/issues/451>`_)
* Contributors: Addisu Z. Taddese, Michael Carroll

0.244.11 (2023-05-23)
---------------------

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
