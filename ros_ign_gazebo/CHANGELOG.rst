^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_ign_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.244.2 (2022-04-25)
--------------------
* Support bridging services (`#211 <https://github.com/osrf/ros_ign/issues/211>`_)
* Add std_msgs as dependency of ros_ign_gazebo (`#242 <https://github.com/osrf/ros_ign/issues/242>`_)
* Fixed ros_ign_gazebo launch file install directory (`#229 <https://github.com/osrf/ros_ign/issues/229>`_) (`#230 <https://github.com/osrf/ros_ign/issues/230>`_)
* Added ign_version launch argument to set ignition gazebo version (`#226 <https://github.com/osrf/ros_ign/issues/226>`_)
* Bring ros2 branch up-to-date with Rolling (`#213 <https://github.com/osrf/ros_ign/issues/213>`_)
* create.cpp usage message fixed for ros2 branch (`#207 <https://github.com/osrf/ros_ign/issues/207>`_)
* Separate galactic branch from ros2 branch (`#201 <https://github.com/osrf/ros_ign/issues/201>`_)
* 🏁 Dome EOL (`#198 <https://github.com/osrf/ros_ign/issues/198>`_)
* Contributors: Alejandro Hernández Cordero, Aryaman Shardul, Ivan Santiago Paunovic, Kenji Brameld, Louise Poubel, Michael Carroll, ahcorde

0.244.1 (2022-01-04)
--------------------

0.244.0 (2021-12-30)
--------------------
* Default to Fortress for Rolling (future Humble) (`#195 <https://github.com/osrf/ros_ign/issues/195>`_)
* [ros2] 🏁 Dome EOL (`#199 <https://github.com/osrf/ros_ign/issues/199>`_)
* Contributors: Guillaume Doisy, Louise Poubel

0.233.2 (2021-07-20)
--------------------
* [ros2] Update version docs, add Galactic and Fortress (`#164 <https://github.com/osrf/ros_ign/issues/164>`_)
* Contributors: Louise Poubel

0.233.1 (2021-04-16)
--------------------
* Default to Edifice for Rolling (`#150 <https://github.com/osrf/ros_ign/issues/150>`_)
* Edifice support (`#140 <https://github.com/osrf/ros_ign/issues/140>`_)
  Co-authored-by: Alejandro Hernández <ahcorde@gmail.com>
* Add topic flag to create robot  (`#128 <https://github.com/osrf/ros_ign/issues/128>`_)
  Now it is possible to run ros_ign_gazebo create specifying a topic as
  source of the robot description
  Add a launch file starting a ignition gazebo world and spawn a sphere in it.
  Additionally a rviz2 interface is loaded to show that also Rviz can load
  the robot description
  The newly created demo introduce a dependency on the robot_state_publisher package
* Add default value for plugin path in launch script (`#125 <https://github.com/osrf/ros_ign/issues/125>`_)
* Fix overwriting of plugin path in launch script (`#122 <https://github.com/osrf/ros_ign/issues/122>`_)
  - IGN_GAZEBO_SYSTEM_PLUGIN_PATH was overwritten by LD_LIBRARY_PATH
  - Now it is instead extended by LD_LIBRARY_PATH
  - This allows use of ign_gazebo.launch.py with custom gazebo plugins
* Changed for loading xml from ROS param(`#119 <https://github.com/osrf/ros_ign/issues/119>`_) (`#120 <https://github.com/osrf/ros_ign/issues/120>`_)
* ros_ign_gazebo exec depend on ign-gazebo (`#110 <https://github.com/osrf/ros_ign/issues/110>`_)
* Update releases (`#108 <https://github.com/osrf/ros_ign/issues/108>`_)
* Add support for Dome (`#103 <https://github.com/osrf/ros_ign/issues/103>`_)
* Contributors: Andrej Orsula, Louise Poubel, Luca Della Vedova, Valerio Magnago, chama1176

0.221.1 (2020-08-19)
--------------------
* Add pkg-config as a buildtool dependency (`#102 <https://github.com/osrf/ros_ign/issues/102>`_)
* Contributors: Louise Poubel

0.221.0 (2020-07-23)
--------------------
* [ros2] Fixed CI - Added Foxy (`#89 <https://github.com/osrf/ros_ign/issues/89>`_)
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
* Added ros_ign_gazebo for ros2 (`#80 <https://github.com/osrf/ros_ign/issues/80>`_)
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
* Update Dashing docs (`#62 <https://github.com/osrf/ros_ign/issues/62>`_)
* Contributors: Alejandro Hernández Cordero, Louise Poubel, chapulina
