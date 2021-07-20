^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_ign_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
