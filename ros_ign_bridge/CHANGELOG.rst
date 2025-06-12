^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_ign_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.244.20 (2025-06-12)
---------------------

0.244.19 (2025-05-26)
---------------------

0.244.18 (2025-05-23)
---------------------

0.244.17 (2025-05-06)
---------------------
* Added codespell pre-commit hook. (backport `#721 <https://github.com/gazebosim/ros_gz/issues/721>`_) (`#723 <https://github.com/gazebosim/ros_gz/issues/723>`_)
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
* Merge branch 'galactic' into ports/galactic_to_ros2
* Make tests faster and more robust (`#272 <https://github.com/gazebosim/ros_gz/issues/272>`_)
* Improve documentation around yaml configuration (`#271 <https://github.com/gazebosim/ros_gz/issues/271>`_)
* Fix small typo in bridge README (`#270 <https://github.com/gazebosim/ros_gz/issues/270>`_)
* Port NavSat (`#224 <https://github.com/gazebosim/ros_gz/issues/224>`_) from ROS 1 to ROS 2 (`#268 <https://github.com/gazebosim/ros_gz/issues/268>`_)
  Co-authored-by: Tyler Howell <76003804+TyHowellWork@users.noreply.github.com>
* Add ParamVec and bridge from Ignition (`#261 <https://github.com/gazebosim/ros_gz/issues/261>`_)
  * Introduces `ros_ign_interfaces::msg::ParamVec` for storing a list of Parameters that are int, bool, double, or string.
  * Introduces bridge for `ignition::msgs::param` to `ros_ign_interfaces::msg::ParamVec`
  * Introduces bridge for `ignition::msgs::param_v` to `ros_ign_interfaces::msg::ParamVec`
* Add support for converting Any <-> ParamValue (`#260 <https://github.com/gazebosim/ros_gz/issues/260>`_)
  * Add support for converting Any <-> ParamValue
* Feature: set QoS options to override durability (`#250 <https://github.com/gazebosim/ros_gz/issues/250>`_) (`#259 <https://github.com/gazebosim/ros_gz/issues/259>`_)
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
  Co-authored-by: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
* Add node component and yaml-configured bridge node (`#238 <https://github.com/gazebosim/ros_gz/issues/238>`_)
  * Refactor in support of adding yaml-configured node
* Add rssi to Dataframe.msg (`#249 <https://github.com/gazebosim/ros_gz/issues/249>`_)
  * Adding rssi field to ros_ign_interfaces/Dataframe.msg
* Use the python generator for tests as well (`#234 <https://github.com/gazebosim/ros_gz/issues/234>`_)
  * Use the python generator for tests as well
* Generate boilerplate files from Python scripts (`#233 <https://github.com/gazebosim/ros_gz/issues/233>`_)
  The way that we add factories can be a bit error-prone, as there are a lot of strings that cannot be checked at compilation time. This changes several of the boilerplate files to be generated automatically by python scripts, in line with how ros1_bridge does it.
* [galactic] Backport GuiCamera, StringVec, TrackVisual, VideoRecord (`#241 <https://github.com/gazebosim/ros_gz/issues/241>`_)
  * [ros_ign_interfaces] Add more interface definitions.
  * Add conversion functions for the added messages
  * Update the factory factory function with the new messages
  * Add new messages to docs
  * Add test cases for the new messages conversions
  Co-authored-by: Ivan Santiago Paunovic <ivanpauno@ekumenlabs.com>
* Add Dataframe message and bridging (`#239 <https://github.com/gazebosim/ros_gz/issues/239>`_)
* Factory interface needs virtual destructor (`#232 <https://github.com/gazebosim/ros_gz/issues/232>`_)
* Optional "lazy" bridge subscribers (`#225 <https://github.com/gazebosim/ros_gz/issues/225>`_)
  This allows for the bridge to be created in such a way that it is "lazy". In this case "lazy" means:
  * The publication (output) side of the bridge is always on and actively looking for subscriptions.
  * The subscription (input) side of the bridge is only turned on in the case that there are subscriptions on the output side.
* Contributors: Carlos Agüero, Louise Poubel, Michael Carroll

0.244.6 (2022-09-14)
--------------------
* Restructured directories (`#296 <https://github.com/gazebosim/ros_gz/issues/296>`_)
* Contributors: Alejandro Hernández Cordero

0.244.5 (2022-09-12)
--------------------
* ign -> gz : ros_gz Migration (Shims) (`#281 <https://github.com/gazebosim/ros_gz/issues/281>`_)
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
* Contributors: methylDragon
