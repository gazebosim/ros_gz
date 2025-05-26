^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_ign_interfaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Add ParamVec and bridge from Ignition (`#261 <https://github.com/gazebosim/ros_gz/issues/261>`_)
  * Introduces `ros_ign_interfaces::msg::ParamVec` for storing a list of Parameters that are int, bool, double, or string.
  * Introduces bridge for `ignition::msgs::param` to `ros_ign_interfaces::msg::ParamVec`
  * Introduces bridge for `ignition::msgs::param_v` to `ros_ign_interfaces::msg::ParamVec`
* Add rssi to Dataframe.msg (`#249 <https://github.com/gazebosim/ros_gz/issues/249>`_)
  * Adding rssi field to ros_ign_interfaces/Dataframe.msg
* [galactic] Backport GuiCamera, StringVec, TrackVisual, VideoRecord (`#241 <https://github.com/gazebosim/ros_gz/issues/241>`_)
  * [ros_ign_interfaces] Add more interface definitions.
  * Add conversion functions for the added messages
  * Update the factory factory function with the new messages
  * Add new messages to docs
  * Add test cases for the new messages conversions
  Co-authored-by: Ivan Santiago Paunovic <ivanpauno@ekumenlabs.com>
* Add Dataframe message and bridging (`#239 <https://github.com/gazebosim/ros_gz/issues/239>`_)
* Contributors: Carlos Agüero, Michael Carroll

0.244.6 (2022-09-14)
--------------------
* Restructured directories (`#296 <https://github.com/gazebosim/ros_gz/issues/296>`_)
* Contributors: Alejandro Hernández Cordero

0.244.5 (2022-09-12)
--------------------
* ign -> gz : ros_gz Migration (Shims) (`#281 <https://github.com/gazebosim/ros_gz/issues/281>`_)
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
* Contributors: methylDragon
