^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_gz_interfaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.244.4 (2022-09-09)
--------------------
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
* Contributors: Louise Poubel

0.244.2 (2022-04-25)
--------------------
* [ros_gz_interfaces] Add GuiCamera, StringVec, TrackVisual, VideoRecord (`#214 <https://github.com/gazebosim/ros_gz/issues/214>`_)
  * [ros_gz_interfaces] Add more interface definitions.
  * Add converion functions for the added messages
  * Update the factory factory function with the new messages
  * Add new messages to docs
  * Add test cases for the new messages conversions
* Update maintainer for ros_gz_interfaces (`#204 <https://github.com/gazebosim/ros_gz/issues/204>`_)
* [ros2]  new package ros_gz_interfaces, provide some  Gazebo-specific ROS messages. (`#152 <https://github.com/gazebosim/ros_gz/issues/152>`_)
  * add new package ros_gz_interfaces,provide some Gazebo-specific ros .msg and .srv files
  * modify to match gz-msgs
  * add author info
  * modify comments
  * update code and doc style
* Contributors: Alejandro Hernández Cordero, Ivan Santiago Paunovic, Louise Poubel, Michael Carroll, ahcorde, gezp

0.244.1 (2022-01-04)
--------------------

0.244.0 (2021-12-30)
--------------------
* New Light Message, also bridge Color (`#187 <https://github.com/gazebosim/ros_gz/issues/187>`_)
* Expose Contacts through ROS bridge (`#175 <https://github.com/gazebosim/ros_gz/issues/175>`_)
* Contributors: Guillaume Doisy, Vatan Aksoy Tezer, William Lew

0.233.2 (2021-07-20)
--------------------
* [ros2]  new package ros_gz_interfaces, provide some  Gazebo-specific ROS messages. (`#152 <https://github.com/gazebosim/ros_gz/issues/152>`_)
  * add new package ros_gz_interfaces,provide some Gazebo-specific ros .msg and .srv files
  * modify to match gz-msgs
  * add author info
  * modify comments
  * update code and doc style
* Contributors: gezp
