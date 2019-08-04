# Release a new version of ros_ign_bridge

The package uses the latest versions of ignition libraries, at this moment only
availables in the packages.osrfoundation.org repository. This fact makes
impossible to use bloom to release it using ros servers.

The steps to make a new release of the package is:

 1. Follow the standard ROS release using catkin_generate_changelog and catkin_prepare_release
    - http://wiki.ros.org/bloom/Tutorials/ReleaseCatkinPackage

 1. Install rosdep for ignition packages and run rosdep update
    - sudo bash -c 'echo "yaml https://github.com/osrf/osrf-rosdep/raw/master/ignition/ignition.yaml" >> /etc/ros/rosdep/sources.list.d/00-ignition.list'
    - rosdep update

 1. Bloom it into a custom repository
    -  bloom-release --rosdistro melodic --track melodic ros_ign
    -  [use repository: git@github.com:osrf/ros_ign-release.git]
    -  [will fail fedora: ignore and continue]

 1. Use release-tools/bloom/ros_ign_bridge-release.py.bash to launch jenkins jobs
