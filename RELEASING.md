# Release a new version of ros_ign_bridge

The package uses the latest versions of the Ignition libraries,
at this moment only available in the https://packages.osrfoundation.org repository.
This fact makes impossible to use bloom to release it using ROS servers.

The steps to make a new release of the package are:

1. Follow the standard ROS release using `catkin_generate_changelog` and
`catkin_prepare_release`
    ```
    http://wiki.ros.org/bloom/Tutorials/ReleaseCatkinPackage
    ```

1. Install `rosdep` for Ignition packages and run rosdep update
    ```
    sudo bash -c 'echo "yaml https://github.com/osrf/osrf-rosdep/raw/master/ignition/ignition.yaml" >> /etc/ros/rosdep/sources.list.d/00-ignition.list'
    rosdep update
    ```

1. Bloom it into a custom repository
    ```
    BLOOM_RELEASE_REPO_BASE=https://github.com/osrf/ bloom-release --no-pull-request --rosdistro melodic --track melodic ros_ign
    ```

    Will fail fedora: ignore and continue:
    ```
    # Would you like to try again? [Y/n]? n
    # Skip generator action and continue with release [y/N]? y
    ```

1. Use [release-tools](https://bitbucket.org/osrf/release-tools)'s script to launch jenkins jobs:
    ```
    cd release-tools/bloom
    ./ros_ign_bridge-release.py.bash 0.8.0 https://github.com/osrf/ros_ign-release token
    ```
