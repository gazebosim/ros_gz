# Release a new version of ros_ign

Different branches of this repository support different combinations of
ROS 1, ROS 2 and Ignition. Refer to the [README](README.md) to see what
combinations are supported.

We use 2 separate approaches for releasing ROS-enabled binaries:

* Release upstream into https://packages.ros.org, so it's easily available to
  all ROS users.
* Release into https://packages.osrfoundation.org, which requires extra setup
  for ROS users.

Releasing into ROS packages is ideal, but that's only possible if the necessary
Ignition libraries are available either through official Ubuntu packages, or
directly from https://packages.ros.org. This situation varies according to the
Ignition version:

* Citadel is available from:
    * https://packages.osrfoundation.org: all Ubuntu versions
    * https://packages.ros.org: only Ubuntu Focal
* Dome is only available from https://packages.osrfoundation.org
* Edifice is available from:
    * https://packages.osrfoundation.org: all Ubuntu versions
    * https://packages.ros.org: only Ubuntu Focal, once ROS 2 Galactic is out

Another factor to take into consideration is which Ignition version is officially
supported for each ROS distro according to the following REPS:

* ROS 1: [REP-0003](https://ros.org/reps/rep-0003.html)
* ROS 2: [REP-2000](https://www.ros.org/reps/rep-2000.html)

These factors determine which ROS + Ignition combinations are released into each
repository.

## Versioning

All `ros_ign` packages are under fast development and haven't reached version
1.0 yet. The team will make an effort to keep changes backwards compatible
within a ROS distribution, but API / ABI / behaviour may be broken if necessary.

The versioning scheme uses the minor version to identify ROS and Ignition
versions:

* 1st digit:
    * ROS 1: 1
    * ROS 2: 2
* 2nd digit:
    * Melodic: 0
    * Noetic: 1
    * Dashing: 0
    * Eloquent: 1
    * Foxy: 2
* 3rd digit:
    * Blueprint: 0
    * Citadel: 1
    * Dome: 2
    * Edifice: 3
    * Fortress: 4

ROS | Ignition | Version
-- | -- | --
Melodic | Blueprint | 0.100.X
Melodic | Citadel | 0.101.X
Noetic | Blueprint | :x:
Noetic | Citadel | 0.111.X
Noetic | Dome | 0.112.X
Noetic | Edifice | 0.113.X
Dashing | Blueprint | 0.200.X
Dashing | Citadel | 0.201.X
Eloquent | Blueprint | 0.210.X
Eloquent | Citadel | 0.211.X
Foxy | Blueprint | :x:
Foxy | Citadel | 0.221.X
Foxy | Dome | 0.222.X
Foxy | Edifice | 0.223.X

## Releasing into https://packages.ros.org

1. Follow the [standard ROS release process](http://wiki.ros.org/bloom/Tutorials/ReleaseCatkinPackage).

## Releasing into https://packages.osrfoundation.org

1. From the [standard ROS release process](http://wiki.ros.org/bloom/Tutorials/ReleaseCatkinPackage),
   follow just these steps:

    * `catkin_generate_changelog`
    * `catkin_prepare_release`

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
    ./ros_ign_bridge-release.py.bash 0.8.0 https://github.com/osrf/ros_ign-release <ros_distro> <token> --ignition-version <version_name>
    ```
