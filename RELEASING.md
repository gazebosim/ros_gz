# Release a new version of ros_ign_bridge

The package uses the latest versions of ignition libraries, at this moment only
availables in the packages.osrfoundation.org repository. This fact makes
impossible to use bloom to release it. The steps to make a new release of the
package is:

 1. Use github to make an official release:
    https://github.com/osrf/ros_ign/releases

 1. Update metadata for packaging in:
    https://bitbucket.org/osrf/ros_ign_bridge-release/

 1. Launch manually the builds in:
    https://build.osrfoundation.org/job/ros_ign_bridge-debbuilder/
