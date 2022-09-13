# Description

These are shim packages for `ros_gz` that redirect executable calls to the related `ros_gz` executable.
These require `ros_gz` to be installed though, but are set up to depend on `ros_gz`.

This allows users to do either of these and get equivalent behavior:

```bash
ros2 run ros_gz parameter_bridge [...]
ros2 run ros_ign parameter_bridge [...]  # Will emit deprecation warning
```

Additionally, installed files like launch files, message interfaces etc. are **duplicated** versions of the ones in `ros_gz` (but renamed as appropriate), and point to `ros_gz` dependencies as well (e.g. launch files pointing to `ros_gz` nodes.)
