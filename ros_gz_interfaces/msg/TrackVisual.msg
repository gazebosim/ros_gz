# Message for a tracking a rendering::Visual with a rendering::Camera.

# Optional header data.
std_msgs/Header header

# Name of the visual to track.
string name

# Id of the visual to track.
uint32 id

# True to have the tracking camera inherit the orientation of
# the tracked visual.
bool inherit_orientation

# Minimum follow distance.
float64 min_dist

# Maximum follow distance.
float64 max_dist

# If set to true, the position of the camera is fixed.
bool is_static

# If set to true, the position of the camera is relative to the.
# model reference frame.
bool use_model_frame

# Position of the camera.
geometry_msgs/Vector3 xyz

# If set to true, the camera inherits the yaw rotation of the model.
bool inherit_yaw
