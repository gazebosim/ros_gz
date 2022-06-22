# A message that allows for control of video recording functions.

# Optional header data.
std_msgs/Header header

# True to start video recording.
bool start

# True to stop video recording.
bool stop

# Video encoding format, e.g. "mp4", "ogv".
string format

# filename of the recorded video.
string save_filename
