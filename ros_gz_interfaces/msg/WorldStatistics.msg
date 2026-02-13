# optional header data
std_msgs/Header header

# Current simulation time
builtin_interfaces/Time sim_time

# Total time spent paused
builtin_interfaces/Time pause_time

# Current real time
builtin_interfaces/Time real_time

# Whether currently paused
bool paused

# Current iteration count
uint64 iterations

# Total number of models in the world
int32 model_count

# Statistics for log playback
LogPlaybackStatistics log_playback_statistics

# This factor expresses how much real time elapses with each step
# of simulation time.
float64 real_time_factor

# Iteration step size. It's zero when paused.
builtin_interfaces/Time step_size

# True if the simulator is stepping, as opposed to running. This
# field can be ignored if the paused field is true.
bool stepping
