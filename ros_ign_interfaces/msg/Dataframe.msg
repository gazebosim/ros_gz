# Message containing some payload as well as sender and destination
# Intended for use with the comms system in Gazebo
std_msgs/Header header                  # Time stamp
string src_address                      # Address of the sender
string dst_address                      # Address of the destination
uint8[] data                            # Payload
float64 rssi                            # Received Signal Strength Indicator
