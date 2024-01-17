# Entities that match to apply material color: constant definition
uint8 FIRST     = 0
uint8 ALL       = 1

std_msgs/Header header            # Optional header data
ros_gz_interfaces/Entity entity   # Entity to change material color
std_msgs/ColorRGBA ambient        # Ambient color
std_msgs/ColorRGBA diffuse        # Diffuse color
std_msgs/ColorRGBA specular       # Specular color
std_msgs/ColorRGBA emissive       # Emissive color
float64 shininess                 # Specular exponent
uint8 entity_match                # Entities that match to apply material color
