std_msgs/Header header            # Optional header data

string name                       # Light name

# Light type: constant definition
uint8 POINT       = 0
uint8 SPOT        = 1
uint8 DIRECTIONAL = 2

uint8 type                        # Light type (from constant definitions)

geometry_msgs/Pose pose           # Light pose
std_msgs/ColorRGBA diffuse        # Light diffuse emission
std_msgs/ColorRGBA specular       # Light specular emission
float32 attenuation_constant      # Constant variable in attenuation formula
float32 attenuation_linear        # Linear variable in attenuation formula
float32 attenuation_quadratic     # Quadratic variable in attenuation formula
geometry_msgs/Vector3 direction   # Light direction
float32 range                     # Light range
bool cast_shadows                 # Enable/disable shadow casting
float32 spot_inner_angle          # Spotlight inner cone angle
float32 spot_outer_angle          # Spotlight outer cone angle
float32 spot_falloff              # Falloff between inner and outer cone

uint32 id                         # Unique id of the light

uint32 parent_id                  # Unique id of the light's parent

float32 intensity                 # Light intensity
