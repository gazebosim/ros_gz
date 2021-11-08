string name

# Light type: constant definition
uint8 POINT       = 0
uint8 SPOT        = 1
uint8 DIRECTIONAL = 2

uint8 type

geometry_msgs/Pose pose
ros_ign_interfaces/Color diffuse
ros_ign_interfaces/Color specular
float32 attenuation_constant
float32 attenuation_linear
float32 attenuation_quadratic
geometry_msgs/Vector3 direction
float32 range
bool cast_shadows
float32 spot_inner_angle
float32 spot_outer_angle
float32 spot_falloff

uint32 id

uint32 parent_id

float32 intensity
