string name                             # New name for the entity, overrides the name on the SDF
bool allow_renaming false               # Whether the server is allowed to rename the entity in case of
                                        # overlap with existing entities.

# Only one method is supported at a time (sdf,sdf_filename,clone_name)
string sdf                              # SDF description in string format
string sdf_filename                     # Full path to SDF file.
string clone_name                       # Name of entity to clone

geometry_msgs/Pose pose                 # Pose where the entity will be spawned in the world.
string relative_to "world"              # Pose is defined relative to the frame of this entity. 
