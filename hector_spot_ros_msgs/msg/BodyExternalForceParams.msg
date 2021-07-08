# External Force on robot body parameters. This is a beta feature and still can have some odd behaviors.
# By default, the external force estimator is disabled on the robot.

# Indicates what external force estimate/override the robot should use.
# By default, the external force estimator is disabled on the robot.
int32 EXTERNAL_FORCE_NONE = 0         # No external forces considered.
int32 EXTERNAL_FORCE_USE_ESTIMATE = 1 # Use external forces estimated by the robot
int32 EXTERNAL_FORCE_USE_OVERRIDE = 2 # Use external forces specified in an override vector.
        
int32 external_force_indicator

# Frame that the external override force is defined in.
# It is currently only accepted in KO or body frames. 
# Frame frame = 2; # Hardcoded to base_linkk

# Specifies a force that the body should expect to feel. This allows the robot to "lean into"
# an external force. Be careful using this override, since incorrect information can cause
# the robot to fall over.
geometry_msgs/Vector3 external_force_override
