# // Max allowable velocity at any point in trajectory.
float64 vel_limit_linear_x
float64 vel_limit_linear_y
float64 vel_limit_angular
     
# Parameters for controlling Spot's body during motion.
# Desired base offset relative to the footprint frame, ewrt the footprint frame.
# Trajectory executed based on header stamps
geometry_msgs/PoseStamped[] base_offset_rt_footprint

# Desired gait during locomotion
int32 HINT_UNKNOWN = 0            # Invalid; do not use.
int32 HINT_AUTO = 1               # No hint, robot chooses an appropriate gait (typically trot.)
int32 HINT_TROT = 2               # Most robust gait which moves diagonal legs together.
int32 HINT_SPEED_SELECT_TROT = 3  # Trot which comes to a stand when not commanded to move.
int32 HINT_CRAWL = 4              # Slow and steady gait which moves only one foot at a time.
int32 HINT_AMBLE = 5              # Four beat gait where one foot touches down at a time.
int32 HINT_SPEED_SELECT_AMBLE = 6 # Amble which comes to a stand when not commanded to move.
int32 HINT_JOG = 7                # Demo gait which moves diagonal leg pairs together with an aerial phase.
int32 HINT_HOP = 8                # Demo gait which hops while holding some feet in the air.

int32 locomotion_hint

# Stairs are only supported in trot gaits. Using this hint will override some user defaults in
# order to optimize stair behavior.
bool stair_hint

# Allow the robot to move with degraded perception when there are perception faults.
bool allow_degraded_perception

# Control of obstacle avoidance.
ObstacleParams obstacle_params

# Swing height setting
int32 SWING_HEIGHT_UNKNOWN = 0 # Invalid; do not use.
int32 SWING_HEIGHT_LOW = 1     # Low-stepping. Robot will try to only swing legs a few cm away from ground.
int32 SWING_HEIGHT_MEDIUM = 2  # Default for most cases, use other values with caution.
int32 SWING_HEIGHT_HIGH = 3    # High-stepping. Possibly useful with degraded vision operation.

int32 swing_height

# Ground terrain parameters.
TerrainParams terrain_params

# Prevent the robot from using StairTracker even if in stairs mode.
bool disallow_stair_tracker

# Robot Body External Force parameters
BodyExternalForceParams external_force_params

# Prevent the robot from pitching to get a better look at rearward terrain except in stairs mode.
bool disallow_non_stairs_pitch_limiting

# Disable the secondary nearmap-based cliff avoidance that runs while on stairs.
bool disable_nearmap_cliff_avoidance
