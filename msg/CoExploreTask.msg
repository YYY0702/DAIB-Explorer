std_msgs/Header header

# Stable sender identity and reboot/session protection.
uint16 robot_id
uint64 session
uint64 sequence

# Explorer application generation and selected target.
uint64 generation
geometry_msgs/Pose goal
float64 score

# The goal is a short-lived spatial lease, not a permanent task lock.
float64 exclusion_radius_m
time valid_until
bool active
