std_msgs/Header header

# Application-level envelope used by the simulation and multi-host relay.
uint8 DATA=0
uint8 NACK=1
uint8 SUMMARY=2
uint16 BROADCAST=65535

uint8 packet_type
uint16 source_robot_id
uint16 target_robot_id
uint64 session
uint64 sequence

# SUMMARY advertises the newest sequence. NACK requests an inclusive range.
uint64 newest_sequence
uint64 request_from
uint64 request_to

# FNV-1a checksum of the PointCloud2 payload and layout metadata.
uint32 checksum
sensor_msgs/PointCloud2 delta
